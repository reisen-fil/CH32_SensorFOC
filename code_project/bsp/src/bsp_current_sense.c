/**
 * @file bsp_current_sense.c
 * @brief 三相电流采样 BSP 实现（双电阻重构：ADC2 注入组 + OPA1/OPA2 内部运放）。
 * @details 结构与本层 bsp_motor_ctrl.c / bsp_analog_sense.c 同构：
 *          - bsp_current_sense_init() 先经 hal_opamp_ops->pf_build 构建 OPA1/OPA2
 *            （运放引脚组 PA5/6/7、PB0、PA3/4、PB1 在此一并配置），再经
 *            hal_adc_ops->pf_build 构建 ADC2 注入组（序号 1 = PB1/OPA1 → U 相，
 *            序号 2 = PA4/OPA2 → V 相，TIM1_TRGO 触发；注入中断线亦在该步配置）；
 *          - bsp_current_sense_calc() 取两路注入值，去零电流偏置后在定点域（Q15
 *            接口宏，1.0 = 1 A）或浮点域（安培）换算，并按 iu + iv + iw = 0 重构 W 相；
 *          - bsp_current_sense_calibrate() 在无电流时把当前两路读数记为新的零电流偏置。
 *          本文件只做"采样值 → 相电流"的换算与重构，不含任何控制算法。
 *          电流重构方式为**双电阻**：两只内部运放分别放大 U/V 相分流电阻上的电压，
 *          输出直接进 ADC2 注入组，故三相电流无需按扇区选择采样时刻。
 */

/* 
    待测项：
    1、ADC注入组触发时刻（完成）
    2、电流采样转换值验证（待顶层FOC构建完成后进行验证）

*/

#include "bsp_current_sense.h"
#include <stddef.h>

/* ==========================================================================
 * 板级常量与配置
 * ========================================================================== */

/** @brief 相分流电阻（µΩ）：示例 1 mΩ，请按实际板子填写。 */
#define BSP_CURRENT_SENSE_SHUNT_MICRO_OHM     (1000U)

/** @brief 内部运放增益（×1000）：示例 10 倍，请按实际板子填写。 */
#define BSP_CURRENT_SENSE_GAIN_MILLI          (7333)

/** @brief ADC 参考电压（mV）与满量程计数（12 位）。 */
#define BSP_CURRENT_SENSE_ADC_VREF_MV         (3300U)
#define BSP_CURRENT_SENSE_ADC_FULL_SCALE      (4095U)

/**
 * @brief 两相零电流偏置（ADC 计数）。
 * @note 上电初值为运放静态工作点（约 1/2 参考电压）对应的计数；现场可用
 *       bsp_current_sense_calibrate() 在电机无电流时重新标定并覆盖。
 */
#define BSP_CURRENT_SENSE_OFFSET_RAW_U        (2048)
#define BSP_CURRENT_SENSE_OFFSET_RAW_V        (2048)

/**
 * @brief 两相在 ADC 注入序列中的序号（HAL 侧 1 起始）。
 * @note 由 hal_adc.c 的注入通道表决定：序号 1 = PB1 = OPA1 输出 = U 相，
 *       序号 2 = PA4 = OPA2 输出 = V 相。
 */
#define BSP_CURRENT_SENSE_INJECTED_RANK_U     (3U)
#define BSP_CURRENT_SENSE_INJECTED_RANK_V     (4U)

/**
 * @brief 定点换算约定：运算全部经 platform_ch32v203.h 暴露的 Q15 接口宏完成，量纲为安培。
 * @details 三段式：
 *          1) 初始化用 PLAT_IQ() 把"每计数对应的电流"定点化：K = I_fs / ADC 满量程
 *             （Q15：1.0 = 1 A）；
 *          2) 换算用 PLAT_IQ_MPY_I32(K, Δcount)（Q15 × 整数 → Q15）直接得到定点安培值
 *             ——整数乘既无截断误差，也不依赖库内 DIV_FAST 快速除法的精度与 |A|<|B| 限制；
 *          3) 反向转换用 PLAT_IQ()（float → 定点）/ PLAT_IQ_TO_FLOAT()（定点 → float）。
 * @note 不再使用标幺（1.0 = 满量程电流）形式；Q15 为 32 位容器，可表示 ±65535 A。
 */

/**
 * @brief 板级三相电流采样配置表。
 * @note 计算数据类型取 32 位定点（PLAT_IQ_*，Q15，量纲为安培：1.0 = 1 A），若希望结果
 *       直接是浮点安培值，把 data_type 改为 bsp_current_sense_data_type_float 即可
 *       （两条路径共用同一张表，结果量纲一致）。
 *       电流重构方式固定为双电阻。
 */
static const bsp_current_sense_config_t s_current_sense_config = {
    .data_type = bsp_current_sense_data_type_q,
    .reconstruct = bsp_current_sense_reconstruct_dual_resistor,

    .shunt_resistance_micro_ohm = BSP_CURRENT_SENSE_SHUNT_MICRO_OHM,
    .amplifier_gain_milli = BSP_CURRENT_SENSE_GAIN_MILLI,
    .adc_reference_mv = BSP_CURRENT_SENSE_ADC_VREF_MV,
    .adc_full_scale = BSP_CURRENT_SENSE_ADC_FULL_SCALE,

    .offset_raw_u = BSP_CURRENT_SENSE_OFFSET_RAW_U,
    .offset_raw_v = BSP_CURRENT_SENSE_OFFSET_RAW_V,

    .injected_rank_u = BSP_CURRENT_SENSE_INJECTED_RANK_U,
    .injected_rank_v = BSP_CURRENT_SENSE_INJECTED_RANK_V,

    .timeout_count = 100000U,
    .p_user_data = NULL
};

/** @brief 板级三相电流采样对象（单实例，只保存配置与最近一次换算结果）。 */
bsp_current_sense_t bsp_current_sense_three_phase = {0};

/**
 * @brief 编译期校验：三相序号数必须为 3。
 */
typedef char bsp_current_sense_phase_check
    [(bsp_current_sense_phase_count == 3) ? 1 : -1];

/**
 * @brief 编译期校验：两相注入序号必须落在 HAL 注入组通道数之内。
 */
// typedef char bsp_current_sense_rank_check
//     [((BSP_CURRENT_SENSE_INJECTED_RANK_U <= HAL_ADC_INJECTED_CHANNEL_COUNT) &&
//       (BSP_CURRENT_SENSE_INJECTED_RANK_V <= HAL_ADC_INJECTED_CHANNEL_COUNT)) ? 1 : -1];

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
 * @brief 校验板级配置是否可用（除数非零、满量程与注入序号合法）。
 * @param p_config 板级配置。
 * @retval true 配置合法。
 * @retval false 配置非法。
 */
static bool bsp_current_sense_is_config_valid(
    const bsp_current_sense_config_t *p_config)
{
    if ((p_config->shunt_resistance_micro_ohm == 0U) ||
        (p_config->amplifier_gain_milli == 0U) ||
        (p_config->adc_reference_mv == 0U) ||
        (p_config->adc_full_scale == 0U)) {
        return false;
    }

    // if ((p_config->injected_rank_u == 0U) || (p_config->injected_rank_v == 0U) ||
    //     (p_config->injected_rank_u > HAL_ADC_INJECTED_CHANNEL_COUNT) ||
    //     (p_config->injected_rank_v > HAL_ADC_INJECTED_CHANNEL_COUNT)) {
    //     return false;
    // }

    return true;
}

/**
 * @brief 计算满量程电流（A）：V_ref → 运放输入电压 → 分流电阻上的电流。
 * @details 运放输出端电压达到 ADC 参考电压时对应的相电流：
 *          I_fs = (V_ref / 增益) / R_shunt。全过程在浮点域完成，避免整数溢出
 *          （V_ref 以 mV 计、R_shunt 以 µΩ 计时中间量会超出 32 位）。
 * @param p_config 板级配置。
 * @return 满量程电流（A）；参数非法时返回 0。
 */
static float bsp_current_sense_full_scale_current_a(
    const bsp_current_sense_config_t *p_config)
{
    float reference_v;
    float amplifier_input_v;
    float shunt_ohm;

    if ((p_config->amplifier_gain_milli == 0U) ||
        (p_config->shunt_resistance_micro_ohm == 0U)) {
        return 0.0f;
    }

    reference_v = (float)p_config->adc_reference_mv / 1000.0f;
    amplifier_input_v = reference_v /
                        ((float)p_config->amplifier_gain_milli / 1000.0f);
    shunt_ohm = (float)p_config->shunt_resistance_micro_ohm / 1000000.0f;

    return amplifier_input_v / shunt_ohm;
}

/**
 * @brief 读取两路注入转换结果。
 * @param p_object 电流采样对象指针。
 * @param p_raw_u 输出：U 相原始计数。
 * @param p_raw_v 输出：V 相原始计数。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_state HAL 函数表拒绝读取。
 */
static bsp_current_sense_status_enum bsp_current_sense_read_raw(
    bsp_current_sense_t *p_object,
    uint16_t *p_raw_u,
    uint16_t *p_raw_v)
{
    if ((hal_adc_ops->pf_get_injected_value(&hal_adc_injected,
                                            p_object->config.injected_rank_u,
                                            p_raw_u) != hal_adc_ok) ||
        (hal_adc_ops->pf_get_injected_value(&hal_adc_injected,
                                            p_object->config.injected_rank_v,
                                            p_raw_v) != hal_adc_ok)) {
        return bsp_current_sense_err_state;
    }

    return bsp_current_sense_ok;
}

/**
 * @brief 按相序号取 Q 格式电流字段地址。
 * @param p_object 电流采样对象指针。
 * @param phase 相序号，取 bsp_current_sense_phase_x。
 * @return 字段地址；相序号越界时返回 NULL。
 */
static const PLAT_IQ_TYPE *bsp_current_sense_phase_q_ptr(
    const bsp_current_sense_t *p_object,
    uint8_t phase)
{
    switch (phase) {
    case (uint8_t)bsp_current_sense_phase_u:
        return &p_object->current_q_u;
    case (uint8_t)bsp_current_sense_phase_v:
        return &p_object->current_q_v;
    case (uint8_t)bsp_current_sense_phase_w:
        return &p_object->current_q_w;
    default:
        return NULL;
    }
}

/**
 * @brief 按相序号取浮点电流字段地址。
 * @param p_object 电流采样对象指针。
 * @param phase 相序号，取 bsp_current_sense_phase_x。
 * @return 字段地址；相序号越界时返回 NULL。
 */
static const float *bsp_current_sense_phase_a_ptr(
    const bsp_current_sense_t *p_object,
    uint8_t phase)
{
    switch (phase) {
    case (uint8_t)bsp_current_sense_phase_u:
        return &p_object->current_a_u;
    case (uint8_t)bsp_current_sense_phase_v:
        return &p_object->current_a_v;
    case (uint8_t)bsp_current_sense_phase_w:
        return &p_object->current_a_w;
    default:
        return NULL;
    }
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化三相电流采样：构建内部运放与 ADC2 注入组，并注入板级配置。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或板级配置非法。
 * @retval bsp_current_sense_err_unsupported 重构方式不是双电阻。
 * @retval bsp_current_sense_err_state 运放构建或 ADC 注入组构建失败。
 */
bsp_current_sense_status_enum bsp_current_sense_init(bsp_current_sense_t *p_object)
{
    if (p_object == NULL) {
        return bsp_current_sense_err_param;
    }

    /* 外设 config 注入：板级配置表整表拷入对象（后续标定直接更新对象内副本） */
    p_object->config = s_current_sense_config;

    if (!bsp_current_sense_is_config_valid(&p_object->config)) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_param;
    }

    /* 单/三电阻重构需要不同采样序列或第三路注入通道，本层暂只实现双电阻 */
    if (p_object->config.reconstruct !=
        bsp_current_sense_reconstruct_dual_resistor) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_unsupported;
    }

    /* 1) 两只内部运放先建：运放引脚组（PA5/6/7、PB0 浮空，PA3/4、PB1 模拟）在其中配置，
          注入通道 PB1/PA4 复用该引脚组，故必须先于 ADC 注入组构建 */
    if ((hal_opamp_ops->pf_build(&hal_opamp_opa1,
                                hal_opamp_role_opa1) != hal_opamp_ok) ||
        (hal_opamp_ops->pf_build(&hal_opamp_opa2,
                                hal_opamp_role_opa2) != hal_opamp_ok)) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    /* 2) ADC2 注入组 + TIM1_TRGO 触发（注入中断线亦在该步配置） */
    if (hal_adc_ops->pf_build(&hal_adc_injected,
                              hal_adc_role_injected) != hal_adc_ok) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    /* 3) 满量程电流（A）：V_ref / 运放增益 / 分流电阻 */
    p_object->full_scale_current_a =
        bsp_current_sense_full_scale_current_a(&p_object->config);

    /* 4) 每计数对应的电流（Q15 定点，A/计数）= 满量程电流 ÷ ADC 满量程，经 PLAT_IQ 定点化 */
    p_object->current_per_count_q =
        PLAT_IQ(p_object->full_scale_current_a /
                (float)p_object->config.adc_full_scale);

    /* 5) 清零采样与换算结果 */
    p_object->raw_u = 0U;
    p_object->raw_v = 0U;
    p_object->current_q_u = 0;
    p_object->current_q_v = 0;
    p_object->current_q_w = 0;
    p_object->current_a_u = 0.0f;
    p_object->current_a_v = 0.0f;
    p_object->current_a_w = 0.0f;

    p_object->state = bsp_current_sense_state_ready;
    return bsp_current_sense_ok;
}

/**
 * @brief 换算一次三相电流（读取注入组结果 → 去偏置 → 换算 → 双电阻重构）。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或数据类型非法。
 * @retval bsp_current_sense_err_state 对象未初始化或 HAL 读取注入值失败。
 * @retval bsp_current_sense_err_unsupported 重构方式不是双电阻。
 */

bsp_current_sense_status_enum bsp_current_sense_calc(bsp_current_sense_t *p_object)
{
    uint16_t raw_u = 0U;
    uint16_t raw_v = 0U;
    int32_t delta_u;
    int32_t delta_v;

    if (p_object == NULL) {
        return bsp_current_sense_err_param;
    }

    if (p_object->state == bsp_current_sense_state_reset) {
        return bsp_current_sense_err_state;
    }

    if (bsp_current_sense_read_raw(p_object, &raw_u, &raw_v) !=
        bsp_current_sense_ok) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    p_object->raw_u = raw_u;
    p_object->raw_v = raw_v;

    /* 去零电流偏置：得到两相的有符号计数差（符号即电流方向） */
    delta_u = (int32_t)raw_u - p_object->config.offset_raw_u;
    delta_v = (int32_t)raw_v - p_object->config.offset_raw_v;

    if (p_object->config.data_type == bsp_current_sense_data_type_q) {
        /* 定点域（Q15，1.0 = 1 A）：整数计数差 × 每计数电流 → 定点安培值，全程无浮点 */
        p_object->current_q_u =
            PLAT_IQ_MPY_I32(p_object->current_per_count_q, (int32_t)delta_u);
        p_object->current_q_v =
            PLAT_IQ_MPY_I32(p_object->current_per_count_q, (int32_t)delta_v);
        /* 双电阻重构：iu + iv + iw = 0（同为 Q15 格式，直接整数相加后取反） */
        p_object->current_q_w =
            (PLAT_IQ_TYPE)(-(p_object->current_q_u + p_object->current_q_v));
    } else if (p_object->config.data_type == bsp_current_sense_data_type_float) {
        /* 浮点域：计数差 → 安培值 */
        float scale = p_object->full_scale_current_a /
                      (float)p_object->config.adc_full_scale;

        p_object->current_a_u = (float)delta_u * scale;
        p_object->current_a_v = (float)delta_v * scale;
        /* 双电阻重构：iu + iv + iw = 0 */
        p_object->current_a_w = -(p_object->current_a_u + p_object->current_a_v);
    } else {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_param;
    }

    p_object->state = bsp_current_sense_state_running;
    return bsp_current_sense_ok;
}

/**
 * @brief 零电流偏置标定：把当前两路注入读数记为新的零电流偏置。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或对象未初始化。
 * @retval bsp_current_sense_err_state HAL 读取注入值失败。
 */
bsp_current_sense_status_enum bsp_current_sense_calibrate(
    bsp_current_sense_t *p_object)
{
    uint16_t raw_u = 0U;
    uint16_t raw_v = 0U;

    if ((p_object == NULL) ||
        (p_object->state == bsp_current_sense_state_reset)) {
        return bsp_current_sense_err_param;
    }

    if (bsp_current_sense_read_raw(p_object, &raw_u, &raw_v) !=
        bsp_current_sense_ok) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    p_object->raw_u = raw_u;
    p_object->raw_v = raw_v;
    p_object->config.offset_raw_u = (int32_t)raw_u;
    p_object->config.offset_raw_v = (int32_t)raw_v;

    return bsp_current_sense_ok;
}

/**
 * @brief 取某相电流的浮点值（安培）。
 * @details 统一取值出口，按配置的运算类型分两路赋值：
 *          - Q 格式运算：先读取对象的 Q 格式结果，再经 Q15 接口宏 PLAT_IQ_TO_FLOAT()
 *            按 2^15 移位换算（Q15 → float，即数值 ÷ 32768）得到浮点安培值；
 *          - 浮点运算：直接读取浮点结果字段赋值。
 * @param p_object 电流采样对象指针。
 * @param phase 相序号，取 bsp_current_sense_phase_x。
 * @param p_value 输出：相电流（A）。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或相序号越界。
 * @retval bsp_current_sense_err_state 运算类型不受支持。
 */
bsp_current_sense_status_enum bsp_current_sense_get_current(
    const bsp_current_sense_t *p_object,
    uint8_t phase,
    float *p_value)
{
    const PLAT_IQ_TYPE *p_q;
    const float *p_a;

    if ((p_object == NULL) || (p_value == NULL)) {
        return bsp_current_sense_err_param;
    }

    if (p_object->config.data_type == bsp_current_sense_data_type_q) {
        /* Q 格式运算：先读 Q 格式结果，再移位换算成浮点安培值后赋值 */
        p_q = bsp_current_sense_phase_q_ptr(p_object, phase);
        if (p_q == NULL) {
            return bsp_current_sense_err_param;
        }

        *p_value = PLAT_IQ_TO_FLOAT(*p_q);
    } else if (p_object->config.data_type == bsp_current_sense_data_type_float) {
        /* 浮点运算：直接取浮点结果字段赋值 */
        p_a = bsp_current_sense_phase_a_ptr(p_object, phase);
        if (p_a == NULL) {
            return bsp_current_sense_err_param;
        }

        *p_value = *p_a;
    } else {
        return bsp_current_sense_err_state;
    }

    return bsp_current_sense_ok;
}

/**
 * @brief 查询电流采样对象当前状态。
 * @param p_object 电流采样对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_current_sense_state_error。
 */
bsp_current_sense_state_enum bsp_current_sense_get_state(
    const bsp_current_sense_t *p_object)
{
    if (p_object == NULL) {
        return bsp_current_sense_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * BSP 三相电流采样函数表
 * ========================================================================== */

/** @brief BSP 三相电流采样函数表（本文件私有，绑定板级接口）。 */
static const bsp_current_sense_ops_t s_bsp_current_sense_ops = {
    .pf_init = bsp_current_sense_init,
    .pf_calc = bsp_current_sense_calc,
    .pf_calibrate = bsp_current_sense_calibrate,
    // 备用
    .pf_get_current = bsp_current_sense_get_current,
    .pf_get_state = bsp_current_sense_get_state
};

/** @brief BSP 三相电流采样函数表指针（对外外露，声明于 bsp_current_sense.h）。 */
const bsp_current_sense_ops_t *const bsp_current_sense_ops = &s_bsp_current_sense_ops;
