/**
 * @file bsp_current_sense.c
 * @brief 三相电流采样 BSP 实现（双电阻重构：ADC2 注入组 + OPA1/OPA2 内部运放）。
 * @details 相序相关数据统一按 bsp_current_sense_phase_enum 数组访问。calc() 只完成：
 *          ADC 读取 → 偏置扣除 → 基于 ADC 满量程的电流标幺化 → W 相重构；实际电流
 *          （mA）在 get_current() 中按满量程电流换算。FOC 构建宏选择 Q1.15 或浮点路径.
 */

/*
    待测项:
    1、ADC注入组触发时刻(完成)
    2、电流采样转换值验证(待顶层FOC构建完成后进行验证)

*/
 
#include "bsp_current_sense.h"
#include "bsp_motor_ctrl.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if defined(FOC_USE_Q15) && defined(FOC_USE_FLOAT)
#error "Select only one FOC arithmetic implementation"
#elif !defined(FOC_USE_Q15) && !defined(FOC_USE_FLOAT)
#error "Define FOC_USE_Q15 or FOC_USE_FLOAT in the CMake configuration"
#endif

#define BSP_CURRENT_SENSE_SHUNT_MICRO_OHM     (1000U)
#define BSP_CURRENT_SENSE_GAIN_MILLI          (7333U)
#define BSP_CURRENT_SENSE_ADC_VREF_MV         (3300U)
#define BSP_CURRENT_SENSE_ADC_FULL_SCALE      (4095U)
#define BSP_CURRENT_SENSE_OFFSET_RAW_U        (2048)
#define BSP_CURRENT_SENSE_OFFSET_RAW_V        (2048)
#define BSP_CURRENT_SENSE_INJECTED_RANK_U     (3U)
#define BSP_CURRENT_SENSE_INJECTED_RANK_V     (4U)

/**
 * @brief 电流采样最小有效窗口，单位为 TIM1 计数。
 * @details 只有 PWM 低电平持续计数大于该值时，ADC 采样才认为避开了开关瞬态；
 *          具体 period 与 PWM 模式取自 hal_tim_motor.config（对应 hal_tim.c 的
 *          s_tim_motor_config）。
 */
#define BSP_CURRENT_SENSE_SAMPLE_WINDOW_COUNT (100U)

/** @brief 当前采样窗口有效相位位掩码。 */
#define BSP_CURRENT_SENSE_VALID_PHASE_U       (0x01U)
#define BSP_CURRENT_SENSE_VALID_PHASE_V       (0x02U)
#define BSP_CURRENT_SENSE_VALID_PHASE_W       (0x04U)

static const bsp_current_sense_config_t s_current_sense_config = {
    .reconstruct = bsp_current_sense_reconstruct_dual_resistor,
    .shunt_resistance_micro_ohm = BSP_CURRENT_SENSE_SHUNT_MICRO_OHM,
    .amplifier_gain_milli = BSP_CURRENT_SENSE_GAIN_MILLI,
    .adc_reference_mv = BSP_CURRENT_SENSE_ADC_VREF_MV,
    .adc_full_scale = BSP_CURRENT_SENSE_ADC_FULL_SCALE,
    .offset_raw = {
        [bsp_current_sense_phase_u] = BSP_CURRENT_SENSE_OFFSET_RAW_U,
        [bsp_current_sense_phase_v] = BSP_CURRENT_SENSE_OFFSET_RAW_V,
        [bsp_current_sense_phase_w] = 0
    },
    .injected_rank = {
        [bsp_current_sense_phase_u] = BSP_CURRENT_SENSE_INJECTED_RANK_U,
        [bsp_current_sense_phase_v] = BSP_CURRENT_SENSE_INJECTED_RANK_V,
        [bsp_current_sense_phase_w] = 0
    },
    .timeout_count = 100000U,
    .p_user_data = NULL
};

bsp_current_sense_t bsp_current_sense_three_phase = {0};

typedef char bsp_current_sense_phase_check
    [(bsp_current_sense_phase_count == 3) ? 1 : -1];

/**
 * @brief 校验电流采样板级配置。
 * @details 校验分流电阻、运放增益、ADC 参考参数、ADC 满量程以及 U/V 两路注入序号，
 *          这些字段都会参与除法或硬件通道访问，不能为零。
 * @param p_config 待校验的电流采样配置。
 * @return true 表示配置合法；false 表示参数为空或存在零值配置。
 */
static bool bsp_current_sense_is_config_valid(
    const bsp_current_sense_config_t *p_config)
{
    if ((p_config == NULL) ||
        (p_config->shunt_resistance_micro_ohm == 0U) ||
        (p_config->amplifier_gain_milli == 0U) ||
        (p_config->adc_reference_mv == 0U) ||
        (p_config->adc_full_scale == 0U) ||
        (p_config->injected_rank[bsp_current_sense_phase_u] == 0U) ||
        (p_config->injected_rank[bsp_current_sense_phase_v] == 0U)) {
        return false;
    }

    return true;
}

/**
 * @brief 计算满量程电流，单位 mA。
 * @details I_fs(mA) = (V_ref(V) / gain / R_shunt(ohm)) × 1000。
 */
static uint32_t bsp_current_sense_full_scale_current_ma(
    const bsp_current_sense_config_t *p_config)
{
    uint32_t reference_gain_scaled_mv;
    uint32_t amplifier_input_mv;
    uint32_t current_ma_numerator;

    if ((p_config == NULL) ||
        (p_config->amplifier_gain_milli == 0U) ||
        (p_config->shunt_resistance_micro_ohm == 0U)) {
        return 0U;
    }

    /*
     * 先把参考电压换算成运放输入电压（mV）：
     * amplifier_input_mv = Vref(mV) × 1000 / gain_milli。
     * 再用分流电阻换算为 mA：
     * current_ma = amplifier_input_mv × 1000000 / shunt_micro_ohm。
     * 分步计算避免大数乘法，所有中间量均为 32 位无符号整数。
     */
    reference_gain_scaled_mv =
        (uint32_t)p_config->adc_reference_mv * 1000U;
    amplifier_input_mv =
        (reference_gain_scaled_mv +
         (p_config->amplifier_gain_milli / 2U)) /
        p_config->amplifier_gain_milli;

    if (amplifier_input_mv > (0xFFFFFFFFU / 1000000U)) {
        return 0U;
    }

    current_ma_numerator = amplifier_input_mv * 1000000U;
    return (current_ma_numerator +
            (p_config->shunt_resistance_micro_ohm / 2U)) /
           p_config->shunt_resistance_micro_ohm;
}

/**
 * @brief 读取 ADC 注入组中的 U/V 两相原始计数。
 * @details 本函数只读取由硬件触发链路产生的最近一次转换结果，不主动触发 ADC；
 *          U/V 注入序号从配置数组中取得，W 相由双电阻电流约束重构。
 * @param p_object 电流采样对象指针。
 * @param p_raw_u 输出：U 相 ADC 原始计数。
 * @param p_raw_v 输出：V 相 ADC 原始计数。
 * @retval bsp_current_sense_ok 读取成功。
 * @retval bsp_current_sense_err_param 参数为空。
 * @retval bsp_current_sense_err_state HAL ADC 读取失败。
 */
static bsp_current_sense_status_enum bsp_current_sense_read_raw(
    bsp_current_sense_t *p_object,
    uint16_t *p_raw_u,
    uint16_t *p_raw_v)
{
    if ((p_object == NULL) || (p_raw_u == NULL) || (p_raw_v == NULL)) {
        return bsp_current_sense_err_param;
    }

    if ((hal_adc_ops->pf_get_injected_value(
             &hal_adc_injected,
             p_object->config.injected_rank[bsp_current_sense_phase_u],
             p_raw_u) != hal_adc_ok) ||
        (hal_adc_ops->pf_get_injected_value(
             &hal_adc_injected,
             p_object->config.injected_rank[bsp_current_sense_phase_v],
             p_raw_v) != hal_adc_ok)) {
        return bsp_current_sense_err_state;
    }

    return bsp_current_sense_ok;
}

/**
 * @brief 按相序统一获取标幺值。
 * @details 按 FOC 构建模式读取对应数组成员，不返回临时地址.
 */
static bsp_current_sense_status_enum bsp_current_sense_get_pu_value(
    const bsp_current_sense_t *p_object,
    uint8_t phase,
    bsp_current_sense_value_t *p_value)
{
    if ((p_object == NULL) || (p_value == NULL) ||
        (phase >= (uint8_t)bsp_current_sense_phase_count)) {
        return bsp_current_sense_err_param;
    }

#if defined(FOC_USE_Q15)
    p_value->q_value = p_object->current_q[phase];
#else
    p_value->float_value = p_object->current_a[phase];
#endif
    return bsp_current_sense_ok;
}

/**
 * @brief 把 ADC 计数差转换为 Q1.15 电流标幺值。
 * @details 标幺值定义为 delta / adc_full_scale。PLAT_IQ_DIV() 负责把两个整数按
 *          GLOBAL_Q=15 的格式进行除法，结果再限幅到 [-1.0, 1.0) 对应的
 *          [-32768, 32767]，防止满量程正值溢出 Q1.15 正数范围。
 * @param delta 去偏置后的 ADC 计数差，可为正或负。
 * @param adc_full_scale ADC 满量程计数。
 * @return Q1.15 标幺值。
 */
#if defined(FOC_USE_Q15)
static PLAT_IQ_TYPE bsp_current_sense_pu_to_q15(int32_t delta,
                                                  uint16_t adc_full_scale)
{
    PLAT_IQ_TYPE pu_q = PLAT_IQ_DIV((PLAT_IQ_TYPE)delta,
                                    (PLAT_IQ_TYPE)adc_full_scale);
    PLAT_IQ_TYPE q_max = (PLAT_IQ_TYPE)(PLAT_IQ(1.0f) - 1);
    PLAT_IQ_TYPE q_min = PLAT_IQ(-1.0f);

    if (pu_q > q_max) {
        pu_q = q_max;
    } else if (pu_q < q_min) {
        pu_q = q_min;
    }
    return pu_q;
}
#else
/**
 * @brief 把 ADC 计数差转换为浮点电流标幺值.
 * @details 浮点路径只保存 ADC 基准对应的无量纲小数值，实际 mA 换算延迟到
 *          bsp_current_sense_get_current()，避免采样计算函数承担物理单位换算。
 * @param delta 去偏置后的 ADC 计数差，可为正或负。
 * @param adc_full_scale ADC 满量程计数。
 * @return 浮点标幺值。
 */
static float bsp_current_sense_pu_to_float(int32_t delta, uint16_t adc_full_scale)
{
    /* 浮点路径只保留 ADC 基准对应的标幺小数值，不计算实际电流。 */
    return (float)delta / (float)adc_full_scale;
}
#endif

/**
 * @brief 计算指定导通占空比对应的 PWM 低电平计数.
 * @details bsp_motor_ctrl 保存的是导通占空比，PWM1/PWM2 的比较值极性不同：
 *          PWM1 的低电平计数为 duty，PWM2 的低电平计数为 period - duty。
 *          中心对齐模式下，比较值和 period 使用同一计数单位，故此处直接比较。
 * @param p_object 电流采样对象指针（保留参数用于接口语义一致）。
 * @param duty 导通占空比计数。
 * @return 当前相在一个 PWM 半周期内可用于采样的低电平计数。
 */
static uint16_t bsp_current_sense_get_low_level_count(
    const bsp_current_sense_t *p_object,
    uint16_t duty)
{
    uint16_t period = hal_tim_motor.config.time_base.period;

    (void)p_object;

    if (duty > period) {
        duty = period;
    }

    if (hal_tim_motor.config.pwm.oc_mode == PLAT_TIMER_OC_MODE_PWM1) {
        return (uint16_t)(period - duty);
    }

    return duty;
}

/**
 * @brief 根据三相导通占空比计算当前 ADC 采样窗口的有效相位位图。
 * @details 低电平时间严格大于 BSP_CURRENT_SENSE_SAMPLE_WINDOW_COUNT 才置位；
 *          W 相没有独立 ADC 通道，只有 U/V 两相均有效时才允许使用 W 相重构值。
 * @param p_object 电流采样对象指针。
 * @return U/V/W 有效位图，分别为 0x01/0x02/0x04。
 */
static uint8_t bsp_current_sense_get_valid_phase_mask(
    const bsp_current_sense_t *p_object)
{
    uint8_t valid_mask = 0U;
    uint16_t low_level_count_u;
    uint16_t low_level_count_v;

    low_level_count_u = bsp_current_sense_get_low_level_count(
        p_object, bsp_motor_ctrl_three_phase.duty_u);
    low_level_count_v = bsp_current_sense_get_low_level_count(
        p_object, bsp_motor_ctrl_three_phase.duty_v);

    if (low_level_count_u > BSP_CURRENT_SENSE_SAMPLE_WINDOW_COUNT) {
        valid_mask = (uint8_t)(valid_mask | BSP_CURRENT_SENSE_VALID_PHASE_U);
    }
    if (low_level_count_v > BSP_CURRENT_SENSE_SAMPLE_WINDOW_COUNT) {
        valid_mask = (uint8_t)(valid_mask | BSP_CURRENT_SENSE_VALID_PHASE_V);
    }
    if ((valid_mask & (BSP_CURRENT_SENSE_VALID_PHASE_U |
                       BSP_CURRENT_SENSE_VALID_PHASE_V)) ==
        (BSP_CURRENT_SENSE_VALID_PHASE_U |
         BSP_CURRENT_SENSE_VALID_PHASE_V)) {
        valid_mask = (uint8_t)(valid_mask | BSP_CURRENT_SENSE_VALID_PHASE_W);
    }

    return valid_mask;
}

/**
 * @brief 初始化三相电流采样对象及其硬件链路。
 * @details 初始化顺序为：复制板级配置 → 校验双电阻配置 → 构建 OPA1/OPA2 → 构建
 *          ADC2 注入组 → 计算 mA 满量程 → 清零数组结果 → 进入 ready 状态。
 *          本函数不执行零电流偏置采样，现场偏置更新由 calibrate() 完成。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 初始化成功。
 * @retval bsp_current_sense_err_param 参数或板级配置非法。
 * @retval bsp_current_sense_err_unsupported 当前重构方式未实现。
 * @retval bsp_current_sense_err_state 运放或 ADC 构建失败。
 */
bsp_current_sense_status_enum bsp_current_sense_init(bsp_current_sense_t *p_object)
{
    if (p_object == NULL) {
        return bsp_current_sense_err_param;
    }

    /* 拷贝板级配置，后续 calibrate() 只修改对象内的偏置副本。 */
    p_object->config = s_current_sense_config;
    if (!bsp_current_sense_is_config_valid(&p_object->config)) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_param;
    }
    if (p_object->config.reconstruct != bsp_current_sense_reconstruct_dual_resistor) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_unsupported;
    }

    /* 必须先构建内部运放：ADC 注入引脚复用了 OPA1/OPA2 输出路径。 */
    if ((hal_opamp_ops->pf_build(&hal_opamp_opa1, hal_opamp_role_opa1) != hal_opamp_ok) ||
        (hal_opamp_ops->pf_build(&hal_opamp_opa2, hal_opamp_role_opa2) != hal_opamp_ok)) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }
    /* 构建 ADC2 注入组；实际转换由外部 TIM1_TRGO 触发。 */
    if (hal_adc_ops->pf_build(&hal_adc_injected, hal_adc_role_injected) != hal_adc_ok) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    /* 缓存满量程 mA，get_current() 只做标幺值到实际单位的换算。 */
    p_object->full_scale_current_ma =
        bsp_current_sense_full_scale_current_ma(&p_object->config);

    /* 初始化数组结果，避免在首次 calc() 前读取到未定义数据。 */
    (void)memset(p_object->raw, 0, sizeof(p_object->raw));
    (void)memset(p_object->current_q, 0, sizeof(p_object->current_q));
    (void)memset(p_object->current_a, 0, sizeof(p_object->current_a));
    p_object->valid_phase_mask = 0U;
    p_object->state = bsp_current_sense_state_ready;

    return bsp_current_sense_ok;
}

/**
 * @brief 读取一次 ADC 并更新三相电流标幺值。
 * @details 处理链路：
 *          1) 读取 U/V 两路 ADC 原始计数；
 *          2) 减去各自零电流偏置，得到有符号计数差；
 *          3) W 相按 iu + iv + iw = 0 重构；
 *          4) 按配置将三相计数差转换为 Q1.15 或浮点标幺值。
 *          本函数不计算实际安培或 mA，实际单位换算集中在 get_current()。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 采样和标幺换算成功。
 * @retval bsp_current_sense_err_param 参数或数据类型非法。
 * @retval bsp_current_sense_err_state 对象未初始化或 ADC 读取失败。
 */
bsp_current_sense_status_enum bsp_current_sense_calc(bsp_current_sense_t *p_object)
{
    uint16_t raw_u = 0U;
    uint16_t raw_v = 0U;
    int32_t delta[bsp_current_sense_phase_count];
    uint8_t phase;
    uint8_t valid_phase_mask;

    if (p_object == NULL) {
        return bsp_current_sense_err_param;
    }
    if (p_object->state == bsp_current_sense_state_reset) {
        return bsp_current_sense_err_state;
    }
    if (bsp_current_sense_read_raw(p_object, &raw_u, &raw_v) != bsp_current_sense_ok) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    /* 根据本次 PWM 占空比和 TIM1 PWM 模式计算 ADC 有效采样窗口。 */
    valid_phase_mask = bsp_current_sense_get_valid_phase_mask(p_object);
    p_object->valid_phase_mask = valid_phase_mask;

    /* 保存最近一次 U/V 原始采样，便于标定、调试和故障分析。 */
    p_object->raw[bsp_current_sense_phase_u] = raw_u;
    p_object->raw[bsp_current_sense_phase_v] = raw_v;

    /* ADC 计数减去静态偏置后，符号表示电流方向。 */
    delta[bsp_current_sense_phase_u] =
        (int32_t)raw_u - p_object->config.offset_raw[bsp_current_sense_phase_u];
    delta[bsp_current_sense_phase_v] =
        (int32_t)raw_v - p_object->config.offset_raw[bsp_current_sense_phase_v];
    /* 双电阻重构第三相：iw = -(iu + iv)。 */
    delta[bsp_current_sense_phase_w] =
        -(delta[bsp_current_sense_phase_u] + delta[bsp_current_sense_phase_v]);

#if defined(FOC_USE_Q15)
    for (phase = 0U; phase < (uint8_t)bsp_current_sense_phase_count; phase++) {
        if ((valid_phase_mask & (uint8_t)(1U << phase)) != 0U) {
            p_object->current_q[phase] = bsp_current_sense_pu_to_q15(
                delta[phase], p_object->config.adc_full_scale);
        } else {
            p_object->current_q[phase] = 0;
        }
    }
#else
    for (phase = 0U; phase < (uint8_t)bsp_current_sense_phase_count; phase++) {
        if ((valid_phase_mask & (uint8_t)(1U << phase)) != 0U) {
            p_object->current_a[phase] = bsp_current_sense_pu_to_float(
                delta[phase], p_object->config.adc_full_scale);
        } else {
            p_object->current_a[phase] = 0.0f;
        }
    }
#endif

    p_object->state = bsp_current_sense_state_running;
    return bsp_current_sense_ok;
}

/**
 * @brief 标定 U/V 两相零电流 ADC 偏置。
 * @details 调用时必须保证电机无电流、PWM 输出已关闭。当前 ADC 注入值被保存到 raw 数组，
 *          同时覆盖 U/V 配置偏置；W 相没有独立 ADC 通道，其偏置保持为 0，继续由两相重构。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 标定成功。
 * @retval bsp_current_sense_err_param 对象为空或尚未初始化。
 * @retval bsp_current_sense_err_state ADC 读取失败。
 */
bsp_current_sense_status_enum bsp_current_sense_calibrate(
    bsp_current_sense_t *p_object)
{
    uint16_t raw_u = 0U;
    uint16_t raw_v = 0U;

    if ((p_object == NULL) || (p_object->state == bsp_current_sense_state_reset)) {
        return bsp_current_sense_err_param;
    }
    if (bsp_current_sense_read_raw(p_object, &raw_u, &raw_v) != bsp_current_sense_ok) {
        p_object->state = bsp_current_sense_state_error;
        return bsp_current_sense_err_state;
    }

    /* 保存标定采样值，并将其作为后续 calc() 使用的零电流基准。 */
    p_object->raw[bsp_current_sense_phase_u] = raw_u;
    p_object->raw[bsp_current_sense_phase_v] = raw_v;
    p_object->config.offset_raw[bsp_current_sense_phase_u] = (int32_t)raw_u;
    p_object->config.offset_raw[bsp_current_sense_phase_v] = (int32_t)raw_v;

    return bsp_current_sense_ok;
}

/**
 * @brief 获取指定相的标幺电流及实际 mA 电流。
 * @details 先通过统一取值函数读取当前相的 Q1.15 或浮点标幺值，再按配置的满量程 mA
 *          换算实际电流。Q 路径使用 PLAT_IQ_MPY_I32_INT() 取整数部分，浮点路径
 *          直接转换为 int32_t；两条路径均保持电流方向符号。
 * @param p_object 电流采样对象指针。
 * @param phase 相序号（U/V/W）。
 * @param p_pu 输出：标幺电流联合体。
 * @param p_current_ma 输出：实际电流，单位 mA，取整数。
 * @retval bsp_current_sense_ok 获取成功。
 * @retval bsp_current_sense_err_param 参数为空或相序号越界。
 * @retval bsp_current_sense_err_state 数据类型不支持。
 */
bsp_current_sense_status_enum bsp_current_sense_get_current(
    const bsp_current_sense_t *p_object,
    uint8_t phase,
    bsp_current_sense_value_t *p_pu,
    int32_t *p_current_ma)
{
    bsp_current_sense_status_enum status;

    if ((p_object == NULL) || (p_pu == NULL) || (p_current_ma == NULL)) {
        return bsp_current_sense_err_param;
    }

    status = bsp_current_sense_get_pu_value(p_object, phase, p_pu);
    if (status != bsp_current_sense_ok) {
        return status;
    }

#if defined(FOC_USE_Q15)
    *p_current_ma = PLAT_IQ_MPY_I32_INT(
        p_pu->q_value,
        (int32_t)p_object->full_scale_current_ma);
#else
    *p_current_ma = (int32_t)(p_pu->float_value *
                              (float)p_object->full_scale_current_ma);
#endif

    return bsp_current_sense_ok;
}

/**
 * @brief 查询电流采样对象的生命周期状态。
 * @param p_object 电流采样对象指针。
 * @return 当前状态；参数为空时返回 bsp_current_sense_state_error。
 */
bsp_current_sense_state_enum bsp_current_sense_get_state(
    const bsp_current_sense_t *p_object)
{
    if (p_object == NULL) {
        return bsp_current_sense_state_error;
    }
    return p_object->state;
}

/**
 * @brief 电流采样 BSP 函数表。
 * @details 统一绑定初始化、采样换算、偏置标定、标幺/mA 读取和状态查询接口。
 */
static const bsp_current_sense_ops_t s_bsp_current_sense_ops = {
    .pf_init = bsp_current_sense_init,
    .pf_calc = bsp_current_sense_calc,
    .pf_calibrate = bsp_current_sense_calibrate,
    .pf_get_current = bsp_current_sense_get_current,
    .pf_get_state = bsp_current_sense_get_state
};

const bsp_current_sense_ops_t *const bsp_current_sense_ops = &s_bsp_current_sense_ops;
