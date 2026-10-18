/**
 * @file bsp_encoder.c
 * @brief MT6701 磁编码器 BSP 实现（接口对 HAL GPIO/SPI 函数表的简单封装）。
 * @details 结构与本层 bsp_led.c 同构：
 *          - bsp_encoder_mt6701_init() 经 HAL GPIO 函数表 hal_gpio_ops->pf_build 构建
 *            SPI 引脚组，再经 HAL SPI 函数表 hal_spi_ops->pf_build 构建 SPI 实例；
 *          - bsp_encoder_mt6701_get_mechanical_angle() 经 hal_spi_ops->pf_receive 采样一帧，
 *            先对 24 位帧的 18 位数据域做 CRC-6 校验（bsp_encoder_mt6701_crc6_itu），
 *            校验通过后才取原始机械角度与 Mg 状态位，再做方向归一与零点对齐；
 *          - bsp_encoder_mt6701_get_electrical_angle() 先调用机械角度获取函数，再按极对数折算电角度；
 *          - bsp_encoder_mt6701_get_angle_pu() 按角度种类读取标幺值（统一浮点出口）；
 *          - bsp_encoder_get_velocity() 差分计算机械转速（通用接口，不绑定传感器）。
 *          角度信息全部保存于 config 结构体中：计数域为原始计数，标幺域为 0 ~ 1
 *          （Q15 或浮点，由 config.arithmetic_type 选择），本文件不做任何弧度换算。
 */

/* 
    待测项：
    1、基本实现原始角度+参数读取（完成）
    2、定点化后处理：计数 → 标幺值（Q 格式/浮点两条路径）已实现，并已通过上位机读取角度值核对（完成）；对于零点对齐部分角度，待顶层FOC构建完成后再进行验证

    对于差分计算机械转速部分，待后续进行速度环控制时再进行适配与更改；

*/

#include "bsp_encoder.h"
#include <stddef.h>

/* ==========================================================================
 * 板级常量与配置
 * ========================================================================== */

/** @brief 一次 SPI 事务的字节数（24 位帧占前 3 字节，第 4 字节为余量）。 */
#define BSP_MT6701_TRANSACTION_BYTES       (4U)

/** @brief 18 位数据域（帧 bit[23:6]）在 24 位帧中的起始位。 */
#define BSP_MT6701_CRC_DATA_SHIFT          (6U)
/** @brief CRC-6 域掩码（帧 bit[5:0]）。 */
#define BSP_MT6701_CRC_MASK                (0x3FU)
/** @brief CRC 校验输入分组数（18 位数据 = 3 组 6 位）。 */
#define BSP_MT6701_CRC_INPUT_BYTES         (3U)
/** @brief 14 位角度在 18 位数据域中的右移位数。 */
#define BSP_MT6701_ANGLE_DATA_SHIFT        (4U)
/** @brief Mg 状态位掩码（18 位数据域低 4 位 = 帧 bit[9:6]）。 */
#define BSP_MT6701_MG_STATUS_MASK          (0x0FU)

/** @brief CRC-6 多项式（x^6 + x + 1）。 */
#define BSP_MT6701_CRC6_POLYNOMIAL         (0x03U)
/** @brief CRC-6 寄存器最高位掩码（bit5）。 */
#define BSP_MT6701_CRC6_MSB_MASK           (0x20U)
/** @brief CRC-6 每字节位移位数（该函数只处理每字节的低 6 位）。 */
#define BSP_MT6701_CRC6_SHIFT_COUNT        (6U)

/** @brief MT6701 单圈角度分辨率（位）。 */
#define BSP_MT6701_ANGLE_RESOLUTION_BITS   (14U)
/** @brief MT6701 单圈角度计数。 */
#define BSP_MT6701_ANGLE_RESOLUTION        (1UL << BSP_MT6701_ANGLE_RESOLUTION_BITS)
/** @brief 单圈角度计数掩码。 */
#define BSP_MT6701_ANGLE_MASK              (BSP_MT6701_ANGLE_RESOLUTION - 1UL)

/**
 * @brief 每计数对应的标幺量（Q15 定点，= 1 / 单圈计数）。
 * @note 经平台 Q15 接口 PLAT_IQ() 取得；Q 格式运算下 标幺值 = 本常量 × 计数
 *       （PLAT_IQ_MPY_I32 整数乘）。14 位分辨率时 1/16384 在 Q15 中恰为 2，
 *       故"计数 → 标幺"的定标换算精确、无截断误差。
 */
#define BSP_MT6701_PU_PER_COUNT_Q          PLAT_IQ(1.0f / (float)BSP_MT6701_ANGLE_RESOLUTION)     // 需要根据取的定点运算小数位数确认存储精度是否过低

/**
 * @brief 板级电机极对数（电角度计数 = 机械角度计数 × 极对数，取单圈）。
 * @note 按实际电机填写；0 表示未配置，此时电角度恒为 0。
 */
#define BSP_ENCODER_MOTOR_POLE_PAIRS       (7U)

/**
 * @brief 板级采样周期（µs），差分测速用。
 * @note 应填写实际调用机械角度获取函数的周期（如 6kHz 控制环填 167）；
 *       0 表示未配置，此时 bsp_encoder_get_velocity() 返回 bsp_encoder_err_state。
 */
#define BSP_ENCODER_SAMPLE_PERIOD_US       (0U)

/** @brief 板级 MT6701 编码器配置（来源：MT6701 手册与本层原有配置；采样信息初值为 0）。 */
static const bsp_encoder_config_t s_mt6701_config = {
    /* 配置 */
    .encoder_type = bsp_encoder_type_mt6701,
    .interface_type = bsp_encoder_interface_spi,
    .motor_pole_pairs = BSP_ENCODER_MOTOR_POLE_PAIRS,
    .direction = bsp_encoder_direction_reverse,
    .resolution_bits = BSP_MT6701_ANGLE_RESOLUTION_BITS,
    .arithmetic_type = bsp_encoder_arithmetic_q,
    .zero_offset = 0U,
    .sample_period_us = BSP_ENCODER_SAMPLE_PERIOD_US,
    .timeout_count = 10000U,
    .is_absolute = true,
    /* 角度信息（原始计数，初始化时清零） */
    .raw_mechanical_angle = 0U,
    .raw_electrical_angle = 0U,
    .zero_aligned_mechanical_angle = 0U,
    .zero_aligned_electrical_angle = 0U,
    .raw_mechanical_angle_pu = {.q_value = 0},
    .zero_aligned_mechanical_angle_pu = {.q_value = 0},
    .raw_electrical_angle_pu = {.q_value = 0},
    .zero_aligned_electrical_angle_pu = {.q_value = 0},
    .mg_status = {.value = 0U},
    /* 零点对齐完成标志（初始化时为未对齐） */
    .is_zero_aligned = false,
    /* 测速状态（原始计数域，初始化时清零） */
    .previous_mechanical_angle = 0U,
    .mechanical_velocity = 0
};

/** @brief 板级 MT6701 SPI 编码器对象（单实例，只保存配置与采样信息）。 */
bsp_encoder_t bsp_encoder_mt6701 = {0};

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
 * @brief 计算单圈角度计数差（整数域），消除 0/满量程跨越带来的跳变。
 * @param current 本次机械角度计数。
 * @param previous 上次机械角度计数。
 * @return 折算到 (-半圈, 半圈] 的计数差。
 */
static int32_t bsp_encoder_count_delta(uint16_t current, uint16_t previous)
{
    int32_t delta = (int32_t)current - (int32_t)previous;
    int32_t half_turn = (int32_t)(BSP_MT6701_ANGLE_RESOLUTION / 2UL);

    if (delta > half_turn) {
        delta -= (int32_t)BSP_MT6701_ANGLE_RESOLUTION;
    } else if (delta < -half_turn) {
        delta += (int32_t)BSP_MT6701_ANGLE_RESOLUTION;
    }

    return delta;
}

/**
 * @brief MT6701 CRC-6/ITU 校验（数据手册示例函数，单独封装以便复用与自检）。
 * @details 多项式 x^6 + x + 1（0x03）、初值 0、不反射、无输出异或；
 *          每字节只处理其低 6 位（高位在 6 次移位中移出寄存器），
 *          故 18 位数据域需按"3 组 6 位、MSB 优先"的顺序传入。
 * @param p_data 数据字节数组（每字节仅低 6 位有效）。
 * @param length 字节数。
 * @return 计算得到的 CRC-6 值（低 6 位）。
 */
static uint8_t bsp_encoder_mt6701_crc6_itu(const uint8_t *p_data,
                                           uint32_t length)
{
    uint8_t crc = 0U;
    uint32_t byte_index;
    uint8_t bit_index;

    for (byte_index = 0U; byte_index < length; byte_index++) {
        crc ^= p_data[byte_index];
        for (bit_index = BSP_MT6701_CRC6_SHIFT_COUNT; bit_index > 0U; bit_index--) {
            if ((crc & BSP_MT6701_CRC6_MSB_MASK) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ BSP_MT6701_CRC6_POLYNOMIAL);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return (uint8_t)(crc & BSP_MT6701_CRC_MASK);
}

/**
 * @brief 把单圈角度计数按配置的运算方式换算成标幺值（0 ~ 1）并写入标幺字段。
 * @details - Q 格式运算：标幺值 = 计数 ÷ 分辨率，全程定点——先用 PLAT_IQ() 把
 *              "每计数对应的标幺量"定点化，再用 PLAT_IQ_MPY_I32()（Q15 × 整数 → Q15）
 *              做整数乘，无浮点参与（14 位分辨率下换算精确，见 BSP_MT6701_PU_PER_COUNT_Q）；
 *            - 浮点运算：标幺值 = 计数 ÷ 分辨率（浮点域）。
 * @param p_config 编码器配置（含运算方式）。
 * @param counts 单圈角度计数（0 … 分辨率 - 1）。
 * @param p_pu 输出：标幺值字段（按运算方式写入对应成员）。
 */
static void bsp_encoder_counts_to_pu(const bsp_encoder_config_t *p_config,
                                     uint32_t counts,
                                     bsp_encoder_pu_angle_t *p_pu)
{
    if (p_config->arithmetic_type == bsp_encoder_arithmetic_q) {
        p_pu->q_value =
            PLAT_IQ_MPY_I32(BSP_MT6701_PU_PER_COUNT_Q, (int32_t)counts);
    } else {
        p_pu->float_value =
            (float)counts / (float)BSP_MT6701_ANGLE_RESOLUTION;
    }
}

/**
 * @brief 按运算方式把标幺字段转换成浮点标幺值（统一浮点出口的内部实现）。
 * @param p_config 编码器配置（含运算方式）。
 * @param p_pu 标幺值字段。
 * @return 标幺角度（0.0f ~ 1.0f）。
 */
static float bsp_encoder_pu_to_float(const bsp_encoder_config_t *p_config,
                                     const bsp_encoder_pu_angle_t *p_pu)
{
    if (p_config->arithmetic_type == bsp_encoder_arithmetic_q) {
        return PLAT_IQ_TO_FLOAT(p_pu->q_value);
    }

    return p_pu->float_value;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化 MT6701：构建 SPI 引脚组与 SPI 实例，并复位角度/转速状态。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效。
 * @retval bsp_encoder_err_state 引脚组构建或 SPI 构建失败。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_init(bsp_encoder_t *p_object)
{
    if (p_object == NULL) {
        return bsp_encoder_err_param;
    }

    /* 板级 SPI 实例（MT6701 编码器专用）——经 HAL SPI 函数表构建 */
    if (hal_spi_ops->pf_build(&hal_spi_encoder) != hal_spi_ok) {
        return bsp_encoder_err_state;
    }

    /* 下发板级配置（角度/状态字段随配置一并清零）并复位测速状态 */
    p_object->config = s_mt6701_config;
    p_object->state = bsp_encoder_state_ready;

    return bsp_encoder_ok;
}

/**
 * @brief 采样一次：先做 CRC-6 校验，通过后更新原始机械角度、Mg 状态位与零点对齐机械角度。
 * @details 24 位帧：bit[23:10] 14 位角度、bit[9:6] Mg[3:0] 状态、bit[5:0] CRC-6；
 *          对 18 位数据域（bit[23:6]）做 CRC-6 校验，失败则直接返回且不更新任何数据；
 *          通过后取角度与状态位，再按 direction 方向归一（方向校正后的角度存入
 *          raw_mechanical_angle）→ 按 zero_offset 零点对齐（以 is_zero_aligned 为
 *          前置条件）→ 按运算方式算出两个机械角度的标幺值 → 记录差分测速基准。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或对象未就绪。
 * @retval bsp_encoder_err_unsupported 配置的传感器/接口类型不受支持。
 * @retval bsp_encoder_err_timeout 传输超时。
 * @retval bsp_encoder_err_crc CRC-6 校验失败（本帧数据被丢弃）。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_get_mechanical_angle(
    bsp_encoder_t *p_object)
{
    uint8_t tx_buffer[BSP_MT6701_TRANSACTION_BYTES] = {0U};
    uint8_t rx_buffer[BSP_MT6701_TRANSACTION_BYTES];
    uint8_t crc_input[BSP_MT6701_CRC_INPUT_BYTES];
    uint32_t frame_value;
    uint32_t data_18bit;
    uint16_t raw_counts;
    uint32_t mechanical_counts;
    uint32_t aligned_counts;
    hal_spi_status_enum spi_status;

    /* 就绪判断与类型判断直接内联在此处，不再单独封装 */
    if ((p_object == NULL) ||
        ((p_object->state != bsp_encoder_state_ready) &&
         (p_object->state != bsp_encoder_state_enabled))) {
        return bsp_encoder_err_param;
    }
    if ((p_object->config.encoder_type != bsp_encoder_type_mt6701) ||
        (p_object->config.interface_type != bsp_encoder_interface_spi)) {
        return bsp_encoder_err_unsupported;
    }

    spi_status = hal_spi_ops->pf_receive(&hal_spi_encoder,
                                        tx_buffer,
                                        rx_buffer,
                                        BSP_MT6701_TRANSACTION_BYTES);
    if (spi_status != hal_spi_ok) {
        return (spi_status == hal_spi_err_timeout) ? bsp_encoder_err_timeout
                                                    : bsp_encoder_err_state;
    }

    /* 24 位帧：bit[23:10] 角度、bit[9:6] Mg[3:0] 状态、bit[5:0] CRC-6 */
    frame_value = ((uint32_t)rx_buffer[0] << 16U) |
                  ((uint32_t)rx_buffer[1] << 8U) |
                  (uint32_t)rx_buffer[2];
    data_18bit = frame_value >> BSP_MT6701_CRC_DATA_SHIFT;

    /* 先做 CRC-6 校验：18 位数据域按 3 组 6 位喂入数据手册示例函数 */
    crc_input[0] = (uint8_t)(data_18bit >> 12U);
    crc_input[1] = (uint8_t)((data_18bit >> 6U) & (uint32_t)BSP_MT6701_CRC_MASK);
    crc_input[2] = (uint8_t)(data_18bit & (uint32_t)BSP_MT6701_CRC_MASK);
    if (bsp_encoder_mt6701_crc6_itu(crc_input, BSP_MT6701_CRC_INPUT_BYTES)
        != (uint8_t)(frame_value & (uint32_t)BSP_MT6701_CRC_MASK)) {
        return bsp_encoder_err_crc;   /* 校验失败：本帧角度与状态位一律不采用 */
    }

    /* 校验通过后才取原始角度与全部 Mg 状态位 */
    raw_counts = data_18bit >> BSP_MT6701_ANGLE_DATA_SHIFT;
    p_object->config.mg_status.value =
        (uint8_t)(data_18bit & (uint32_t)BSP_MT6701_MG_STATUS_MASK);

    /* 采样前记录上一次的机械角度，供 bsp_encoder_get_velocity() 差分测速使用
       （与 raw_mechanical_angle 同域：均为方向校正后的角度，测速方向才与之一致） */
    p_object->config.previous_mechanical_angle =
        p_object->config.raw_mechanical_angle;

    /* 方向归一：反向下取镜像角度；raw_mechanical_angle 存方向校正后的原始角度 */
    mechanical_counts = raw_counts & BSP_MT6701_ANGLE_MASK;
    if (p_object->config.direction == bsp_encoder_direction_reverse) {
        mechanical_counts = (BSP_MT6701_ANGLE_RESOLUTION - mechanical_counts) &
                            BSP_MT6701_ANGLE_MASK;
    }
    p_object->config.raw_mechanical_angle = (uint16_t)mechanical_counts;
    bsp_encoder_counts_to_pu(&p_object->config, mechanical_counts,
                             &p_object->config.raw_mechanical_angle_pu);

    /* 零点对齐：仅在"零点对齐完成"标志置位时计算（前置条件） */
    if (p_object->config.is_zero_aligned) {
        aligned_counts = (mechanical_counts + p_object->config.zero_offset) &
                         BSP_MT6701_ANGLE_MASK;
        p_object->config.zero_aligned_mechanical_angle = (uint16_t)aligned_counts;
        bsp_encoder_counts_to_pu(
            &p_object->config, aligned_counts,
            &p_object->config.zero_aligned_mechanical_angle_pu);
    }

    return bsp_encoder_ok;
}

/**
 * @brief 在机械角度基础上折算原始电角度与零点对齐后的电角度（均为计数）。
 * @details 先调用 bsp_encoder_mt6701_get_mechanical_angle() 刷新机械角度，
 *          再按 电角度 = 机械角度 × 极对数（取单圈）得到电角度计数，
 *          并按运算方式同步算出电角度的标幺值；极对数为 0 时电角度为 0。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或对象未就绪。
 * @retval bsp_encoder_err_unsupported 配置的传感器/接口类型不受支持。
 * @retval bsp_encoder_err_timeout 传输超时。
 * @retval bsp_encoder_err_crc CRC-6 校验失败（本帧数据被丢弃）。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_get_electrical_angle(
    bsp_encoder_t *p_object)
{
    bsp_encoder_status_enum status;
    uint32_t pole_pairs;

    /* 就绪判断直接内联在此处，不再单独封装 */
    if ((p_object == NULL) ||
        ((p_object->state != bsp_encoder_state_ready) &&
         (p_object->state != bsp_encoder_state_enabled))) {
        return bsp_encoder_err_param;
    }

    /* 电角度建立在机械角度之上：先刷新机械角度（含 CRC 校验、方向归一与零点对齐） */
    status = bsp_encoder_mt6701_get_mechanical_angle(p_object);
    if (status != bsp_encoder_ok) {
        return status;
    }

    pole_pairs = (uint32_t)p_object->config.motor_pole_pairs;
    p_object->config.raw_electrical_angle = (uint16_t)(
        ((uint32_t)p_object->config.raw_mechanical_angle * pole_pairs) %
        BSP_MT6701_ANGLE_RESOLUTION);
    bsp_encoder_counts_to_pu(&p_object->config,
                             p_object->config.raw_electrical_angle,
                             &p_object->config.raw_electrical_angle_pu);

    /* 零点对齐电角度同样以"零点对齐完成"为前置条件 */
    if (p_object->config.is_zero_aligned) {
        p_object->config.zero_aligned_electrical_angle = (uint16_t)(
            ((uint32_t)p_object->config.zero_aligned_mechanical_angle * pole_pairs) %
            BSP_MT6701_ANGLE_RESOLUTION);
        bsp_encoder_counts_to_pu(
            &p_object->config,
            p_object->config.zero_aligned_electrical_angle,
            &p_object->config.zero_aligned_electrical_angle_pu);
    }

    return bsp_encoder_ok;
}

/**
 * @brief 按角度种类读取标幺值（0 ~ 1，统一浮点出口）。
 * @details 按 config.arithmetic_type 取对应标幺字段：Q 格式运算时经 Q15 接口
 *          PLAT_IQ_TO_FLOAT() 换算成浮点，浮点运算时直接取用；
 *          零点对齐类角度以 is_zero_aligned 为前置条件，未对齐时直接报状态错误。
 * @param p_object 编码器对象指针。
 * @param angle 角度种类，取 bsp_encoder_angle_x。
 * @param p_value 输出：标幺角度（0.0f ~ 1.0f）。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或角度种类越界。
 * @retval bsp_encoder_err_state 读取零点对齐角度但尚未完成零点对齐。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_get_angle_pu(
    const bsp_encoder_t *p_object,
    bsp_encoder_angle_enum angle,
    float *p_value)
{
    const bsp_encoder_pu_angle_t *p_pu;
    bool need_alignment = false;

    if ((p_object == NULL) || (p_value == NULL) ||
        ((uint32_t)angle >= (uint32_t)bsp_encoder_angle_count)) {
        return bsp_encoder_err_param;
    }

    switch (angle) {
        case bsp_encoder_angle_raw_mechanical:
            p_pu = &p_object->config.raw_mechanical_angle_pu;
            break;
        case bsp_encoder_angle_aligned_mechanical:
            p_pu = &p_object->config.zero_aligned_mechanical_angle_pu;
            need_alignment = true;
            break;
        case bsp_encoder_angle_raw_electrical:
            p_pu = &p_object->config.raw_electrical_angle_pu;
            break;
        case bsp_encoder_angle_aligned_electrical:
            p_pu = &p_object->config.zero_aligned_electrical_angle_pu;
            need_alignment = true;
            break;
        case bsp_encoder_angle_count:
        default:
            return bsp_encoder_err_param;
    }

    /* 零点对齐角度在完成对齐前无意义：直接按状态错误返回 */
    if (need_alignment && (!p_object->config.is_zero_aligned)) {
        return bsp_encoder_err_state;
    }

    *p_value = bsp_encoder_pu_to_float(&p_object->config, p_pu);

    return bsp_encoder_ok;
}

/**
 * @brief 差分计算机械转速（通用接口，不绑定具体传感器）。
 * @details 本函数不采样：用本次与上一次采样（由机械角度获取函数记录）的
 *          原始机械角度计数差表示转速，单位为"计数/采样周期"；
 *          除以 config.sample_period_us 即为计数/秒。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或对象未就绪。
 * @retval bsp_encoder_err_state 未配置采样周期。
 */
bsp_encoder_status_enum bsp_encoder_get_velocity(bsp_encoder_t *p_object)
{
    /* 就绪判断直接内联在此处，不再单独封装 */
    if ((p_object == NULL) ||
        ((p_object->state != bsp_encoder_state_ready) &&
         (p_object->state != bsp_encoder_state_enabled))) {
        return bsp_encoder_err_param;
    }
    if (p_object->config.sample_period_us == 0U) {
        return bsp_encoder_err_state;
    }

    /* 差分测速：本次与上一次采样的原始机械角度计数差（计数/采样周期） */
    p_object->config.mechanical_velocity = bsp_encoder_count_delta(
        p_object->config.raw_mechanical_angle,
        p_object->config.previous_mechanical_angle);

    return bsp_encoder_ok;
}

/**
 * @brief 查询编码器对象当前状态（通用接口，不绑定具体传感器）。
 * @param p_object 编码器对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_encoder_state_error。
 */
bsp_encoder_state_enum bsp_encoder_get_state(const bsp_encoder_t *p_object)
{
    if (p_object == NULL) {
        return bsp_encoder_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * BSP 编码器函数表
 * ========================================================================== */

/** @brief BSP MT6701 编码器函数表（本文件私有，绑定板级接口）。 */
static const bsp_encoder_mt6701_ops_t s_bsp_encoder_mt6701_ops = {
    .pf_init = bsp_encoder_mt6701_init,
    .pf_get_mechanical_angle = bsp_encoder_mt6701_get_mechanical_angle,
    .pf_get_electrical_angle = bsp_encoder_mt6701_get_electrical_angle,
    .pf_get_angle_pu = bsp_encoder_mt6701_get_angle_pu,
    .pf_get_velocity = bsp_encoder_get_velocity
};

/** @brief BSP MT6701 编码器函数表指针（对外外露，声明于 bsp_encoder.h）。 */
const bsp_encoder_mt6701_ops_t *const bsp_encoder_mt6701_ops =
    &s_bsp_encoder_mt6701_ops;
