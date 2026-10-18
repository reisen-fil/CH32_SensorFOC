/**
 * @file bsp_encoder.h
 * @brief MT6701 磁编码器 BSP 接口。
 * @details 结构与本层 bsp_led 同构：对象只保存状态，接口 + 一张 BSP 函数表：
 *          - bsp_encoder_mt6701_init()：构建 SPI 引脚组与 SPI 实例，复位角度与转速状态；
 *          - bsp_encoder_mt6701_get_mechanical_angle()：一次 SPI 采样，先做 CRC-6 校验，
 *            校验通过后更新原始机械角度（已做方向校正）、Mg 状态位与零点对齐后的机械
 *            角度（均为原始计数），并按配置的运算方式同步算出两者的标幺值（0 ~ 1）；
 *          - bsp_encoder_mt6701_get_electrical_angle()：在调用机械角度获取函数的基础上，
 *            按极对数折算原始电角度与零点对齐后的电角度（原始计数）及其标幺值；
 *          - bsp_encoder_mt6701_get_angle_pu()：按角度种类读取标幺值（统一浮点出口）；
 *          - bsp_encoder_get_velocity()：差分计算机械转速（本次与上一次采样的计数差）；
 *          - bsp_encoder_get_state()：查询对象状态。
 *          角度信息分两套并存：**原始计数域**（0 … 单圈计数 - 1，可直接作正弦表索引）
 *          与**标幺值域**（0 ~ 1，Q15 或浮点，由 config.arithmetic_type 选择）；
 *          本层只做"计数 ↔ 标幺"换算，不做弧度换算。
 *          接口内部一律经 HAL GPIO / HAL SPI 函数表指针（hal_gpio_ops、hal_spi_ops）
 *          访问硬件；对象不携带操作表、不保存下层对象指针（板级单例
 *          hal_gpio_spi_hardware / hal_spi_encoder 由本层直接引用）。
 *          四个接口经 bsp_encoder_mt6701_ops 函数表对外外露。
 */
#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 编码器对象的生命周期状态。 */
typedef enum {
    bsp_encoder_state_reset = 0,
    bsp_encoder_state_ready,
    bsp_encoder_state_enabled,
    bsp_encoder_state_disabled,
    bsp_encoder_state_error
} bsp_encoder_state_enum;

/** @brief 编码器接口调用返回的状态码。 */
typedef enum {
    bsp_encoder_ok = 0,
    bsp_encoder_err_param = -1,
    bsp_encoder_err_state = -2,
    bsp_encoder_err_unsupported = -3,
    bsp_encoder_err_timeout = -4,
    bsp_encoder_err_crc = -5            /**< CRC-6 校验失败，本帧数据不可用 */
} bsp_encoder_status_enum;

/** @brief 编码器传感器类型。 */
typedef enum {
    bsp_encoder_type_mt6701 = 0,
    bsp_encoder_type_as5600,
    bsp_encoder_type_hall
} bsp_encoder_type_enum;

/** @brief 编码器物理接口类型。 */
typedef enum {
    bsp_encoder_interface_abz = 0,
    bsp_encoder_interface_spi,
    bsp_encoder_interface_i2c,
    bsp_encoder_interface_analog
} bsp_encoder_interface_enum;

/** @brief 编码器角度正方向。 */
typedef enum {
    bsp_encoder_direction_forward = 0,
    bsp_encoder_direction_reverse
} bsp_encoder_direction_enum;

/**
 * @brief 标幺角度的运算方式。
 * @details 决定各标幺角度字段按哪种方式计算与解释：
 *          - q：Q 格式运算，字段为 Q15 定点值（0 ~ 32768 对应标幺 0.0 ~ 1.0），
 *            换算一律经 platform_ch32v203.h 暴露的 PLAT_IQ_* 接口完成；
 *          - float：浮点运算，字段为浮点值（0.0f ~ 1.0f）。
 */
typedef enum {
    bsp_encoder_arithmetic_q = 0,      /**< Q 格式运算（Q15 定点） */
    bsp_encoder_arithmetic_float       /**< 浮点运算 */
} bsp_encoder_arithmetic_enum;

/**
 * @brief 标幺角度值（0 ~ 1）：同一存储按 config.arithmetic_type 解释为 Q 格式或浮点。
 * @note 有效成员由 config.arithmetic_type 决定；取值请优先用
 *       bsp_encoder_mt6701_get_angle_pu()（统一浮点出口），
 *       定点消费者可按运算方式直接读取对应成员。
 */
typedef union {
    PLAT_IQ_TYPE q_value;    /**< Q 格式标幺值（Q15：0 ~ 32768 = 0.0 ~ 1.0） */
    float float_value;       /**< 浮点标幺值（0.0f ~ 1.0f） */
} bsp_encoder_pu_angle_t;

/** @brief 标幺角度种类（bsp_encoder_mt6701_get_angle_pu 的选择子）。 */
typedef enum {
    bsp_encoder_angle_raw_mechanical = 0,  /**< 原始机械角度（方向校正后）标幺值 */
    bsp_encoder_angle_aligned_mechanical,  /**< 零点对齐后的机械角度标幺值 */
    bsp_encoder_angle_raw_electrical,      /**< 原始电角度标幺值 */
    bsp_encoder_angle_aligned_electrical,  /**< 零点对齐后的电角度标幺值 */
    bsp_encoder_angle_count                /**< 角度种类数量（哨兵） */
} bsp_encoder_angle_enum;

/**
 * @brief MT6701 数据手册 Mg 状态位（24 位帧的 bit[9:6]）联合体定义。
 * @details 按数据手册的位定义组织：既可整体取原始 4 位值，也可按位访问——
 *          - bits.mg_field：Mg[1:0]（帧 bit[7:6]）；
 *          - bits.mg_bit2 ：Mg[2]  （帧 bit[8]）；
 *          - bits.mg_bit3 ：Mg[3]  （帧 bit[9]）。
 *          各状态位的具体含义与有效电平以数据手册为准。
 * @note 位域在原厂工具链（GCC/RISC-V）上自低位起分配，故 bits 与 value 的低 4 位一一对应。
 */
typedef union {
    uint8_t value;              /**< 原始 4 位状态值（= 帧 bit[9:6]） */
    struct {
        uint8_t mg_field : 2;   /**< Mg[1:0]：磁场强度状态位（帧 bit[7:6]） */
        uint8_t mg_bit2 : 1;    /**< Mg[2]：状态位（帧 bit[8]，含义见数据手册） */
        uint8_t mg_bit3 : 1;    /**< Mg[3]：状态位（帧 bit[9]，含义见数据手册） */
        uint8_t reserved : 4;   /**< 保留（恒为 0） */
    } bits;
} bsp_encoder_mt6701_mg_status_t;

/**
 * @brief 编码器配置与角度信息。
 * @details 前半部分为"怎么用这个编码器"的配置量；后半部分为最近一次**通过 CRC 校验**的
 *          采样信息，同一角度给出两套表示——**原始计数**（0 … 单圈计数 - 1）与
 *          **标幺值**（0 ~ 1，运算方式见 arithmetic_type），均不做弧度换算：
 *          - raw_mechanical_angle：已按 direction 做方向校正的原始机械角度（计数）；
 *          - raw_electrical_angle：由 raw_mechanical_angle 按极对数折算的原始电角度；
 *          - zero_aligned_*：在方向校正基础上再叠加 zero_offset 取单圈后的值，
 *            以 is_zero_aligned（零点对齐完成标志）为前置条件，标志为 false 时不计算；
 *          - *_pu：上述四个角度对应的标幺值（0 ~ 1），随计数域同步更新；
 *          - mg_status：数据手册 Mg[1:0]/Mg[2]/Mg[3] 状态位；
 *          - previous_mechanical_angle / mechanical_velocity：差分测速的基准与结果。
 *          is_zero_aligned 由应用层在完成零点标定（确定 zero_offset）后置 true。
 */
typedef struct {
    /* ------------------------------ 配置 ------------------------------ */
    bsp_encoder_type_enum encoder_type;        /**< 传感器类型 */
    bsp_encoder_interface_enum interface_type; /**< 物理接口类型 */
    uint8_t motor_pole_pairs;                  /**< 电机极对数（电角度 = 机械角度 × 极对数） */
    bsp_encoder_direction_enum direction;      /**< 角度正方向 */
    uint16_t resolution_bits;                  /**< 单圈分辨率位数（MT6701 为 14） */
    bsp_encoder_arithmetic_enum arithmetic_type; /**< 标幺角度运算方式（Q 格式/浮点） */
    uint32_t zero_offset;                      /**< 零点偏置（计数），用于零点对齐 */
    uint32_t sample_period_us;                 /**< 采样周期（µs），差分测速用；0 表示未配置 */
    uint32_t timeout_count;                    /**< 传输等待超时计数 */
    bool is_absolute;                          /**< true 单圈绝对式，false 增量式 */

    /* ---------------------------- 角度信息 ---------------------------- */
    uint16_t raw_mechanical_angle;             /**< 原始机械角度（计数，已做方向校正） */
    uint16_t raw_electrical_angle;             /**< 原始电角度（计数，单圈折算） */
    uint16_t zero_aligned_mechanical_angle;    /**< 零点对齐后的机械角度（计数） */
    uint16_t zero_aligned_electrical_angle;    /**< 零点对齐后的电角度（计数） */
    bsp_encoder_pu_angle_t raw_mechanical_angle_pu;          /**< 原始机械角度标幺值（0~1） */
    bsp_encoder_pu_angle_t zero_aligned_mechanical_angle_pu; /**< 零点对齐机械角度标幺值 */
    bsp_encoder_pu_angle_t raw_electrical_angle_pu;          /**< 原始电角度标幺值（0~1） */
    bsp_encoder_pu_angle_t zero_aligned_electrical_angle_pu; /**< 零点对齐电角度标幺值 */
    bsp_encoder_mt6701_mg_status_t mg_status;  /**< 最近一次通过 CRC 校验的 Mg 状态位 */
    bool is_zero_aligned;                      /**< 零点对齐完成标志：false 时不计算 zero_aligned_* */

    /* ---------------------------- 测速状态 ---------------------------- */
    uint16_t previous_mechanical_angle;        /**< 上一次采样的机械角度（计数，方向归一前） */
    int32_t mechanical_velocity;               /**< 机械转速：相邻两次采样的计数差（计数/采样周期） */
} bsp_encoder_config_t;

/**
 * @brief 编码器状态对象：生命周期状态 + 配置与角度信息。
 * @details 不携带操作表，也不保存下层 HAL/平台对象指针——硬件实例由 bsp_encoder.c
 *          直接引用板级单例；角度与测速信息集中保存在 config 中（见上）。
 */
typedef struct {
    bsp_encoder_state_enum state;             /**< 对象生命周期状态 */
    bsp_encoder_config_t config;              /**< 配置 + 最近一次采样的角度/测速信息 */
} bsp_encoder_t;

/** @brief 板级 MT6701 SPI 编码器对象（单实例）。 */
extern bsp_encoder_t bsp_encoder_mt6701;

/**
 * @brief 初始化编码器：构建 SPI 引脚组与 SPI 实例，并复位角度/转速状态。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效。
 * @retval bsp_encoder_err_state 引脚组构建或 SPI 构建失败。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_init(bsp_encoder_t *p_object);

/**
 * @brief 采样一次：先做 CRC-6 校验，通过后更新原始机械角度、Mg 状态位与零点对齐机械角度。
 * @details 读取 24 位帧——bit[23:10] 为 14 位角度、bit[9:6] 为 Mg[3:0] 状态、bit[5:0] 为 CRC-6；
 *          先对 18 位数据域（bit[23:6]）做 CRC-6 校验（bsp_encoder_mt6701_crc6_itu），
 *          **校验不通过时直接返回错误，角度与状态位一律不更新**；
 *          校验通过后才取原始机械角度与全部 Mg 状态位，再做方向归一与零点对齐
 *          （零点对齐以 is_zero_aligned 为前置条件），并记录本次机械角度作为差分测速基准。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或对象未就绪。
 * @retval bsp_encoder_err_unsupported 配置的传感器/接口类型不受支持。
 * @retval bsp_encoder_err_timeout 传输超时。
 * @retval bsp_encoder_err_crc CRC-6 校验失败（本帧数据被丢弃）。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_get_mechanical_angle(bsp_encoder_t *p_object);

/**
 * @brief 在机械角度基础上折算原始电角度与零点对齐后的电角度（均为计数）。
 * @details 内部先调用 bsp_encoder_mt6701_get_mechanical_angle() 刷新机械角度，
 *          再按 电角度 = 机械角度 × 极对数（取单圈）得到电角度计数；
 *          零点对齐电角度同样以 is_zero_aligned 为前置条件。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或对象未就绪。
 * @retval bsp_encoder_err_unsupported 配置的传感器/接口类型不受支持。
 * @retval bsp_encoder_err_timeout 传输超时。
 * @retval bsp_encoder_err_crc CRC-6 校验失败（本帧数据被丢弃）。
 */
bsp_encoder_status_enum bsp_encoder_mt6701_get_electrical_angle(bsp_encoder_t *p_object);

/**
 * @brief 按角度种类读取标幺值（0 ~ 1，统一浮点出口）。
 * @details 按 config.arithmetic_type 从对应标幺字段取值：Q 格式运算时经 Q15 接口
 *          PLAT_IQ_TO_FLOAT() 换算成浮点，浮点运算时直接取用；
 *          零点对齐类角度以 is_zero_aligned 为前置条件，未完成对齐时返回错误。
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
    float *p_value);

/**
 * @brief 差分计算机械转速（本函数不采样，用本次与上一次采样的机械角度计数差表示）。
 * @param p_object 编码器对象指针。
 * @retval bsp_encoder_ok 操作成功。
 * @retval bsp_encoder_err_param 参数无效或对象未就绪。
 * @retval bsp_encoder_err_state 未配置采样周期（config.sample_period_us == 0）。
 */
bsp_encoder_status_enum bsp_encoder_get_velocity(bsp_encoder_t *p_object);

/**
 * @brief 查询编码器对象当前状态。
 * @param p_object 编码器对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_encoder_state_error。
 */
bsp_encoder_state_enum bsp_encoder_get_state(const bsp_encoder_t *p_object);

/**
 * @brief BSP 编码器函数表：初始化 / 机械角度获取 / 电角度获取 / 标幺角度读取 / 机械转速获取。
 * @details 表中的函数指针在 bsp_encoder.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 bsp_encoder_mt6701_ops 指针统一调用。
 */
typedef struct {
    /** @brief 初始化编码器（见 bsp_encoder_mt6701_init）。 */
    bsp_encoder_status_enum (*pf_init)(bsp_encoder_t *p_object);
    /** @brief 采样并更新机械角度（见 bsp_encoder_mt6701_get_mechanical_angle）。 */
    bsp_encoder_status_enum (*pf_get_mechanical_angle)(bsp_encoder_t *p_object);
    /** @brief 在机械角度基础上折算电角度（见 bsp_encoder_mt6701_get_electrical_angle）。 */
    bsp_encoder_status_enum (*pf_get_electrical_angle)(bsp_encoder_t *p_object);
    /** @brief 读取标幺角度（见 bsp_encoder_mt6701_get_angle_pu）。 */
    bsp_encoder_status_enum (*pf_get_angle_pu)(const bsp_encoder_t *p_object,
                                               bsp_encoder_angle_enum angle,
                                               float *p_value);
    /** @brief 差分计算机械转速（见 bsp_encoder_get_velocity）。 */
    bsp_encoder_status_enum (*pf_get_velocity)(bsp_encoder_t *p_object);
} bsp_encoder_mt6701_ops_t;

/** @brief BSP 编码器函数表指针（在 bsp_encoder.c 中定义并绑定）。 */
extern const bsp_encoder_mt6701_ops_t *const bsp_encoder_mt6701_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ENCODER_H__ */
