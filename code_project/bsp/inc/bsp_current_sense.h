/**
 * @file bsp_current_sense.h
 * @brief 三相电流采样 BSP 接口（双电阻重构：ADC2 注入组 + OPA1/OPA2 内部运放）。
 * @details 对象中的相量统一按 bsp_current_sense_phase_enum 数组保存：
 *          raw / offset / Q 标幺值 / 浮点标幺值 / 注入序号均使用相序索引，避免 U/V/W
 *          三套同类单变量定义。bsp_current_sense_calc() 只完成 ADC 去偏置、标幺化与
 *          双电阻第三相重构，不在采样环节计算实际安培值；bsp_current_sense_get_current()
 *          再根据满量程电流同时输出标幺值与取整后的 mA 电流。
 *
 *          标幺值约定：ADC 去偏置后的计数差除以 ADC 满量程计数，得到 -1.0 ~ 1.0。
 *          Q 路径保存为 Q1.15（1 位符号位 + 15 位小数位），范围 -32768 ~ 32767；
 *          浮点路径直接保存标幺小数值。满量程电流使用 mA 单位。
 *          本层只做采样值 → 标幺相电流的换算与重构，不含 Clarke/Park/PI 等控制算法。
 */
#ifndef __BSP_CURRENT_SENSE_H__
#define __BSP_CURRENT_SENSE_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 电流采样对象的生命周期状态。 */
typedef enum {
    bsp_current_sense_state_reset = 0,
    bsp_current_sense_state_ready,
    bsp_current_sense_state_running,
    bsp_current_sense_state_error
} bsp_current_sense_state_enum;

/** @brief 电流采样接口调用返回的状态码。 */
typedef enum {
    bsp_current_sense_ok = 0,
    bsp_current_sense_err_param = -1,
    bsp_current_sense_err_state = -2,
    bsp_current_sense_err_unsupported = -3,
    bsp_current_sense_err_timeout = -4
} bsp_current_sense_status_enum;

/** @brief 三相序号（0 起始）。 */
typedef enum {
    bsp_current_sense_phase_u = 0,
    bsp_current_sense_phase_v,
    bsp_current_sense_phase_w,
    bsp_current_sense_phase_count
} bsp_current_sense_phase_enum;

/**
 * @brief 电流计算数据类型。
 * @details calc() 输出的是相电流标幺值：Q 路径保存 Q1.15，浮点路径保存 -1.0 ~ 1.0 的
 *          浮点值。两条路径的物理量都是“相对 ADC 满量程”的无量纲值，不是安培值。
 */
typedef enum {
    bsp_current_sense_data_type_q = 0, /**< Q1.15 标幺值，范围 -32768 ~ 32767 */
    bsp_current_sense_data_type_float  /**< 浮点标幺值，范围约 -1.0 ~ 1.0 */
} bsp_current_sense_data_type_enum;

/**
 * @brief 电流重构方式（采样电阻拓扑）。
 * @note 本板为双电阻：U/V 两相直接采样，W 相由 iu + iv + iw = 0 重构。
 */
typedef enum {
    bsp_current_sense_reconstruct_single_resistor = 0,
    bsp_current_sense_reconstruct_dual_resistor,
    bsp_current_sense_reconstruct_triple_resistor
} bsp_current_sense_reconstruct_enum;

/**
 * @brief 同时兼容 Q1.15 与浮点标幺值的输出联合体。
 * @details 调用方按 config.data_type 读取对应成员：Q 路径读取 q_value，浮点路径读取
 *          float_value；另一个成员没有定义意义。
 */
typedef union {
    PLAT_IQ_TYPE q_value; /**< Q1.15 标幺值 */
    float float_value;    /**< 浮点标幺值 */
} bsp_current_sense_value_t;

/**
 * @brief 三相电流采样板级配置。
 * @details 相序相关字段统一使用 phase_count 长度数组；W 相在双电阻配置下不使用 ADC
 *          注入序号，置 0 即可。
 */
typedef struct {
    bsp_current_sense_data_type_enum data_type;
    bsp_current_sense_reconstruct_enum reconstruct;

    uint32_t shunt_resistance_micro_ohm;
    uint32_t amplifier_gain_milli;
    uint16_t adc_reference_mv;
    uint16_t adc_full_scale;

    int32_t offset_raw[bsp_current_sense_phase_count];
    uint8_t injected_rank[bsp_current_sense_phase_count];

    uint32_t timeout_count;
    void *p_user_data;
} bsp_current_sense_config_t;

/**
 * @brief 三相电流采样状态对象。
 * @details raw、current_q、current_a 均按 bsp_current_sense_phase_enum 数组保存。
 *          current_q/current_a 保存标幺值；实际电流由 get_current() 按满量程 mA 换算。
 */
typedef struct {
    bsp_current_sense_state_enum state;
    bsp_current_sense_config_t config;

    uint16_t raw[bsp_current_sense_phase_count];
    PLAT_IQ_TYPE current_q[bsp_current_sense_phase_count];
    float current_a[bsp_current_sense_phase_count];

    uint8_t valid_phase_mask; /**< 当前采样窗口有效相位位图：U=0x01、V=0x02、W=0x04 */
    uint32_t full_scale_current_ma; /**< 满量程电流（mA） */
} bsp_current_sense_t;

/** @brief 板级三相电流采样对象（单实例）。 */
extern bsp_current_sense_t bsp_current_sense_three_phase;

/** @brief 初始化三相电流采样，构建内部运放与 ADC2 注入组。 */
bsp_current_sense_status_enum bsp_current_sense_init(bsp_current_sense_t *p_object);

/**
 * @brief 读取 ADC、去偏置、计算标幺值并完成双电阻第三相重构。
 * @details 不在此函数内计算实际安培值；Q 路径把标幺值限幅并转换为 Q1.15，浮点路径
 *          直接保存浮点标幺值。
 */
bsp_current_sense_status_enum bsp_current_sense_calc(bsp_current_sense_t *p_object);

/** @brief 零电流偏置标定：把当前两路注入读数记为新的零电流偏置。 */
bsp_current_sense_status_enum bsp_current_sense_calibrate(
    bsp_current_sense_t *p_object);

/**
 * @brief 获取指定相的标幺电流与实际电流大小。
 * @param p_object 电流采样对象指针。
 * @param phase 相序号，取 bsp_current_sense_phase_x。
 * @param p_pu 输出：标幺电流（按配置取 q_value 或 float_value）。
 * @param p_current_ma 输出：实际电流（mA，带符号，取整）。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或相序号越界。
 * @retval bsp_current_sense_err_state 运算类型不受支持。
 */
bsp_current_sense_status_enum bsp_current_sense_get_current(
    const bsp_current_sense_t *p_object,
    uint8_t phase,
    bsp_current_sense_value_t *p_pu,
    int32_t *p_current_ma);

/** @brief 查询电流采样对象当前状态。 */
bsp_current_sense_state_enum bsp_current_sense_get_state(
    const bsp_current_sense_t *p_object);

/** @brief BSP 三相电流采样函数表。 */
typedef struct {
    bsp_current_sense_status_enum (*pf_init)(bsp_current_sense_t *p_object);
    bsp_current_sense_status_enum (*pf_calc)(bsp_current_sense_t *p_object);
    bsp_current_sense_status_enum (*pf_calibrate)(bsp_current_sense_t *p_object);
    bsp_current_sense_status_enum (*pf_get_current)(
        const bsp_current_sense_t *p_object,
        uint8_t phase,
        bsp_current_sense_value_t *p_pu,
        int32_t *p_current_ma);
    bsp_current_sense_state_enum (*pf_get_state)(const bsp_current_sense_t *p_object);
} bsp_current_sense_ops_t;

/** @brief BSP 三相电流采样函数表指针。 */
extern const bsp_current_sense_ops_t *const bsp_current_sense_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CURRENT_SENSE_H__ */
