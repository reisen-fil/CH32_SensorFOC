/**
 * @file bsp_current_sense.h
 * @brief 三相电流采样 BSP 接口（双电阻重构：ADC2 注入组 + OPA1/OPA2 内部运放）。
 * @details 结构与本层 bsp_motor_ctrl / bsp_analog_sense 同构：对象只保存状态与最近一次
 *          采样结果，接口 + 一张 BSP 函数表：
 *          - bsp_current_sense_init()：先构建两只内部运放（hal_opamp_opa1/opa2，运放引脚组
 *            在其中一并配置），再构建 ADC2 注入组（hal_adc_injected，TIM1_TRGO 触发），
 *            并按板级配置表注入各外设配置；
 *          - bsp_current_sense_calc()：取注入组两路转换值 → 去零电流偏置 → 按配置的
 *            计算数据类型换算成相电流（定点侧经 platform_ch32v203.h 的 PLAT_IQ_* 宏
 *            直接算出**定点化安培值**，1.0 = 1 A），并按 iu + iv + iw = 0 重构第三相；
 *          - bsp_current_sense_calibrate()：无电流时把两路读数记为新的零电流偏置；
 *          - bsp_current_sense_get_current()：取某相电流的浮点值——Q 格式运算时先读
 *            Q 格式结果再经 PLAT_IQ_TO_FLOAT() 按 2^15 移位换算，浮点运算时直接取
 *            浮点结果，两条路径都输出浮点安培值。
 *          本层只做"采样值 → 相电流"的换算与重构，不含任何控制算法（Clarke/Park/PI 等
 *          属 service 层）；硬件访问一律经 HAL ADC / HAL 运放函数表指针
 *          hal_adc_ops / hal_opamp_ops。
 *          板级注入序号事实（由 hal_adc.c 的注入通道表决定）：
 *            序号 1 = PB1 = OPA1 输出 → U 相；序号 2 = PA4 = OPA2 输出 → V 相。
 *          本模块独占 hal_adc_injected 与 hal_opamp_opa1/opa2 三个板级实例，
 *          其它模块不得重复构建。
 *          接口经 bsp_current_sense_ops 函数表对外外露（状态查询为普通访问函数）。
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
    bsp_current_sense_state_ready,   /**< 运放与注入组已构建，尚未换算过电流 */
    bsp_current_sense_state_running, /**< 已在正常换算电流 */
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

/**
 * @brief 三相序号（0 起始）。
 * @note 与 bsp_motor_ctrl_phase_x 同序，便于 service 层在 PWM 与控制算法间直接对应。
 */
typedef enum {
    bsp_current_sense_phase_u = 0,
    bsp_current_sense_phase_v,
    bsp_current_sense_phase_w,
    bsp_current_sense_phase_count
} bsp_current_sense_phase_enum;

/**
 * @brief 电流计算数据类型。
 * @details 决定 bsp_current_sense_calc() 走哪条换算路径：
 *          - q：全程定点域，运算一律经 platform_ch32v203.h 暴露的 Q15 接口宏
 *            （PLAT_IQ / PLAT_IQ_MPY_I32 / PLAT_IQ_TO_FLOAT），结果为**定点化安培值**
 *            （Q15，1.0 = 1 A；32 位可表示 ±65535 A），无浮点运算；
 *          - float：浮点域，结果为**浮点安培值**。
 *          两条路径结果的**量纲都是安培**，不存在标幺（1.0 = 满量程电流）形式；
 *          另一侧的结果字段不更新，取用时由 bsp_current_sense_get_current() 换算成
 *          浮点值输出。
 */
typedef enum {
    bsp_current_sense_data_type_q = 0, /**< 32 位定点安培值（PLAT_IQ_TYPE / PLAT_IQ_*，1.0 = 1 A） */
    bsp_current_sense_data_type_float  /**< 单精度浮点（A） */
} bsp_current_sense_data_type_enum;

/**
 * @brief 电流重构方式（采样电阻拓扑）。
 * @note 本板为**双电阻**：U/V 两相经内部运放接 ADC2 注入组序号 1/2，W 相由
 *       iu + iv + iw = 0 重构。单电阻（需按扇区选择采样时刻）与三电阻（需第三路
 *       注入通道）暂未实现，init/calc 会返回 bsp_current_sense_err_unsupported。
 */
typedef enum {
    bsp_current_sense_reconstruct_single_resistor = 0, /**< 单电阻：单分流 + 扇区重构 */
    bsp_current_sense_reconstruct_dual_resistor,       /**< 双电阻：两相直接采样 + 第三相重构 */
    bsp_current_sense_reconstruct_triple_resistor      /**< 三电阻：三相直接采样 */
} bsp_current_sense_reconstruct_enum;

/**
 * @brief 三相电流采样板级配置。
 * @details 只放"板级电气事实"与采样映射，不含控制策略：
 *          - data_type：换算结果的数据类型（32 位 Q 格式 / float）；
 *          - reconstruct：电流重构方式（本板取双电阻）；
 *          - shunt_resistance_micro_ohm / amplifier_gain_milli：分流电阻（µΩ）与运放增益
 *            （×1000），二者与参考电压共同决定满量程电流；
 *          - adc_reference_mv / adc_full_scale：ADC 参考电压（mV）与满量程计数（12 位 4095）；
 *          - offset_raw_u / offset_raw_v：两相零电流偏置（ADC 计数），可由 calibrate 现场标定；
 *          - injected_rank_u / injected_rank_v：两相在 ADC 注入序列中的序号（HAL 侧 1 起始）；
 *          - timeout_count / p_user_data：预留（等待超时与上层私有数据）。
 */
typedef struct {
    bsp_current_sense_data_type_enum data_type;      /**< 计算数据类型：Q 格式 / float */
    bsp_current_sense_reconstruct_enum reconstruct;  /**< 电流重构方式：单/双/三电阻 */

    uint32_t shunt_resistance_micro_ohm;  /**< 相分流电阻（µΩ），如 1000 = 1 mΩ */
    uint32_t amplifier_gain_milli;        /**< 内部运放增益（×1000），如 10000 = 10 倍 */
    uint16_t adc_reference_mv;            /**< ADC 参考电压（mV），如 3300 */
    uint16_t adc_full_scale;              /**< ADC 满量程计数，如 4095 */

    int32_t offset_raw_u;                 /**< U 相零电流偏置（ADC 计数） */
    int32_t offset_raw_v;                 /**< V 相零电流偏置（ADC 计数） */

    uint8_t injected_rank_u;              /**< U 相注入序号（HAL 侧 1 起始） */
    uint8_t injected_rank_v;              /**< V 相注入序号（HAL 侧 1 起始） */

    uint32_t timeout_count;               /**< 预留：等待转换完成的超时计数 */
    void *p_user_data;                    /**< 预留：上层私有数据 */
} bsp_current_sense_config_t;

/**
 * @brief 三相电流采样状态对象：生命周期状态 + 配置 + 最近一次采样与换算结果。
 * @details 不携带操作表，也不保存下层 HAL 对象指针——ADC2 注入组与两只运放实例由
 *          bsp_current_sense.c 直接引用板级单例 hal_adc_injected / hal_opamp_opa1 / hal_opamp_opa2。
 *          电流字段只有配置数据类型那一侧在 calc 中被写；另一侧由取用接口实时换算。
 */
typedef struct {
    bsp_current_sense_state_enum state;  /**< 对象生命周期状态 */
    bsp_current_sense_config_t config;   /**< 板级配置（偏置可被 calibrate 更新） */

    uint16_t raw_u;                      /**< U 相注入通道最近一次原始计数 */
    uint16_t raw_v;                      /**< V 相注入通道最近一次原始计数 */

    PLAT_IQ_TYPE current_q_u;            /**< U 相电流：定点化安培值（Q15，1.0 = 1 A） */
    PLAT_IQ_TYPE current_q_v;            /**< V 相电流：定点化安培值（Q15） */
    PLAT_IQ_TYPE current_q_w;            /**< W 相电流：定点化安培值（Q15，由 U/V 重构） */

    float current_a_u;                   /**< U 相电流（A） */
    float current_a_v;                   /**< V 相电流（A） */
    float current_a_w;                   /**< W 相电流（A，由 U/V 重构） */

    float full_scale_current_a;          /**< 满量程电流（A）= V_ref / 运放增益 / 分流电阻 */
    PLAT_IQ_TYPE current_per_count_q;    /**< 每 ADC 计数对应的电流（Q15 定点，A/计数） */
} bsp_current_sense_t;

/** @brief 板级三相电流采样对象（单实例）。 */
extern bsp_current_sense_t bsp_current_sense_three_phase;

/**
 * @brief 初始化三相电流采样：构建内部运放与 ADC2 注入组，并注入板级配置。
 * @details 顺序不可颠倒：运放引脚组（PA5/6/7、PB0 浮空，PA3/4、PB1 模拟）由
 *          hal_opamp_build() 配置，而注入通道 PB1/PA4 复用同一引脚组，故必须先建运放。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或板级配置非法。
 * @retval bsp_current_sense_err_unsupported 重构方式不是双电阻。
 * @retval bsp_current_sense_err_state 运放构建或 ADC 注入组构建失败。
 */
bsp_current_sense_status_enum bsp_current_sense_init(bsp_current_sense_t *p_object);

/**
 * @brief 换算一次三相电流（读取注入组结果 → 去偏置 → 换算 → 双电阻重构）。
 * @details 读取的是注入组最近一次转换结果，不在本函数内触发转换（触发源为 TIM1_TRGO）；
 *          Q 路径全程整数、float 路径为软浮点，均可在控制周期内调用。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或数据类型非法。
 * @retval bsp_current_sense_err_state 对象未初始化或 HAL 读取注入值失败。
 * @retval bsp_current_sense_err_unsupported 重构方式不是双电阻。
 */
bsp_current_sense_status_enum bsp_current_sense_calc(bsp_current_sense_t *p_object);

/**
 * @brief 零电流偏置标定：把当前两路注入读数记为新的零电流偏置。
 * @note 必须在电机无电流（主输出关闭、相电流为零）时调用，否则偏置会被污染。
 * @param p_object 电流采样对象指针。
 * @retval bsp_current_sense_ok 操作成功。
 * @retval bsp_current_sense_err_param 参数无效或对象未初始化。
 * @retval bsp_current_sense_err_state HAL 读取注入值失败。
 */
bsp_current_sense_status_enum bsp_current_sense_calibrate(
    bsp_current_sense_t *p_object);

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
    float *p_value);

/**
 * @brief 查询电流采样对象当前状态。
 * @param p_object 电流采样对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_current_sense_state_error。
 */
bsp_current_sense_state_enum bsp_current_sense_get_state(
    const bsp_current_sense_t *p_object);

/**
 * @brief BSP 三相电流采样函数表：初始化 / 换算 / 标定 / 取值。
 * @details 表中的函数指针在 bsp_current_sense.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 bsp_current_sense_ops 指针统一调用。
 */
typedef struct {
    /** @brief 初始化（见 bsp_current_sense_init）。 */
    bsp_current_sense_status_enum (*pf_init)(bsp_current_sense_t *p_object);
    /** @brief 换算一次三相电流（见 bsp_current_sense_calc）。 */
    bsp_current_sense_status_enum (*pf_calc)(bsp_current_sense_t *p_object);
    /** @brief 零电流偏置标定（见 bsp_current_sense_calibrate）。 */
    bsp_current_sense_status_enum (*pf_calibrate)(bsp_current_sense_t *p_object);
    /** @brief 取相电流浮点值（见 bsp_current_sense_get_current）。 */
    bsp_current_sense_status_enum (*pf_get_current)(
        const bsp_current_sense_t *p_object,
        uint8_t phase,
        float *p_value);
    bsp_current_sense_state_enum (*pf_get_state)(const bsp_current_sense_t *p_object);
} bsp_current_sense_ops_t;

/** @brief BSP 三相电流采样函数表指针（在 bsp_current_sense.c 中定义并绑定）。 */
extern const bsp_current_sense_ops_t *const bsp_current_sense_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CURRENT_SENSE_H__ */
