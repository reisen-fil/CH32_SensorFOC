/**
 * @file bsp_motor_ctrl.h
 * @brief 三相电机 PWM 控制 BSP 接口。
 * @details 结构与本层 bsp_encoder / bsp_led 同构：对象只保存状态，接口 + 一张 BSP 函数表：
 *          - bsp_motor_ctrl_init()：构建三相 PWM（TIM1 互补输出 + 死区 + 主输出）并把
 *            三相占空比置于"无输出"的安全状态；
 *          - bsp_motor_ctrl_set_duty()：下发三相导通占空比（计数域，0 … pwm_period）；
 *          - bsp_motor_ctrl_stop()：三相回到"无输出"状态并置对象为 disabled；
 *          - bsp_motor_ctrl_get_state()：查询对象状态。
 *          本层只做"把占空比变成硬件动作"的执行，不含任何控制算法（Clarke/Park/调制等
 *          属 service 层）；硬件访问一律经 HAL 定时器函数表指针 hal_tim_ops。
 *          对象不携带操作表、不保存下层对象指针（板级单例 hal_tim_motor 由本层直接引用）。
 *          四个接口经 bsp_motor_ctrl_ops 函数表对外外露（状态查询为普通访问函数）。
 */
#ifndef __BSP_MOTOR_CTRL_H__
#define __BSP_MOTOR_CTRL_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 电机控制对象的生命周期状态。 */
typedef enum {
    bsp_motor_ctrl_state_reset = 0,
    bsp_motor_ctrl_state_ready,     /**< 已初始化且处于"无输出"安全状态 */
    bsp_motor_ctrl_state_enabled,   /**< 正在输出三相 PWM */
    bsp_motor_ctrl_state_disabled,  /**< 已停止输出（三相无输出） */
    bsp_motor_ctrl_state_error
} bsp_motor_ctrl_state_enum;

/** @brief 电机控制接口调用返回的状态码。 */
typedef enum {
    bsp_motor_ctrl_ok = 0,
    bsp_motor_ctrl_err_param = -1,
    bsp_motor_ctrl_err_state = -2,
    bsp_motor_ctrl_err_unsupported = -3,
    bsp_motor_ctrl_err_timeout = -4
} bsp_motor_ctrl_status_enum;

/**
 * @brief 三相序号（0 起始）。
 * @note 与 HAL 定时器函数表的通道号（1 起始）之间相差 1，由本层内部换算，
 *       上层无需知道具体通道号。
 */
typedef enum {
    bsp_motor_ctrl_phase_u = 0,
    bsp_motor_ctrl_phase_v,
    bsp_motor_ctrl_phase_w,
    bsp_motor_ctrl_phase_count
} bsp_motor_ctrl_phase_enum;

/**
 * @brief 三相 PWM 板级配置。
 * @details 只描述"本板三相桥怎么被驱动"，不含控制策略：
 *          - pwm_period：中心对齐模式下的占空比满量程（= HAL 定时器 ARR），
 *            占空比以计数表示，范围 0 … pwm_period；
 *          - is_pwm_output_inverted：输出极性。本板 TIM1 为中心对齐 + PWM 模式 2
 *            （CNT ≥ CCR 时有效），故取 true——此时 导通占空比 d 对应 CCR = period - d，
 *            d = 0（无输出）对应 CCR = period。
 */
typedef struct {
    uint16_t pwm_period;            /**< 占空比满量程（计数），对应 HAL 定时器 ARR */
    bool is_pwm_output_inverted;    /**< true：CCR = period - duty（PWM 模式 2） */
} bsp_motor_ctrl_config_t;

/**
 * @brief 电机控制状态对象：生命周期状态 + 配置 + 最近一次下发的三相占空比。
 * @details 不携带操作表，也不保存下层 HAL/平台对象指针——定时器实例由 bsp_motor_ctrl.c
 *          直接引用板级单例 hal_tim_motor。
 */
typedef struct {
    bsp_motor_ctrl_state_enum state;  /**< 对象生命周期状态 */
    bsp_motor_ctrl_config_t config;   /**< 板级三相 PWM 配置 */

    uint16_t duty_u;                  /**< 最近一次下发的 U 相导通占空比（计数） */
    uint16_t duty_v;                  /**< 最近一次下发的 V 相导通占空比（计数） */
    uint16_t duty_w;                  /**< 最近一次下发的 W 相导通占空比（计数） */
} bsp_motor_ctrl_t;

/** @brief 板级三相 PWM 电机控制对象（单实例）。 */
extern bsp_motor_ctrl_t bsp_motor_ctrl_three_phase;

/**
 * @brief 初始化三相 PWM：构建 TIM1 互补输出并把三相置于"无输出"安全状态。
 * @param p_object 电机控制对象指针。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_param 参数无效。
 * @retval bsp_motor_ctrl_err_state 定时器构建或占空比下发失败。
 */
bsp_motor_ctrl_status_enum bsp_motor_ctrl_init(bsp_motor_ctrl_t *p_object);

/**
 * @brief 下发三相导通占空比（计数域）。
 * @details 每相占空比按 config.pwm_period 限幅后，经 config.is_pwm_output_inverted
 *          换算成定时器比较值再下发；三相全部为 0 时等效于无输出。
 * @param p_object 电机控制对象指针。
 * @param duty_u U 相导通占空比（0 … config.pwm_period）。
 * @param duty_v V 相导通占空比（0 … config.pwm_period）。
 * @param duty_w W 相导通占空比（0 … config.pwm_period）。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_param 参数无效或对象未就绪。
 * @retval bsp_motor_ctrl_err_state 定时器通道拒绝该比较值。
 */
bsp_motor_ctrl_status_enum bsp_motor_ctrl_set_duty(bsp_motor_ctrl_t *p_object,
                                                   uint16_t duty_u,
                                                   uint16_t duty_v,
                                                   uint16_t duty_w);

/**
 * @brief 停止输出：三相置为"无输出"占空比，并把对象状态置为 disabled。
 * @note 本接口只把占空比降到无输出，不切断定时器主输出（MOE）；
 *       若需硬件级急停，请由应用层同时关闭主输出（当前 HAL 定时器函数表未外露该接口）。
 * @param p_object 电机控制对象指针。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_param 参数无效。
 * @retval bsp_motor_ctrl_err_state 定时器通道拒绝该比较值。
 */
bsp_motor_ctrl_status_enum bsp_motor_ctrl_stop(bsp_motor_ctrl_t *p_object);

/**
 * @brief 查询电机控制对象当前状态。
 * @param p_object 电机控制对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_motor_ctrl_state_error。
 */
bsp_motor_ctrl_state_enum bsp_motor_ctrl_get_state(const bsp_motor_ctrl_t *p_object);

/**
 * @brief BSP 三相 PWM 控制函数表：初始化 / 三相占空比下发 / 停止输出。
 * @details 表中的函数指针在 bsp_motor_ctrl.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 bsp_motor_ctrl_ops 指针统一调用。
 */
typedef struct {
    /** @brief 初始化三相 PWM（见 bsp_motor_ctrl_init）。 */
    bsp_motor_ctrl_status_enum (*pf_init)(bsp_motor_ctrl_t *p_object);
    /** @brief 下发三相导通占空比（见 bsp_motor_ctrl_set_duty）。 */
    bsp_motor_ctrl_status_enum (*pf_set_duty)(bsp_motor_ctrl_t *p_object,
                                              uint16_t duty_u,
                                              uint16_t duty_v,
                                              uint16_t duty_w);
    /** @brief 停止输出（见 bsp_motor_ctrl_stop）。 */
    bsp_motor_ctrl_status_enum (*pf_stop)(bsp_motor_ctrl_t *p_object);
} bsp_motor_ctrl_ops_t;

/** @brief BSP 三相 PWM 控制函数表指针（在 bsp_motor_ctrl.c 中定义并绑定）。 */
extern const bsp_motor_ctrl_ops_t *const bsp_motor_ctrl_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_CTRL_H__ */
