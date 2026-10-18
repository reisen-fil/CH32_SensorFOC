/**
 * @file bsp_motor_ctrl.c
 * @brief 三相电机 PWM 控制 BSP 实现（接口对 HAL 定时器函数表的简单封装）。
 * @details 结构与本层 bsp_encoder.c / bsp_led.c 同构：
 *          - bsp_motor_ctrl_init() 经 HAL 定时器函数表 hal_tim_ops->pf_motor_pwm_config
 *            构建板级 TIM1 三相互补 PWM（含死区与主输出），随后把三相比较值置为无输出；
 *          - bsp_motor_ctrl_set_duty() / bsp_motor_ctrl_stop() 经 hal_tim_ops->pf_set_compare
 *            下发三相导通占空比（按 config.is_pwm_output_inverted 换算比较值）。
 *          本文件只做"占空比 → 硬件动作"的执行，不含任何控制算法；对象只保存状态。
 */

/*
    待测项：
    1、BSP层三相PWM发波测试（完成）
    2、BSP层三相停止发波测试（完成）

*/

#include "bsp_motor_ctrl.h"
#include <stddef.h>

/* ==========================================================================
 * 板级常量与配置
 * ========================================================================== */

/**
 * @brief HAL 定时器通道号基数（HAL 侧通道号 1 起始，本层相序号 0 起始）。
 */
#define BSP_MOTOR_CTRL_CHANNEL_BASE        (1U)

/** @brief 板级三相 PWM 配置（TIM1 中心对齐 + PWM 模式 2，故输出极性取反）。 */
static const bsp_motor_ctrl_config_t s_motor_ctrl_config = {
    .pwm_period = HAL_TIM_PWM_PERIOD,
    .is_pwm_output_inverted = true
};

/** @brief 板级三相 PWM 电机控制对象（单实例，只保存配置与最近下发的占空比）。 */
bsp_motor_ctrl_t bsp_motor_ctrl_three_phase = {0};

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
 * @brief 把导通占空比（计数）按配置极性换算为定时器比较值。
 * @param p_config 板级配置。
 * @param duty 导通占空比（调用前已限幅到 0 … pwm_period）。
 * @return 定时器比较值（CCR）。
 */
static uint16_t bsp_motor_ctrl_duty_to_compare(
    const bsp_motor_ctrl_config_t *p_config,
    uint16_t duty)
{
    if (p_config->is_pwm_output_inverted) {
        return (uint16_t)(p_config->pwm_period - duty);
    }

    return duty;
}

/**
 * @brief 把导通占空比限幅到 0 … pwm_period。
 * @param p_config 板级配置。
 * @param duty 原始占空比。
 * @return 限幅后的占空比。
 */
static uint16_t bsp_motor_ctrl_clamp_duty(const bsp_motor_ctrl_config_t *p_config,
                                          uint16_t duty)
{
    if (duty > p_config->pwm_period) {
        return p_config->pwm_period;
    }

    return duty;
}

/**
 * @brief 下发单相占空比（限幅 + 极性换算 + 经 HAL 函数表写比较值）。
 * @param p_object 电机控制对象指针。
 * @param phase 相序号，取 bsp_motor_ctrl_phase_x。
 * @param duty 导通占空比（计数）。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_state 定时器通道拒绝该比较值。
 */
static bsp_motor_ctrl_status_enum bsp_motor_ctrl_write_phase(
    bsp_motor_ctrl_t *p_object,
    uint8_t phase,
    uint16_t duty)
{
    uint16_t compare_value;
    uint8_t channel;

    duty = bsp_motor_ctrl_clamp_duty(&p_object->config, duty);
    compare_value = bsp_motor_ctrl_duty_to_compare(&p_object->config, duty);
    channel = (uint8_t)(phase + BSP_MOTOR_CTRL_CHANNEL_BASE);

    if (hal_tim_ops->pf_set_compare(&hal_tim_motor, channel, compare_value)
        != hal_tim_ok) {
        return bsp_motor_ctrl_err_state;
    }

    switch (phase) {
    case (uint8_t)bsp_motor_ctrl_phase_u:
        p_object->duty_u = duty;
        break;
    case (uint8_t)bsp_motor_ctrl_phase_v:
        p_object->duty_v = duty;
        break;
    default:
        p_object->duty_w = duty;
        break;
    }

    return bsp_motor_ctrl_ok;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化三相 PWM：构建 TIM1 互补输出并把三相置于"无输出"安全状态。
 * @param p_object 电机控制对象指针。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_param 参数无效。
 * @retval bsp_motor_ctrl_err_state 定时器构建或占空比下发失败。
 */
bsp_motor_ctrl_status_enum bsp_motor_ctrl_init(bsp_motor_ctrl_t *p_object)
{
    if (p_object == NULL) {
        return bsp_motor_ctrl_err_param;
    }

    /* 板级 TIM1 三相互补 PWM（引脚组 + 时基 + 三通道 + 死区 + 主输出）——经 HAL 函数表构建 */
    if (hal_tim_ops->pf_build(&hal_tim_motor,hal_tim_role_motor_pwm) != hal_tim_ok) {
        p_object->state = bsp_motor_ctrl_state_error;
        return bsp_motor_ctrl_err_state;
    }

    /* 下发板级配置并清零占空比记录 */
    p_object->config = s_motor_ctrl_config;
    p_object->duty_u = 0U;
    p_object->duty_v = 0U;
    p_object->duty_w = 0U;

    /* 三相置于"无输出"安全状态（占空比 0） */
    if ((bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_u, 0U)
         != bsp_motor_ctrl_ok) ||
        (bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_v, 0U)
         != bsp_motor_ctrl_ok) ||
        (bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_w, 0U)
         != bsp_motor_ctrl_ok)) {
        p_object->state = bsp_motor_ctrl_state_error;
        return bsp_motor_ctrl_err_state;
    }

    p_object->state = bsp_motor_ctrl_state_ready;
    return bsp_motor_ctrl_ok;
}

/**
 * @brief 下发三相导通占空比（计数域，0 … config.pwm_period）。
 * @param p_object 电机控制对象指针。
 * @param duty_u U 相导通占空比。
 * @param duty_v V 相导通占空比。
 * @param duty_w W 相导通占空比。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_param 参数无效或对象未就绪。
 * @retval bsp_motor_ctrl_err_state 定时器通道拒绝该比较值。
 */
bsp_motor_ctrl_status_enum bsp_motor_ctrl_set_duty(bsp_motor_ctrl_t *p_object,
                                                   uint16_t duty_u,
                                                   uint16_t duty_v,
                                                   uint16_t duty_w)
{
    /* 就绪判断直接内联在此处，不再单独封装 */
    if ((p_object == NULL) ||
        ((p_object->state != bsp_motor_ctrl_state_ready) &&
         (p_object->state != bsp_motor_ctrl_state_enabled) &&
         (p_object->state != bsp_motor_ctrl_state_disabled))) {
        return bsp_motor_ctrl_err_param;
    }

    if ((bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_u, duty_u)
         != bsp_motor_ctrl_ok) ||
        (bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_v, duty_v)
         != bsp_motor_ctrl_ok) ||
        (bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_w, duty_w)
         != bsp_motor_ctrl_ok)) {
        p_object->state = bsp_motor_ctrl_state_error;
        return bsp_motor_ctrl_err_state;
    }

    p_object->state = bsp_motor_ctrl_state_enabled;
    return bsp_motor_ctrl_ok;
}

/**
 * @brief 停止输出：三相置为"无输出"占空比，并把对象状态置为 disabled。
 * @param p_object 电机控制对象指针。
 * @retval bsp_motor_ctrl_ok 操作成功。
 * @retval bsp_motor_ctrl_err_param 参数无效。
 * @retval bsp_motor_ctrl_err_state 定时器通道拒绝该比较值。
 */
bsp_motor_ctrl_status_enum bsp_motor_ctrl_stop(bsp_motor_ctrl_t *p_object)
{
    if (p_object == NULL) {
        return bsp_motor_ctrl_err_param;
    }

    if ((bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_u, 0U)
         != bsp_motor_ctrl_ok) ||
        (bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_v, 0U)
         != bsp_motor_ctrl_ok) ||
        (bsp_motor_ctrl_write_phase(p_object, (uint8_t)bsp_motor_ctrl_phase_w, 0U)
         != bsp_motor_ctrl_ok)) {
        p_object->state = bsp_motor_ctrl_state_error;
        return bsp_motor_ctrl_err_state;
    }

    p_object->state = bsp_motor_ctrl_state_disabled;
    return bsp_motor_ctrl_ok;
}

/**
 * @brief 查询电机控制对象当前状态。
 * @param p_object 电机控制对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_motor_ctrl_state_error。
 */
bsp_motor_ctrl_state_enum bsp_motor_ctrl_get_state(const bsp_motor_ctrl_t *p_object)
{
    if (p_object == NULL) {
        return bsp_motor_ctrl_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * BSP 三相 PWM 控制函数表
 * ========================================================================== */

/** @brief BSP 三相 PWM 控制函数表（本文件私有，绑定板级三个接口）。 */
static const bsp_motor_ctrl_ops_t s_bsp_motor_ctrl_ops = {
    .pf_init = bsp_motor_ctrl_init,
    .pf_set_duty = bsp_motor_ctrl_set_duty,
    .pf_stop = bsp_motor_ctrl_stop
};

/** @brief BSP 三相 PWM 控制函数表指针（对外外露，声明于 bsp_motor_ctrl.h）。 */
const bsp_motor_ctrl_ops_t *const bsp_motor_ctrl_ops = &s_bsp_motor_ctrl_ops;
