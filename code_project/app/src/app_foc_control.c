/**
 * @file app_foc_control.c
 * @brief 应用层 FOC 控制实现（零点标定 + Q15 d 轴角度强拖）。
 * @details 本文件只保留控制业务：角度积分、零点标定、编码器零点偏置写回。FOC 运算与
 *          三相 PWM 下发全部经服务层函数表 svc_foc_ops：
 *            svc_foc_ops->pf_get_current()      采样电流并刷新 Clarke/Park 观测值
 *            svc_foc_ops->pf_output_dq_pwm()    dq(V) → 逆 Park → SVPWM → 三相占空比计数 → BSP
 *          零点标定不再手写 switch 状态机，而是**直接注入通用状态机服务 svc_fsm**：
 *          本文件提供 energizing / sampling / done 三个状态函数与状态表，框架负责状态号、
 *          状态计时与切换（exit → enter）；app_foc_control_zero_calibrate() 只是"首次初始化
 *          状态机 + 每拍推进一次"的薄封装。
 *          本文件不做任何 BSP 外设初始化：编码器 / 电流采样 / 三相 PWM 由 app_system 集中
 *          初始化；这里只在标定时直接读编码器、写回零点偏置。
 *          量纲：dq 电压按服务层基值归一化为标幺值，电角度为 Q15 标幺值。
 */
#include "app_foc_control.h"
#include "bsp_encoder.h"
#include "bsp_current_sense.h"
#include "hal_gpio.h"
#include "hal_tim.h"

#include <stddef.h>
#include <string.h>

/** @brief Q15 电角度一周的计数值。 */
#define APP_FOC_CONTROL_ANGLE_PERIOD_Q15   (32768U)

/** @brief 允许的最大采样次数（用于配置合法性检查）。 */
#define APP_FOC_CONTROL_SAMPLE_COUNT_MAX   (64U)

/** @brief 单次采样允许的连续失败次数上限（超过则判为标定失败）。 */
#define APP_FOC_CONTROL_SAMPLE_RETRY_MAX   (3U)

/**
 * @brief 应用 FOC 控制板级配置。
 * @note 对齐电压/保持时间、采样次数/间隔与 d 轴强拖电压/频率是电气参数的占位值，需按实际
 *       电机实测调整；强拖频率必须为 PWM 开关频率的整数约数。
 */
static const app_foc_control_config_t s_app_foc_control_config = {
    .encoder_resolution_bits = 14U,
    .zero_align_voltage_q15 = PLAT_IQ(0.3f),
    .zero_align_time_us = 300000U,
    .zero_align_sample_count = 16U,
    .zero_align_sample_interval_us = 1000U,      /* 采样窗口 = 16 × 1 ms = 16 ms */
    .open_loop_ud_q15 = PLAT_IQ(0.2f),
    .open_loop_frequency_hz = 1U,       // 1Hz-20Hz
    .open_loop_adc_irq_count = 0U,
    .open_loop_angle_step_q15 = 0,
    .p_user_data = NULL
};

/** @brief 板级应用 FOC 控制对象（单实例）。 */
app_foc_control_t app_foc_control_motor = {0};

/**
 * @brief 零点标定状态机的状态号（即状态表下标）。
 * @details 只在标定过程中使用；标定阶段对外的表达仍是 app_foc_control_align_state_enum。
 */
typedef enum {
    app_foc_control_align_fsm_energizing = 0, /**< 施加对齐矢量并保持 */
    app_foc_control_align_fsm_sampling,       /**< 多次采样机械角求均值 */
    app_foc_control_align_fsm_done            /**< 写回偏置并撤矢量（终态） */
} app_foc_control_align_fsm_enum;

/**
 * @brief 判断对象指针是否可用。
 * @param p_object 应用 FOC 控制对象指针。
 * @return true 可用。
 */
static bool app_foc_control_is_object_valid(const app_foc_control_t *p_object)
{
    return (p_object != NULL);
}

/**
 * @brief 把服务层状态码映射为本层状态码。
 * @param status 服务层返回的状态码。
 * @return 对应的应用层状态码。
 */
static app_foc_control_status_enum app_foc_control_map_status(svc_foc_status_enum status)
{
    switch (status) {
    case svc_foc_ok:
        return app_foc_control_ok;
    case svc_foc_err_param:
        return app_foc_control_err_param;
    case svc_foc_err_unsupported:
        return app_foc_control_err_unsupported;
    case svc_foc_err_state:
    default:
        return app_foc_control_err_state;
    }
}

/**
 * @brief 把服务层状态码映射为状态机回调返回值。
 * @param status 服务层返回的状态码。
 * @return 对应的状态机状态码（非 ok 会让框架把状态机置 error）。
 */
static svc_fsm_status_enum app_foc_control_svc_to_fsm(svc_foc_status_enum status)
{
    switch (status) {
    case svc_foc_ok:
        return svc_fsm_ok;
    case svc_foc_err_param:
        return svc_fsm_err_param;
    case svc_foc_err_unsupported:
        return svc_fsm_err_unsupported;
    case svc_foc_err_state:
    default:
        return svc_fsm_err_state;
    }
}

/**
 * @brief 把状态机状态码映射为本层状态码。
 * @param status 状态机返回的状态码。
 * @return 对应的应用层状态码。
 */
static app_foc_control_status_enum app_foc_control_map_fsm_status(
    svc_fsm_status_enum status)
{
    switch (status) {
    case svc_fsm_ok:
        return app_foc_control_ok;
    case svc_fsm_err_param:
        return app_foc_control_err_param;
    case svc_fsm_err_unsupported:
        return app_foc_control_err_unsupported;
    case svc_fsm_err_state:
    case svc_fsm_err_timeout:
    default:
        return app_foc_control_err_state;
    }
}

/**
 * @brief 计算两个机械角计数之间的最短弧差（跨 0/单圈计数 边界安全）。
 * @details 结果落在 ±(单圈计数/2] 之内：把差值按单圈计数取模后，超过半圈的一侧折算成
 *          负值。用于多次采样求均值时避免零点位于边界处导致均值跳变。
 * @param sample_counts 本次采样机械角（计数）。
 * @param reference_counts 参考点机械角（计数）。
 * @param resolution 单圈计数。
 * @return 最短弧差（计数，带符号）。
 */
static int32_t app_foc_control_angle_delta(uint32_t sample_counts,
                                           uint32_t reference_counts,
                                           uint16_t resolution)
{
    int32_t period = (int32_t)resolution;
    int32_t half_period = period / 2;
    int32_t delta = (int32_t)sample_counts - (int32_t)reference_counts;

    delta %= period;
    if (delta > half_period) {
        delta -= period;
    } else if (delta < -half_period) {
        delta += period;
    }

    return delta;
}

/**
 * @brief 状态机回调：进入 energizing —— 施加对齐矢量并复位采样统计。
 * @param p_fsm 状态机对象指针（本回调不使用）。
 * @param p_context 宿主对象指针（app_foc_control_t *）。
 * @param elapsed_us 本次周期时长（进入回调固定为 0，未使用）。
 * @return svc_fsm_ok 表示矢量已下发。
 */
static svc_fsm_status_enum app_foc_control_align_enter_energizing(svc_fsm_t *p_fsm,
                                                                  void *p_context,
                                                                  uint32_t elapsed_us)
{
    app_foc_control_t *p_object = (app_foc_control_t *)p_context;
    svc_foc_value_t angle_pu = {0};
    svc_foc_value_t align_voltage_q15 = {0};
    svc_foc_value_t zero_voltage_q15 = {0};

    (void)p_fsm;
    (void)elapsed_us;

    if (p_object == NULL) {
        return svc_fsm_err_param;
    }
    if ((p_object->state != app_foc_control_state_ready) &&
        (p_object->state != app_foc_control_state_running)) {
        return svc_fsm_err_state;
    }

    /* 对齐矢量直接按 Q15 标幺值设置：角度 0、Ud=config、Uq=0。 */
    angle_pu.q_value = 0;
    align_voltage_q15.q_value = p_object->config.zero_align_voltage_q15;
    zero_voltage_q15.q_value = 0;

    if (app_foc_control_svc_to_fsm(
            svc_foc_ops->pf_output_dq_pwm(&svc_foc_motor, align_voltage_q15,
                                          zero_voltage_q15, angle_pu)) != svc_fsm_ok) {
        return svc_fsm_err_state;
    }

    /* 复位采样统计（采样阶段的首次采样不额外等待间隔） */
    p_object->zero_align_sample_index = 0U;
    p_object->zero_align_retry_count = 0U;
    p_object->zero_align_reference_counts = 0U;
    p_object->zero_align_delta_sum = 0;
    p_object->zero_align_sample_timer_us = p_object->config.zero_align_sample_interval_us;

    p_object->zero_align_state = app_foc_control_align_energizing;
    p_object->state = app_foc_control_state_running;

    return svc_fsm_ok;
}

/**
 * @brief 状态机回调：energizing 周期动作 —— 保持时间到则请求切到 sampling。
 * @param p_fsm 状态机对象指针（用于读取状态累计时间）。
 * @param p_context 宿主对象指针（app_foc_control_t *）。
 * @param elapsed_us 本次周期时长（本状态用框架累计时间判断，未使用）。
 * @return svc_fsm_ok 表示正常。
 */
static svc_fsm_status_enum app_foc_control_align_run_energizing(svc_fsm_t *p_fsm,
                                                                void *p_context,
                                                                uint32_t elapsed_us)
{
    app_foc_control_t *p_object = (app_foc_control_t *)p_context;

    (void)elapsed_us;

    if ((p_fsm == NULL) || (p_object == NULL)) {
        return svc_fsm_err_param;
    }

    if (svc_fsm_ops->pf_get_state_elapsed_us(p_fsm)
        >= p_object->config.zero_align_time_us) {
        return svc_fsm_ops->pf_request(p_fsm, (uint16_t)app_foc_control_align_fsm_sampling);
    }

    return svc_fsm_ok;
}

/**
 * @brief 状态机回调：进入 sampling —— 复位采样统计与错误重试计数。
 * @param p_fsm 状态机对象指针（未使用）。
 * @param p_context 宿主对象指针（app_foc_control_t *）。
 * @param elapsed_us 本次周期时长（进入回调固定为 0，未使用）。
 * @return svc_fsm_ok 表示正常。
 */
static svc_fsm_status_enum app_foc_control_align_enter_sampling(svc_fsm_t *p_fsm,
                                                                void *p_context,
                                                                uint32_t elapsed_us)
{
    app_foc_control_t *p_object = (app_foc_control_t *)p_context;

    (void)p_fsm;
    (void)elapsed_us;

    if (p_object == NULL) {
        return svc_fsm_err_param;
    }

    p_object->zero_align_sample_index = 0U;
    p_object->zero_align_retry_count = 0U;
    p_object->zero_align_reference_counts = 0U;
    p_object->zero_align_delta_sum = 0;
    p_object->zero_align_sample_timer_us = p_object->config.zero_align_sample_interval_us;

    p_object->zero_align_state = app_foc_control_align_sampling;

    return svc_fsm_ok;
}

/**
 * @brief 状态机回调：sampling 周期动作 —— 按间隔采样机械角，采满则请求切到 done。
 * @details 首次采样作为参考点，后续只累加相对参考点的最短弧差，均值在 done 阶段计算。
 *          单次 CRC 失败作废本次采样，下次间隔重试；连续失败超限则返回错误让框架置 error。
 * @param p_fsm 状态机对象指针（用于请求切换）。
 * @param p_context 宿主对象指针（app_foc_control_t *）。
 * @param elapsed_us 本次周期时长（µs）。
 * @return svc_fsm_ok 表示正常。
 */
static svc_fsm_status_enum app_foc_control_align_run_sampling(svc_fsm_t *p_fsm,
                                                              void *p_context,
                                                              uint32_t elapsed_us)
{
    app_foc_control_t *p_object = (app_foc_control_t *)p_context;
    uint32_t raw_mechanical;

    if ((p_fsm == NULL) || (p_object == NULL)) {
        return svc_fsm_err_param;
    }

    p_object->zero_align_sample_timer_us += elapsed_us;
    if (p_object->zero_align_sample_timer_us < p_object->config.zero_align_sample_interval_us) {
        return svc_fsm_ok;
    }
    p_object->zero_align_sample_timer_us = 0U;

    /* 采一次机械角 */
    if (bsp_encoder_mt6701_ops->pf_get_mechanical_angle(&bsp_encoder_mt6701)
        != bsp_encoder_ok) {
        p_object->zero_align_retry_count++;
        if (p_object->zero_align_retry_count > APP_FOC_CONTROL_SAMPLE_RETRY_MAX) {
            return svc_fsm_err_state;
        }
        return svc_fsm_ok;    /* 本次作废，下次间隔重试 */
    }
    p_object->zero_align_retry_count = 0U;

    raw_mechanical = (uint32_t)bsp_encoder_mt6701.config.raw_mechanical_angle;

    if (p_object->zero_align_sample_index == 0U) {
        /* 首次采样作为参考点 */
        p_object->zero_align_reference_counts = raw_mechanical;
        p_object->zero_align_delta_sum = 0;
    } else {
        p_object->zero_align_delta_sum +=
            app_foc_control_angle_delta(raw_mechanical,
                                        p_object->zero_align_reference_counts,
                                        p_object->encoder_resolution);
    }
    p_object->zero_align_sample_index++;

    if (p_object->zero_align_sample_index >= p_object->config.zero_align_sample_count) {
        return svc_fsm_ops->pf_request(p_fsm, (uint16_t)app_foc_control_align_fsm_done);
    }

    return svc_fsm_ok;
}

/**
 * @brief 状态机回调：进入 done —— 多次采样求平均、反算并写回零点偏置、撤掉对齐矢量。
 * @details 均值算法：以首次采样为参考点，累加各次采样的最短弧差后取平均，再折回
 *          0 ~ 单圈计数，得到零点位置的机械角；随后按 BSP 侧公式
 *          zero_aligned = (raw + zero_offset) & mask 反算偏置。
 * @param p_fsm 状态机对象指针（未使用）。
 * @param p_context 宿主对象指针（app_foc_control_t *）。
 * @param elapsed_us 本次周期时长（进入回调固定为 0，未使用）。
 * @return svc_fsm_ok 表示标定已完成。
 */
static svc_fsm_status_enum app_foc_control_align_enter_done(svc_fsm_t *p_fsm,
                                                            void *p_context,
                                                            uint32_t elapsed_us)
{
    app_foc_control_t *p_object = (app_foc_control_t *)p_context;
    svc_foc_value_t angle_pu = {0};
    svc_foc_value_t zero_voltage_q15 = {0};
    int32_t resolution;
    int32_t sample_count;
    int32_t average_counts;

    (void)p_fsm;
    (void)elapsed_us;

    if (p_object == NULL) {
        return svc_fsm_err_param;
    }

    resolution = (int32_t)p_object->encoder_resolution;
    sample_count = (int32_t)p_object->config.zero_align_sample_count;
    if ((resolution <= 0) || (sample_count <= 0)) {
        return svc_fsm_err_param;
    }

    /* 均值：参考点 + 平均最短弧差，再折回 0 ~ 单圈计数 */
    average_counts = ((int32_t)p_object->zero_align_reference_counts
                      + (p_object->zero_align_delta_sum / sample_count)) % resolution;
    if (average_counts < 0) {
        average_counts += resolution;
    }

    /* BSP 侧对齐公式为 zero_aligned = (raw_mechanical + zero_offset) & mask，
       故要让零点位置成为电角度零点，取 zero_offset = (单圈计数 - 平均机械角) % 单圈计数 */
    p_object->zero_offset_counts = (uint32_t)((resolution - average_counts) % resolution);

    bsp_encoder_mt6701.config.zero_offset = p_object->zero_offset_counts;
    bsp_encoder_mt6701.config.is_zero_aligned = true;

    /* 刷新零点标定后的机械/电角度及其标幺值 */
    (void)bsp_encoder_mt6701_ops->pf_get_electrical_angle(&bsp_encoder_mt6701);

    /* 撤掉对齐矢量：Q15 dq = (0, 0)，线电压为零。 */
    angle_pu.q_value = 0;
    zero_voltage_q15.q_value = 0;
    if (app_foc_control_svc_to_fsm(
            svc_foc_ops->pf_output_dq_pwm(&svc_foc_motor, zero_voltage_q15,
                                          zero_voltage_q15, angle_pu)) != svc_fsm_ok) {
        return svc_fsm_err_state;
    }

    p_object->is_zero_aligned = true;
    p_object->open_loop_angle_pu_q = 0;
    p_object->zero_align_state = app_foc_control_align_done;
    p_object->state = app_foc_control_state_ready;

    return svc_fsm_ok;
}

/** @brief 零点标定状态表（状态号 = 数组下标；done 为无周期动作的终态）。 */
static const svc_fsm_state_t s_app_foc_control_align_states[] = {
    {
        .p_name = "energizing",
        .pf_enter = app_foc_control_align_enter_energizing,
        .pf_run = app_foc_control_align_run_energizing,
        .pf_exit = NULL
    },
    {
        .p_name = "sampling",
        .pf_enter = app_foc_control_align_enter_sampling,
        .pf_run = app_foc_control_align_run_sampling,
        .pf_exit = NULL
    },
    {
        .p_name = "done",
        .pf_enter = app_foc_control_align_enter_done,
        .pf_run = NULL,
        .pf_exit = NULL
    }
};

/**
 * @brief 零点标定状态机板级配置。
 * @note p_context 在第一次标定时按实例绑定为宿主对象指针，故这里留空。
 */
static const svc_fsm_config_t s_app_foc_control_align_fsm_config = {
    .p_states = s_app_foc_control_align_states,
    .state_count =
        (uint16_t)(sizeof(s_app_foc_control_align_states)
                   / sizeof(s_app_foc_control_align_states[0])),
    .initial_state_index = (uint16_t)app_foc_control_align_fsm_energizing,
    .p_context = NULL
};

/**
 * @brief 初始化应用 FOC 控制对象：注入配置并预计算强拖相位步进。
 * @param p_object 应用 FOC 控制对象指针。
 * @retval app_foc_control_ok 操作成功。
 * @retval app_foc_control_err_param 参数无效或配置非法。
 * @retval app_foc_control_err_unsupported 服务层运算类型或调制方式不受支持。
 */
app_foc_control_status_enum app_foc_control_init(app_foc_control_t *p_object)
{
    if (!app_foc_control_is_object_valid(p_object)) {
        return app_foc_control_err_param;
    }

    (void)memset(p_object, 0, sizeof(*p_object));
    p_object->config = s_app_foc_control_config;

    if ((p_object->config.encoder_resolution_bits == 0U) ||
        (p_object->config.encoder_resolution_bits > 16U) ||
        (p_object->config.zero_align_time_us == 0U) ||
        (p_object->config.zero_align_sample_count == 0U) ||
        (p_object->config.zero_align_sample_count > APP_FOC_CONTROL_SAMPLE_COUNT_MAX) ||
        (p_object->config.zero_align_sample_interval_us == 0U) ||
        (p_object->config.open_loop_frequency_hz == 0U) ||
        (HAL_TIM_PWM_FREQUENCY_HZ % p_object->config.open_loop_frequency_hz != 0U) ||
        (p_object->config.open_loop_frequency_hz > HAL_TIM_PWM_FREQUENCY_HZ)) {
        return app_foc_control_err_param;
    }

    p_object->encoder_resolution =
        (uint16_t)(1U << p_object->config.encoder_resolution_bits);

#if defined(FOC_USE_FLOAT)
    return app_foc_control_err_unsupported;
#endif

    p_object->config.open_loop_adc_irq_count = (uint16_t)(
        HAL_TIM_PWM_FREQUENCY_HZ / p_object->config.open_loop_frequency_hz);
    p_object->config.open_loop_angle_step_q15 = PLAT_IQ_DIV(
        PLAT_IQ(1.0f),
        PLAT_IQ(p_object->config.open_loop_adc_irq_count));
    if ((p_object->config.open_loop_adc_irq_count == 0U) ||
        (p_object->config.open_loop_angle_step_q15 == 0)) {
        return app_foc_control_err_param;
    }

    /* 标定状态机在第一次标定时才初始化，保证初始化阶段不会给电机加电压 */
    p_object->zero_align_state = app_foc_control_align_idle;
    p_object->state = app_foc_control_state_ready;

    return app_foc_control_ok;
}

/**
 * @brief 零点标定（整合入口，非阻塞）：首次调用初始化状态机并进入 energizing，之后每拍
 *        推进一次，直到状态机进入 done（零点偏置已写回编码器对象）。
 * @param p_object 应用 FOC 控制对象指针。
 * @param elapsed_us 距上次调用的时间（µs），用于保持计时与采样间隔计时。
 * @retval app_foc_control_ok 操作成功（含"已完成"的空操作）。
 * @retval app_foc_control_err_param 参数无效。
 * @retval app_foc_control_err_state 对象未就绪、编码器连续采样失败或服务层/PWM 下发失败。
 * @retval app_foc_control_err_unsupported 服务层运算类型或调制方式不受支持。
 */
app_foc_control_status_enum app_foc_control_zero_calibrate(app_foc_control_t *p_object,
                                                           uint32_t elapsed_us)
{
    svc_fsm_config_t fsm_config;
    svc_fsm_status_enum fsm_status;

    if (!app_foc_control_is_object_valid(p_object)) {
        return app_foc_control_err_param;
    }
    if (p_object->state == app_foc_control_state_error) {
        return app_foc_control_err_state;
    }
    if (p_object->zero_align_state == app_foc_control_align_done) {
        return app_foc_control_ok;                 /* 已完成：空操作 */
    }

    /* 首次调用：绑定宿主对象为上下文，初始化状态机（enter 中施加对齐矢量） */
    if (p_object->zero_align_state == app_foc_control_align_idle) {
        fsm_config = s_app_foc_control_align_fsm_config;
        fsm_config.p_context = p_object;

        fsm_status = svc_fsm_ops->pf_init(&p_object->zero_align_fsm, &fsm_config);
        if (fsm_status != svc_fsm_ok) {
            p_object->zero_align_state = app_foc_control_align_error;
            p_object->state = app_foc_control_state_error;
            return app_foc_control_map_fsm_status(fsm_status);
        }
        return app_foc_control_ok;
    }

    /* 之后每拍推进一次状态机 */
    fsm_status = svc_fsm_ops->pf_run(&p_object->zero_align_fsm, elapsed_us);
    if (fsm_status != svc_fsm_ok) {
        p_object->zero_align_state = app_foc_control_align_error;
        p_object->state = app_foc_control_state_error;
        return app_foc_control_map_fsm_status(fsm_status);
    }

    return app_foc_control_ok;
}

/**
 * @brief 按 ADC 中断频率推进 Q15 d 轴电角度强拖。
 * @details 每次 ADC 注入中断调用一次；按 init 预计算的 Q15 步进更新电角度并输出
 *          Ud、Uq=0 的 d 轴矢量，不依赖编码器零点对齐状态。
 * @param p_object 应用 FOC 控制对象指针。
 * @retval app_foc_control_ok 操作成功。
 * @retval app_foc_control_err_param 参数无效。
 * @retval app_foc_control_err_state 对象未就绪或服务层/PWM 拒绝。
 */
app_foc_control_status_enum app_foc_control_open_loop_d_q15(app_foc_control_t *p_object)
{
    app_foc_control_status_enum status;
    svc_foc_value_t angle_pu = {0};
    svc_foc_value_t ud_voltage_q15 = {0};
    svc_foc_value_t zero_voltage_q15 = {0};

    if (!app_foc_control_is_object_valid(p_object)) {
        return app_foc_control_err_param;
    }
    if ((p_object->state != app_foc_control_state_ready) &&
        (p_object->state != app_foc_control_state_running)) {
        return app_foc_control_err_state;
    }
    if ((p_object->config.open_loop_adc_irq_count == 0U) ||
        (p_object->config.open_loop_angle_step_q15 <= 0)) {
        return app_foc_control_err_param;
    }

    /* 累加 Q15 标幺角度步进，越过一周上限后回绕到零。 */
    p_object->open_loop_angle_pu_q += p_object->config.open_loop_angle_step_q15;
    if (p_object->open_loop_angle_pu_q >= APP_FOC_CONTROL_ANGLE_PERIOD_Q15) {
        p_object->open_loop_angle_pu_q = 0;
    }
    angle_pu.q_value = p_object->open_loop_angle_pu_q;

    /* 电流观测失败不阻止强拖输出。 */
    // (void)svc_foc_ops->pf_get_current(&svc_foc_motor, angle_pu);

    /* d 轴强拖参数已是 Q15 标幺值，直接送入服务层；Uq 固定为零。 */
    ud_voltage_q15.q_value = p_object->config.open_loop_ud_q15;
    zero_voltage_q15.q_value = 0;
    status = app_foc_control_map_status(
        svc_foc_ops->pf_output_dq_pwm(&svc_foc_motor,
                                      ud_voltage_q15,
                                      zero_voltage_q15,
                                      angle_pu));
    if (status != app_foc_control_ok) {
        return status;
    }

    p_object->state = app_foc_control_state_running;
    return app_foc_control_ok;
}

/**
 * @brief 查询应用 FOC 控制对象当前状态。
 * @param p_object 应用 FOC 控制对象指针。
 * @return 当前状态；参数为空时返回 app_foc_control_state_error。
 */
app_foc_control_state_enum app_foc_control_get_state(const app_foc_control_t *p_object)
{
    if (p_object == NULL) {
        return app_foc_control_state_error;
    }
    return p_object->state;
}

/** @brief 应用 FOC 控制函数表（本文件私有，只包含对外暴露的四组接口）。 */
static const app_foc_control_ops_t s_app_foc_control_ops = {
    .pf_init = app_foc_control_init,
    .pf_zero_calibrate = app_foc_control_zero_calibrate,
    .pf_open_loop_d_q15 = app_foc_control_open_loop_d_q15,
    .pf_get_state = app_foc_control_get_state
};

/** @brief 应用 FOC 控制函数表指针（绑定到本文件的私有函数表）。 */
const app_foc_control_ops_t *const app_foc_control_ops = &s_app_foc_control_ops;

/* ADC注入组中断线测试 */

bool irq_count = 0;

PLAT_ISR_DEFINE(PLAT_ISR_ADC1_2)
{
    bool is_set;
    (void)hal_adc_ops->pf_get_flag(&hal_adc_injected, PLAT_ADC_FLAG_JEOC, &is_set);

    if(is_set)
    {
        PLAT_PERIPH_ADC2->STATR &= ~(0x04);

        if(!irq_count)
        {
            irq_count = 1;
            hal_gpio_ops->pf_write(&hal_gpio_io_test,hal_gpio_test_pin_1,1);
        }
        else
        {
            irq_count = 0;
            hal_gpio_ops->pf_write(&hal_gpio_io_test,hal_gpio_test_pin_1,0);
        }

        // bsp_current_sense_calc(&bsp_current_sense_three_phase);
        // (void)app_foc_control_open_loop_d_q15(&app_foc_control_motor);
    }
}
