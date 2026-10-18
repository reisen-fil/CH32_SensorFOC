/**
 * @file platform_ch32v203_tim.c
 * @brief CH32V203 定时器平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 TIM 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_tim_ops 函数表对外绑定；hal_tim.c 通过该表访问定时器硬件。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；厂商类型名一律经
 *       platform_ch32v203_tim.h 的"类型接入宏"（PLAT_TIMER_*_TYPE）书写，
 *       本文件内不出现任何厂商类型名。时钟使能沿用平台时钟宏
 *       PLAT_RCC_APB2_CLOCK_ENABLE / PLAT_RCC_APB1_CLOCK_ENABLE，中断配置沿用
 *       PLAT_PFIC_INIT。
 * @note 通道号对外统一使用 PLAT_TIMER_CHANNEL_1..4（1 起始），在本文件内翻译成
 *       标准库的 TIM_Channel_x（TIM_OCxInit / TIM_OCxPreloadConfig 系列 API 直接
 *       以"哪个函数"区分通道，故用 switch 分派；TIM_SetCompare 系列同理）。
 */
#include "platform_ch32v203_tim.h"
#include <stddef.h>

/**
 * @brief 判断实例是否为受支持的定时器外设。
 * @param instance 实例基地址，取 PLAT_PERIPH_TIM1..TIM5。
 * @retval true 受支持。
 * @retval false 其它外设。
 */
static bool plat_ch32v203_tim_is_valid(uintptr_t instance)
{
    PLAT_TIMER_INSTANCE_TYPE *p_tim = (PLAT_TIMER_INSTANCE_TYPE *)(instance);

    return (p_tim == PLAT_PERIPH_TIM1) || (p_tim == PLAT_PERIPH_TIM2) ||
           (p_tim == PLAT_PERIPH_TIM3) || (p_tim == PLAT_PERIPH_TIM4) ||
           (p_tim == PLAT_PERIPH_TIM5);
}

/**
 * @brief 判断实例是否为高级定时器（带 BDTR/MOE 的 TIM1）。
 * @details 标准库的 TIM_BDTRConfig 与 TIM_CtrlPWMOutputs 会无条件读写 TIMx->BDTR，
 *          而 TIM2..TIM5 没有 BDTR 寄存器（该偏移为保留区），故这两个接口只对
 *          TIM1 开放，避免误写通用定时器的保留寄存器。
 * @param instance 实例基地址。
 * @retval true 是高级定时器（TIM1）。
 * @retval false 通用定时器或非法实例。
 */
static bool plat_ch32v203_tim_is_advanced(uintptr_t instance)
{
    return ((PLAT_TIMER_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_TIM1);
}

/**
 * @brief 使能定时器实例时钟。
 * @details TIM1 挂在 APB2，TIM2..TIM5 挂在 APB1。
 * @note 时钟使能是本文件通过平台宏完成的动作之一。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_clock_enable(uintptr_t instance)
{
    PLAT_TIMER_INSTANCE_TYPE *p_tim = (PLAT_TIMER_INSTANCE_TYPE *)(instance);

    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    if (p_tim == PLAT_PERIPH_TIM1) {
        PLAT_RCC_APB2_CLOCK_ENABLE(PLAT_RCC_APB2_TIM1);
    } else if (p_tim == PLAT_PERIPH_TIM2) {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_TIM2);
    } else if (p_tim == PLAT_PERIPH_TIM3) {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_TIM3);
    } else if (p_tim == PLAT_PERIPH_TIM4) {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_TIM4);
    } else {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_TIM5);
    }

    return true;
}

/**
 * @brief 复位定时器外设（TIM_DeInit）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_deinit(uintptr_t instance)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_DeInit((PLAT_TIMER_INSTANCE_TYPE *)(instance));
    return true;
}

/**
 * @brief 配置时基（TIM_TimeBaseInit）。
 * @param instance 实例基地址。
 * @param p_config 平台时基配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持或配置为空。
 */
static bool plat_ch32v203_tim_time_base_config(
    uintptr_t instance,
    const platform_ch32v203_tim_time_base_config_t *p_config)
{
    PLAT_TIMER_TIME_BASE_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_tim_is_valid(instance))) {
        return false;
    }

    init_config.TIM_Period = p_config->period;
    init_config.TIM_Prescaler = p_config->prescaler;
    init_config.TIM_ClockDivision = p_config->clock_division;
    init_config.TIM_CounterMode = p_config->counter_mode;
    init_config.TIM_RepetitionCounter = p_config->repetition_counter;

    TIM_TimeBaseInit((PLAT_TIMER_INSTANCE_TYPE *)(instance), &init_config);
    return true;
}

/**
 * @brief 配置指定通道的 PWM 输出（TIM_OC1/2/3/4Init）。
 * @param instance 实例基地址。
 * @param channel 通道号，取 PLAT_TIMER_CHANNEL_x。
 * @param p_config 输出比较配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持、配置为空或通道号越界。
 */
static bool plat_ch32v203_tim_oc_channel_config(
    uintptr_t instance,
    uint8_t channel,
    const platform_ch32v203_tim_oc_config_t *p_config)
{
    PLAT_TIMER_OC_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_tim_is_valid(instance))) {
        return false;
    }

    init_config.TIM_OCMode = p_config->oc_mode;
    init_config.TIM_OutputState = p_config->output_state;
    init_config.TIM_OutputNState = p_config->output_n_state;
    init_config.TIM_Pulse = p_config->pulse;
    init_config.TIM_OCPolarity = p_config->oc_polarity;
    init_config.TIM_OCNPolarity = p_config->ocn_polarity;
    init_config.TIM_OCIdleState = p_config->oc_idle_state;
    init_config.TIM_OCNIdleState = p_config->ocn_idle_state;

    switch (channel) {
    case PLAT_TIMER_CHANNEL_1:
        TIM_OC1Init((PLAT_TIMER_INSTANCE_TYPE *)(instance), &init_config);
        break;
    case PLAT_TIMER_CHANNEL_2:
        TIM_OC2Init((PLAT_TIMER_INSTANCE_TYPE *)(instance), &init_config);
        break;
    case PLAT_TIMER_CHANNEL_3:
        TIM_OC3Init((PLAT_TIMER_INSTANCE_TYPE *)(instance), &init_config);
        break;
    case PLAT_TIMER_CHANNEL_4:
        TIM_OC4Init((PLAT_TIMER_INSTANCE_TYPE *)(instance), &init_config);
        break;
    default:
        return false;
    }

    return true;
}

/**
 * @brief 配置死区与刹车（TIM_BDTRConfig）。
 * @note 仅高级定时器（TIM1）支持：TIM2..TIM5 无 BDTR 寄存器。
 * @param instance 实例基地址。
 * @param p_config 死区与刹车配置。
 * @retval true 操作成功。
 * @retval false 实例不是高级定时器、实例不受支持或配置为空。
 */
static bool plat_ch32v203_tim_bdtr_config(
    uintptr_t instance,
    const platform_ch32v203_tim_bdtr_config_t *p_config)
{
    PLAT_TIMER_BDTR_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_tim_is_advanced(instance))) {
        return false;
    }

    init_config.TIM_OSSRState = p_config->ossr_state;
    init_config.TIM_OSSIState = p_config->ossi_state;
    init_config.TIM_LOCKLevel = p_config->lock_level;
    init_config.TIM_DeadTime = p_config->dead_time;
    init_config.TIM_Break = p_config->break_state;
    init_config.TIM_BreakPolarity = p_config->break_polarity;
    init_config.TIM_AutomaticOutput = p_config->automatic_output;

    TIM_BDTRConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), &init_config);
    return true;
}

/**
 * @brief 使能/禁止指定通道的比较寄存器预装载（TIM_OCxPreloadConfig）。
 * @param instance 实例基地址。
 * @param channel 通道号，取 PLAT_TIMER_CHANNEL_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持或通道号越界。
 */
static bool plat_ch32v203_tim_oc_preload_config(uintptr_t instance,
                                                uint8_t channel,
                                                bool enable)
{
    uint16_t preload;

    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    preload = enable ? PLAT_TIMER_OC_PRELOAD_ENABLE : PLAT_TIMER_OC_PRELOAD_DISABLE;

    switch (channel) {
    case PLAT_TIMER_CHANNEL_1:
        TIM_OC1PreloadConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), preload);
        break;
    case PLAT_TIMER_CHANNEL_2:
        TIM_OC2PreloadConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), preload);
        break;
    case PLAT_TIMER_CHANNEL_3:
        TIM_OC3PreloadConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), preload);
        break;
    case PLAT_TIMER_CHANNEL_4:
        TIM_OC4PreloadConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), preload);
        break;
    default:
        return false;
    }

    return true;
}

/**
 * @brief 使能/禁止自动重装载寄存器预装载（TIM_ARRPreloadConfig）。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_arr_preload_config(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_ARRPreloadConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止主输出 MOE（TIM_CtrlPWMOutputs）。
 * @note 仅高级定时器（TIM1）支持：MOE 关闭时六路 PWM 引脚被强制到空闲电平；
 *       TIM2..TIM5 无 BDTR 寄存器，调用本接口返回 false。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不是高级定时器或实例不受支持。
 */
static bool plat_ch32v203_tim_pwm_outputs_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_tim_is_advanced(instance)) {
        return false;
    }

    TIM_CtrlPWMOutputs((PLAT_TIMER_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 选择主输出触发（TRGO）源（TIM_SelectOutputTrigger）。
 * @details 本工程 TIM1 的 TRGO 取"更新事件"，作为 ADC2 注入通道的外部触发源。
 * @param instance 实例基地址。
 * @param trgo_source 触发源，取 PLAT_TIMER_TRGO_SOURCE_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_select_output_trigger(uintptr_t instance,
                                                    uint16_t trgo_source)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_SelectOutputTrigger((PLAT_TIMER_INSTANCE_TYPE *)(instance), trgo_source);
    return true;
}

/**
 * @brief 使能/禁止计数器（TIM_Cmd）。
 * @param instance 实例基地址。
 * @param enable true 启动计数，false 停止计数。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_Cmd((PLAT_TIMER_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 置指定通道的比较值（TIM_SetCompare1/2/3/4）。
 * @param instance 实例基地址。
 * @param channel 通道号，取 PLAT_TIMER_CHANNEL_x。
 * @param compare_value 比较值 CCR。
 * @retval true 操作成功。
 * @retval false 实例不受支持或通道号越界。
 */
static bool plat_ch32v203_tim_set_compare(uintptr_t instance, uint8_t channel,
                                          uint16_t compare_value)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    switch (channel) {
    case PLAT_TIMER_CHANNEL_1:
        TIM_SetCompare1((PLAT_TIMER_INSTANCE_TYPE *)(instance), compare_value);
        break;
    case PLAT_TIMER_CHANNEL_2:
        TIM_SetCompare2((PLAT_TIMER_INSTANCE_TYPE *)(instance), compare_value);
        break;
    case PLAT_TIMER_CHANNEL_3:
        TIM_SetCompare3((PLAT_TIMER_INSTANCE_TYPE *)(instance), compare_value);
        break;
    case PLAT_TIMER_CHANNEL_4:
        TIM_SetCompare4((PLAT_TIMER_INSTANCE_TYPE *)(instance), compare_value);
        break;
    default:
        return false;
    }

    return true;
}

/**
 * @brief 读当前计数值（TIM_GetCounter）。
 * @param instance 实例基地址。
 * @return 计数值；实例非法时返回 0。
 */
static uint16_t plat_ch32v203_tim_get_counter(uintptr_t instance)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return 0U;
    }

    return TIM_GetCounter((PLAT_TIMER_INSTANCE_TYPE *)(instance));
}

/**
 * @brief 使能/禁止指定的定时器中断源（TIM_ITConfig）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_TIMER_IT_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_it_config(uintptr_t instance, uint16_t it,
                                        bool enable)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_ITConfig((PLAT_TIMER_INSTANCE_TYPE *)(instance), it, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 清中断挂起位（TIM_ClearITPendingBit）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_TIMER_IT_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_clear_it_pending(uintptr_t instance, uint16_t it)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_ClearITPendingBit((PLAT_TIMER_INSTANCE_TYPE *)(instance), it);
    return true;
}

/**
 * @brief 查询中断状态（TIM_GetITStatus）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_TIMER_IT_x。
 * @retval true 中断已产生且未清除。
 * @retval false 未产生中断或实例非法。
 */
static bool plat_ch32v203_tim_get_it_status(uintptr_t instance, uint16_t it)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    return (TIM_GetITStatus((PLAT_TIMER_INSTANCE_TYPE *)(instance), it) != PLAT_RESET);
}

/**
 * @brief 软件产生事件（TIM_GenerateEvent）。
 * @param instance 实例基地址。
 * @param event_source 事件源，取 PLAT_TIMER_EVENT_SOURCE_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_tim_generate_event(uintptr_t instance,
                                            uint16_t event_source)
{
    if (!plat_ch32v203_tim_is_valid(instance)) {
        return false;
    }

    TIM_GenerateEvent((PLAT_TIMER_INSTANCE_TYPE *)(instance), event_source);
    return true;
}

/**
 * @brief 配置并使能定时器中断线。
 * @param irq_channel 中断号，取 PLAT_TIMER_IRQ_x。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_tim_irq_enable(uint32_t irq_channel,
                                         uint8_t preemption_priority,
                                         uint8_t sub_priority)
{
    PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority, PLAT_ENABLE);
    return true;
}

/** @brief CH32V203 定时器操作函数表（本文件私有）。 */
static const platform_ch32v203_tim_ops_t s_tim_ops = {
    .pf_clock_enable = plat_ch32v203_tim_clock_enable,
    .pf_deinit = plat_ch32v203_tim_deinit,
    .pf_time_base_config = plat_ch32v203_tim_time_base_config,
    .pf_oc_channel_config = plat_ch32v203_tim_oc_channel_config,
    .pf_bdtr_config = plat_ch32v203_tim_bdtr_config,
    .pf_oc_preload_config = plat_ch32v203_tim_oc_preload_config,
    .pf_arr_preload_config = plat_ch32v203_tim_arr_preload_config,
    .pf_pwm_outputs_cmd = plat_ch32v203_tim_pwm_outputs_cmd,
    .pf_select_output_trigger = plat_ch32v203_tim_select_output_trigger,
    .pf_cmd = plat_ch32v203_tim_cmd,
    .pf_set_compare = plat_ch32v203_tim_set_compare,
    .pf_get_counter = plat_ch32v203_tim_get_counter,
    .pf_it_config = plat_ch32v203_tim_it_config,
    .pf_clear_it_pending = plat_ch32v203_tim_clear_it_pending,
    .pf_get_it_status = plat_ch32v203_tim_get_it_status,
    .pf_generate_event = plat_ch32v203_tim_generate_event,
    .pf_irq_enable = plat_ch32v203_tim_irq_enable
};

/** @brief CH32V203 定时器操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_tim_ops_t *const plat_ch32v203_tim_ops = &s_tim_ops;
