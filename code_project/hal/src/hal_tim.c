/**
 * @file hal_tim.c
 * @brief 定时器对象操作 + CH32V203 板级实例（TIM1 三相 PWM / TIM2 时基 / TIM3 中断时基）
 *        + HAL 函数表。
 * @details 硬件访问统一经平台定时器操作函数表指针 plat_ch32v203_tim_ops 完成，
 *          引脚组构建经 HAL GPIO 函数表指针 hal_gpio_ops 完成，本文件不含厂商库符号。
 *          板级配置依据参考实现 drv_tim.c：
 *          - TIM1（drv_TIM1_CenterPWM_init()）：PA8/9/10 高端 + PB13/14/15 低端
 *            （交替功能推挽，引脚表复用 hal_gpio_tim_pin_table），中心对齐模式 1、
 *            CKD=DIV1、RCR=1、ARR=11999（6kHz@144MHz）、PSC=0、PWM 模式 2、
 *            主/互补输出均使能且极性为高、空闲电平复位、死区 DTG=50、
 *            OSSR/OSSI 使能、LOCK 级别 1、刹车关闭、自动输出使能，
 *            OCx 预装载与 ARR 预装载使能，MOE 使能，
 *            **TRGO = 更新事件**（作为 ADC2 注入通道的外部触发源），最后启动计数；
 *          - TIM2（drv_TIM2_Count_Init()）：PSC=71、ARR=9999、向上计数、CKD=DIV1、
 *            RCR=0，不开中断，只启动计数；
 *          - TIM3（drv_TIM3_UpdateIT_Init()）：PSC=143、ARR=999、向上计数、CKD=DIV1、
 *            RCR=0；参考实现里开启了更新中断 + NVIC，但该初始化在 Drv_TIM_Init()
 *            中被注释掉，且本工程启动文件的对应 IRQ 处理函数是 `.weak` 死循环，
 *            故本层构建实例时**不使能中断**，仅由 hal_tim_config_update_irq() 按需开启。
 * @note 参考实现未给 TIM_OCInitTypeDef.TIM_Pulse 赋值（局部变量未初始化），
 *       本层显式取 0：上电三相占空比为 0，避免上电瞬间误驱动。
 */

#include "hal_tim.h"
#include "hal_gpio.h"
#include <stddef.h>

/* ==========================================================================
 * 板级配置：定时器参数与通道（参数来源见文件头注释）
 * ========================================================================== */

/**
 * @brief TIM1 三相 PWM 配置。
 * @details TRGO 取"更新事件"：该触发信号是本工程 ADC2 注入通道
 *          （hal_adc_build(..., hal_adc_role_injected) 中配置的
 *          PLAT_ADC_EXTERNAL_TRIG_INJEC_T1_TRGO）的触发源，因此本实例必须在
 *          ADC 注入实例之前构建，才能保证注入采样与 PWM 载波同步。
 */
static const hal_tim_config_t s_tim_motor_config = {
    .time_base = {
        .period = HAL_TIM_PWM_PERIOD,                    /* drv_tim.c: PWM_PeriodValue - 1 */
        .prescaler = HAL_TIM_PWM_PRESCALER,              /* drv_tim.c: MC_PWM_PrescalerValue */
        .clock_division = PLAT_TIMER_CLOCK_DIV_1,        /* drv_tim.c: TIM_CKD_DIV1 */
        .counter_mode = PLAT_TIMER_COUNTER_MODE_CENTER_ALIGNED_1, /* 中心对齐模式 1 */
        .repetition_counter = 1U                         /* drv_tim.c: TIM_RepetitionCounter = 1 */
    },
    .pwm = {
        .oc_mode = PLAT_TIMER_OC_MODE_PWM2,              /* drv_tim.c: TIM_OCMode_PWM2 */
        .output_state = PLAT_TIMER_OUTPUT_STATE_ENABLE,  /* 主输出使能 */
        .output_n_state = PLAT_TIMER_OUTPUT_N_STATE_ENABLE, /* 互补输出使能 */
        .pulse = 0U,                                     /* 参考实现未赋值，此处显式取 0 */
        .oc_polarity = PLAT_TIMER_OC_POLARITY_HIGH,      /* 主输出极性：高 */
        .ocn_polarity = PLAT_TIMER_OCN_POLARITY_HIGH,    /* 互补输出极性：高 */
        .oc_idle_state = PLAT_TIMER_OC_IDLE_STATE_RESET, /* 空闲电平：复位 */
        .ocn_idle_state = PLAT_TIMER_OCN_IDLE_STATE_RESET
    },
    .dead_time = {
        .ossr_state = PLAT_TIMER_OSSR_STATE_ENABLE,      /* drv_tim.c: TIM_OSSRState_Enable */
        .ossi_state = PLAT_TIMER_OSSI_STATE_ENABLE,      /* drv_tim.c: TIM_OSSIState_Enable */
        .lock_level = PLAT_TIMER_LOCK_LEVEL_1,           /* drv_tim.c: TIM_LOCKLevel_1 */
        .dead_time = HAL_TIM_PWM_DEAD_TIME,              /* drv_tim.c: PWM_Dead_Time = 50 */
        .break_state = PLAT_TIMER_BREAK_DISABLE,         /* drv_tim.c: TIM_Break_Disable */
        .break_polarity = PLAT_TIMER_BREAK_POLARITY_LOW, /* drv_tim.c: TIM_BreakPolarity_Low */
        .automatic_output = PLAT_TIMER_AUTOMATIC_OUTPUT_ENABLE /* 自动输出使能 */
    },
    .trgo_source = PLAT_TIMER_TRGO_SOURCE_UPDATE,        /* TRGO = 更新事件（触发 ADC 注入） */
    .channel_count = HAL_TIM_MOTOR_CHANNEL_COUNT,        /* CH1/CH2/CH3 三次 OCxInit */
    .is_pulse_preload_enabled = true,                    /* OCxPreloadConfig(ENABLE) */
    .is_period_preload_enabled = true,                   /* ARRPreloadConfig(ENABLE) */
    .is_complementary_enabled = true,                    /* 高级定时器：配置 BDTR */
    .is_trgo_enabled = true,                             /* TIM_SelectOutputTrigger */
    .is_main_output_enabled = true,                      /* TIM_CtrlPWMOutputs(ENABLE) */
    .irq_channel = 0U,                                   /* 不使用中断 */
    .timeout_count = 0U,
    .p_user_data = NULL
};

/** @brief TIM2 基础时基配置：PSC=71、ARR=9999、向上计数、无中断（drv_TIM2_Count_Init）。 */
static const hal_tim_config_t s_tim_timebase_config = {
    .time_base = {
        .period = HAL_TIM_TIMEBASE_PERIOD,
        .prescaler = HAL_TIM_TIMEBASE_PRESCALER,
        .clock_division = PLAT_TIMER_CLOCK_DIV_1,
        .counter_mode = PLAT_TIMER_COUNTER_MODE_UP,
        .repetition_counter = 0U
    },
    .pwm = {0},
    .dead_time = {0},
    .trgo_source = PLAT_TIMER_TRGO_SOURCE_RESET,
    .channel_count = 0U,
    .is_pulse_preload_enabled = false,
    .is_period_preload_enabled = false,
    .is_complementary_enabled = false,
    .is_trgo_enabled = false,
    .is_main_output_enabled = false,
    .irq_channel = 0U,
    .timeout_count = 0U,
    .p_user_data = NULL
};

/**
 * @brief TIM3 更新中断时基配置：PSC=143、ARR=999、向上计数（drv_TIM3_UpdateIT_Init）。
 * @note 中断源与中断线在本层构建时**不使能**：参考实现的该初始化被注释掉，
 *       且本工程启动文件中 TIM3_IRQHandler 为 `.weak` 死循环，需调用方实现
 *       中断服务函数后再用 hal_tim_config_update_irq() 打开。
 */
static const hal_tim_config_t s_tim_update_irq_config = {
    .time_base = {
        .period = HAL_TIM_UPDATE_IRQ_PERIOD,
        .prescaler = HAL_TIM_UPDATE_IRQ_PRESCALER,
        .clock_division = PLAT_TIMER_CLOCK_DIV_1,
        .counter_mode = PLAT_TIMER_COUNTER_MODE_UP,
        .repetition_counter = 0U
    },
    .pwm = {0},
    .dead_time = {0},
    .trgo_source = PLAT_TIMER_TRGO_SOURCE_RESET,
    .channel_count = 0U,
    .is_pulse_preload_enabled = false,
    .is_period_preload_enabled = false,
    .is_complementary_enabled = false,
    .is_trgo_enabled = false,
    .is_main_output_enabled = false,
    .irq_channel = PLAT_TIMER_IRQ_TIM3, /**< 中断线号，仅 hal_tim_config_update_irq 使用 */
    .timeout_count = 0U,
    .p_user_data = NULL
};

/** @brief TIM1 三相 PWM 板级实例。 */
hal_tim_t hal_tim_motor = {0};
/** @brief TIM2 基础时基板级实例。 */
hal_tim_t hal_tim_timebase = {0};
/** @brief TIM3 更新中断时基板级实例。 */
hal_tim_t hal_tim_update_irq = {0};

/* ==========================================================================
 * 对象操作
 * ========================================================================== */

/**
 * @brief 检查定时器对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（实例基地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_tim_is_object_valid(const hal_tim_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 把 HAL 配置翻译成平台配置并下发到硬件（时基/PWM/死区/预装载/TRGO/主输出）。
 * @param instance 外设实例基地址。
 * @param p_config HAL 定时器配置。
 * @retval true 全部配置下发成功。
 * @retval false 平台层拒绝（实例不受支持、通道号越界等）。
 */
static bool hal_tim_apply_config(uintptr_t instance,
                                 const hal_tim_config_t *p_config)
{
    platform_ch32v203_tim_time_base_config_t time_base;
    platform_ch32v203_tim_oc_config_t oc;
    platform_ch32v203_tim_bdtr_config_t bdtr;
    uint8_t channel;

    /* 时基：ARR / PSC / CKD / 计数模式 / RCR */
    time_base.period = p_config->time_base.period;
    time_base.prescaler = p_config->time_base.prescaler;
    time_base.clock_division = p_config->time_base.clock_division;
    time_base.counter_mode = p_config->time_base.counter_mode;
    time_base.repetition_counter = p_config->time_base.repetition_counter;

    if (!plat_ch32v203_tim_ops->pf_time_base_config(instance, &time_base)) {
        return false;
    }

    /* PWM 输出通道：参考实现用同一份 OC 结构体连续配置 CH1/CH2/CH3 */
    if (p_config->channel_count > 0U) {
        oc.oc_mode = p_config->pwm.oc_mode;
        oc.output_state = p_config->pwm.output_state;
        oc.output_n_state = p_config->pwm.output_n_state;
        oc.pulse = p_config->pwm.pulse;
        oc.oc_polarity = p_config->pwm.oc_polarity;
        oc.ocn_polarity = p_config->pwm.ocn_polarity;
        oc.oc_idle_state = p_config->pwm.oc_idle_state;
        oc.ocn_idle_state = p_config->pwm.ocn_idle_state;

        for (channel = PLAT_TIMER_CHANNEL_1;
             channel <= p_config->channel_count; channel++) {
            if (!plat_ch32v203_tim_ops->pf_oc_channel_config(instance, channel,
                                                             &oc)) {
                return false;
            }
        }
    }

    /* 死区与刹车（仅高级定时器 TIM1） */
    if (p_config->is_complementary_enabled) {
        bdtr.ossr_state = p_config->dead_time.ossr_state;
        bdtr.ossi_state = p_config->dead_time.ossi_state;
        bdtr.lock_level = p_config->dead_time.lock_level;
        bdtr.dead_time = p_config->dead_time.dead_time;
        bdtr.break_state = p_config->dead_time.break_state;
        bdtr.break_polarity = p_config->dead_time.break_polarity;
        bdtr.automatic_output = p_config->dead_time.automatic_output;

        if (!plat_ch32v203_tim_ops->pf_bdtr_config(instance, &bdtr)) {
            return false;
        }
    }

    /* 比较寄存器预装载：新比较值在下一个更新事件生效 */
    if ((p_config->is_pulse_preload_enabled) &&
        (p_config->channel_count > 0U)) {
        for (channel = PLAT_TIMER_CHANNEL_1;
             channel <= p_config->channel_count; channel++) {
            if (!plat_ch32v203_tim_ops->pf_oc_preload_config(instance, channel,
                                                             true)) {
                return false;
            }
        }
    }

    /* 自动重装载预装载 */
    if ((p_config->is_period_preload_enabled) &&
        (!plat_ch32v203_tim_ops->pf_arr_preload_config(instance, true))) {
        return false;
    }

    /* 主输出触发源：TIM1 取更新事件，供 ADC 注入通道同步采样 */
    if ((p_config->is_trgo_enabled) &&
        (!plat_ch32v203_tim_ops->pf_select_output_trigger(
            instance, p_config->trgo_source))) {
        return false;
    }

    /* 主输出 MOE：使能后六路 PWM 引脚才真正输出 */
    if ((p_config->is_main_output_enabled) &&
        (!plat_ch32v203_tim_ops->pf_pwm_outputs_cmd(instance, true))) {
        return false;
    }

    return true;
}

/**
 * @brief 绑定定时器对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（PLAT_PERIPH_TIMx）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效或通道数越界。
 */
hal_tim_status_enum hal_tim_bind(hal_tim_t *p_object,
                                 uintptr_t base_address,
                                 const hal_tim_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) || (base_address == 0U) ||
        (p_config->channel_count > PLAT_TIMER_CHANNEL_MAX)) {
        return hal_tim_err_param;
    }

    p_object->state = hal_tim_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;

    return hal_tim_ok;
}

/**
 * @brief 初始化定时器：时钟 + 复位 + 配置下发 + 启动计数。
 * @details 顺序与参考实现一致：TRGO 在启动计数器之前设置完成，保证 ADC 注入通道
 *          使能外部触发后即可被更新事件触发。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定、实例不受支持或配置下发失败。
 */
hal_tim_status_enum hal_tim_init(hal_tim_t *p_object)
{
    uintptr_t instance;

    if (!hal_tim_is_object_valid(p_object)) {
        return hal_tim_err_param;
    }

    instance = p_object->base_address;

    if ((!plat_ch32v203_tim_ops->pf_clock_enable(instance)) ||
        (!plat_ch32v203_tim_ops->pf_deinit(instance)) ||
        (!hal_tim_apply_config(instance, &p_object->config)) ||
        (!plat_ch32v203_tim_ops->pf_cmd(instance, true))) {
        p_object->state = hal_tim_state_error;
        return hal_tim_err_param;
    }

    p_object->state = hal_tim_state_enabled;
    return hal_tim_ok;
}

/**
 * @brief 关闭并复位定时器外设：先停计数器与主输出，再复位外设。
 * @note 只有配置中声明使用主输出（is_main_output_enabled，即三相 PWM 实例）才会
 *       关闭 MOE：死区/刹车寄存器只存在于高级定时器，纯时基实例不得触碰。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或实例不受支持。
 */
hal_tim_status_enum hal_tim_deinit(hal_tim_t *p_object)
{
    uintptr_t instance;

    if (!hal_tim_is_object_valid(p_object)) {
        return hal_tim_err_param;
    }

    instance = p_object->base_address;

    if (p_object->config.is_main_output_enabled) {
        if (!plat_ch32v203_tim_ops->pf_pwm_outputs_cmd(instance, false)) {
            return hal_tim_err_param;
        }
    }

    if ((!plat_ch32v203_tim_ops->pf_cmd(instance, false)) ||
        (!plat_ch32v203_tim_ops->pf_deinit(instance))) {
        return hal_tim_err_param;
    }

    p_object->state = hal_tim_state_reset;
    return hal_tim_ok;
}

/**
 * @brief 启动定时器计数。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或实例不受支持。
 */
hal_tim_status_enum hal_tim_enable(hal_tim_t *p_object)
{
    if (!hal_tim_is_object_valid(p_object)) {
        return hal_tim_err_param;
    }

    if (!plat_ch32v203_tim_ops->pf_cmd(p_object->base_address, true)) {
        return hal_tim_err_param;
    }

    p_object->state = hal_tim_state_enabled;
    return hal_tim_ok;
}

/**
 * @brief 停止定时器计数。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或实例不受支持。
 */
hal_tim_status_enum hal_tim_disable(hal_tim_t *p_object)
{
    if (!hal_tim_is_object_valid(p_object)) {
        return hal_tim_err_param;
    }

    if (!plat_ch32v203_tim_ops->pf_cmd(p_object->base_address, false)) {
        return hal_tim_err_param;
    }

    p_object->state = hal_tim_state_disabled;
    return hal_tim_ok;
}

/**
 * @brief 使能/禁止三相 PWM 主输出 MOE。
 * @note 只有三相 PWM 实例（配置 is_main_output_enabled == true，对应高级定时器）
 *       支持该接口，纯时基实例返回 hal_tim_err_unsupported。
 * @param p_object 模块对象指针。
 * @param enable true 使能主输出，false 关闭主输出（引脚回到空闲电平）。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定。
 * @retval hal_tim_err_unsupported 该实例未使用主输出。
 * @retval hal_tim_err_state 平台层拒绝。
 */
hal_tim_status_enum hal_tim_output_enable(hal_tim_t *p_object, bool enable)
{
    if (!hal_tim_is_object_valid(p_object)) {
        return hal_tim_err_param;
    }

    if (!p_object->config.is_main_output_enabled) {
        return hal_tim_err_unsupported;
    }

    if (!plat_ch32v203_tim_ops->pf_pwm_outputs_cmd(p_object->base_address,
                                                  enable)) {
        return hal_tim_err_state;
    }

    return hal_tim_ok;
}

/**
 * @brief 设置指定通道的比较值（占空比）。
 * @param p_object 模块对象指针。
 * @param channel 通道号，取 PLAT_TIMER_CHANNEL_x（1 起始）。
 * @param compare_value 比较值。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或通道号越界。
 * @retval hal_tim_err_state 平台层拒绝。
 */
hal_tim_status_enum hal_tim_set_compare(hal_tim_t *p_object,
                                       uint8_t channel,
                                       uint16_t compare_value)
{
    if ((!hal_tim_is_object_valid(p_object)) || (channel == 0U) ||
        (channel > PLAT_TIMER_CHANNEL_MAX) ||
        (channel > p_object->config.channel_count)) {
        return hal_tim_err_param;
    }

    if (!plat_ch32v203_tim_ops->pf_set_compare(p_object->base_address, channel,
                                              compare_value)) {
        return hal_tim_err_state;
    }

    return hal_tim_ok;
}

/**
 * @brief 读取定时器当前计数值。
 * @param p_object 模块对象指针。
 * @param p_counter 输出计数值指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 */
hal_tim_status_enum hal_tim_get_counter(hal_tim_t *p_object,
                                       uint16_t *p_counter)
{
    if ((!hal_tim_is_object_valid(p_object)) || (p_counter == NULL)) {
        return hal_tim_err_param;
    }

    *p_counter = plat_ch32v203_tim_ops->pf_get_counter(p_object->base_address);
    return hal_tim_ok;
}

/**
 * @brief 配置更新中断（可选接口，默认不使能）。
 * @warning 使能前必须已实现对应的定时器更新中断服务函数：本工程启动文件对 IRQ
 *          只提供 `.weak` 默认处理函数（内容为死循环），未实现服务函数就打开
 *          中断会直接跑飞。
 * @param p_object 模块对象指针。
 * @param enable true 使能更新中断，false 关闭更新中断。
 * @param preemption_priority 抢占优先级（仅使能时使用）。
 * @param sub_priority 子优先级（仅使能时使用）。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定。
 * @retval hal_tim_err_unsupported 对象未配置中断线。
 * @retval hal_tim_err_state 平台层拒绝。
 */
hal_tim_status_enum hal_tim_config_update_irq(hal_tim_t *p_object,
                                             bool enable,
                                             uint8_t preemption_priority,
                                             uint8_t sub_priority)
{
    uintptr_t instance;

    if (!hal_tim_is_object_valid(p_object)) {
        return hal_tim_err_param;
    }

    instance = p_object->base_address;

    if (!enable) {
        if (!plat_ch32v203_tim_ops->pf_it_config(instance,
                                                 PLAT_TIMER_IT_UPDATE, false)) {
            return hal_tim_err_state;
        }
        return hal_tim_ok;
    }

    if (p_object->config.irq_channel == 0U) {
        return hal_tim_err_unsupported;
    }

    if ((!plat_ch32v203_tim_ops->pf_clear_it_pending(instance,
                                                    PLAT_TIMER_IT_UPDATE)) ||
        (!plat_ch32v203_tim_ops->pf_it_config(instance, PLAT_TIMER_IT_UPDATE,
                                              true)) ||
        (!plat_ch32v203_tim_ops->pf_irq_enable(p_object->config.irq_channel,
                                               preemption_priority,
                                               sub_priority))) {
        return hal_tim_err_state;
    }

    return hal_tim_ok;
}

/**
 * @brief 返回定时器对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_tim_state_enum hal_tim_get_state(const hal_tim_t *p_object)
{
    if (p_object == NULL) {
        return hal_tim_state_error;
    }
    return p_object->state;
}

/* ==========================================================================
 * 板级构建
 * ========================================================================== */

/**
 * @brief 构建 TIM1 三相 PWM 实例（引脚组 + 配置 + 启动）。
 * @details 先经 hal_gpio_ops->pf_build() 构建 hal_gpio_tim 引脚组
 *          （PA8/9/10 高端 + PB13/14/15 低端，表项顺序与 hal_gpio_tim_pin_enum 一致），
 *          再绑定并初始化 TIM1。TRGO 在本步骤中被设置为更新事件，因此本函数必须在
 *          hal_adc_build(&hal_adc_injected, hal_adc_role_injected) 之前调用。
 * @param p_object 定时器对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 * @retval hal_tim_err_state 引脚组构建或平台层配置失败。
 */
static hal_tim_status_enum hal_tim_build_motor_pwm(hal_tim_t *p_object)
{
    hal_tim_status_enum status;

    /* 板级前置：电机 PWM 引脚组（复用 hal_gpio_tim，经 HAL GPIO 函数表构建） */
    if (hal_gpio_ops->pf_build(&hal_gpio_tim, hal_gpio_tim_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_tim_pin_table))
        != hal_gpio_ok) {
        return hal_tim_err_state;
    }

    status = hal_tim_bind(p_object, (uintptr_t)PLAT_PERIPH_TIM1,
                          &s_tim_motor_config);
    if (status != hal_tim_ok) {
        return status;
    }

    return hal_tim_init(p_object);
}

/**
 * @brief 构建 TIM2 基础时基实例（配置 + 启动，无中断）。
 * @param p_object 定时器对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 * @retval hal_tim_err_state 平台层配置失败。
 */
static hal_tim_status_enum hal_tim_build_timebase(hal_tim_t *p_object)
{
    hal_tim_status_enum status;

    status = hal_tim_bind(p_object, (uintptr_t)PLAT_PERIPH_TIM2,
                          &s_tim_timebase_config);
    if (status != hal_tim_ok) {
        return status;
    }

    return hal_tim_init(p_object);
}

/**
 * @brief 构建 TIM3 更新中断时基实例（只配置时基并启动，不使能中断）。
 * @note 更新中断与中断线保持关闭，需调用方实现中断服务函数后自行调用
 *       hal_tim_config_update_irq()。
 * @param p_object 定时器对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 * @retval hal_tim_err_state 平台层配置失败。
 */
static hal_tim_status_enum hal_tim_build_update_irq(hal_tim_t *p_object)
{
    hal_tim_status_enum status;

    status = hal_tim_bind(p_object, (uintptr_t)PLAT_PERIPH_TIM3,
                          &s_tim_update_irq_config);
    if (status != hal_tim_ok) {
        return status;
    }

    // 需补上中断线配置+使能，后续需要对hal_tim_config_update_irq函数的中断源使能与中断线配置进行分离

    return hal_tim_init(p_object);
}

/**
 * @brief 构建板级定时器实例。
 * @param p_object 定时器对象指针。
 * @param role 板级用途，取 hal_tim_role_x。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效或用途不支持。
 * @retval hal_tim_err_state 引脚组构建或平台层配置失败。
 */
hal_tim_status_enum hal_tim_build(hal_tim_t *p_object, hal_tim_role_enum role)
{
    if (p_object == NULL) {
        return hal_tim_err_param;
    }

    if (role == hal_tim_role_motor_pwm) {
        return hal_tim_build_motor_pwm(p_object);
    }
    if (role == hal_tim_role_timebase) {
        return hal_tim_build_timebase(p_object);
    }
    if (role == hal_tim_role_update_irq) {
        return hal_tim_build_update_irq(p_object);
    }

    return hal_tim_err_param;
}

/* ==========================================================================
 * HAL 层定时器函数表
 * ========================================================================== */

/** @brief HAL 层定时器函数表（本文件私有，绑定本层对外接口）。 */
static const hal_tim_ops_t s_hal_tim_ops = {
    .pf_build = hal_tim_build,
    .pf_set_compare = hal_tim_set_compare,
    .pf_get_counter = hal_tim_get_counter,
    /* 备用接口：与同名 HAL 函数一一对应，供外部按需接入 */
    .pf_get_state = hal_tim_get_state,    
    .pf_deinit = hal_tim_deinit,
    .pf_enable = hal_tim_enable,
    .pf_disable = hal_tim_disable,
    .pf_output_enable = hal_tim_output_enable
};

/** @brief HAL 层定时器函数表指针（对外外露，声明于 hal_tim.h）。 */
const hal_tim_ops_t *const hal_tim_ops = &s_hal_tim_ops;
