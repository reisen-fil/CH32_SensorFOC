/**
 * @file platform_ch32v203_tim.h
 * @brief CH32V203 定时器（含三相 PWM）的 MCU 抽象接口（取值宏 + 操作函数表）。
 * @details 与 GPIO/ADC/SPI 平台层同构：
 *          - 取值宏：计数模式 / 时钟分频 / 输出比较(PWM)模式 / 输出与互补输出使能 /
 *            输出与互补输出极性 / 空闲电平 / 预装载 / 死区与刹车(BDTR) / 通道号 /
 *            中断源 / 更新事件源 / 主输出触发（TRGO）源 / 中断线；
 *          - 操作函数表 platform_ch32v203_tim_ops_t：把标准库 TIM 接口
 *            （时钟使能、TIM_DeInit、TIM_TimeBaseInit、TIM_OC1..4Init、
 *             TIM_BDTRConfig、TIM_OC1..4PreloadConfig、TIM_ARRPreloadConfig、
 *             TIM_CtrlPWMOutputs、TIM_SelectOutputTrigger、TIM_Cmd、
 *             TIM_SetCompare1..4、TIM_GetCounter、TIM_ITConfig、
 *             TIM_ClearITPendingBit、TIM_GetITStatus、TIM_GenerateEvent、
 *             中断线配置）逐个做简单函数封装后绑定，并通过 plat_ch32v203_tim_ops
 *             指针对外外露。
 *          定时器的对象模型、板级实例（TIM1 三相 PWM / TIM2 基础时基 /
 *          TIM3 更新中断时基）与 PWM 流程实现于 hal_tim.c/h。
 * @note 本层不提供任何业务语义（电机换相、电流环、速度环等），只做 MCU 抽象。
 * @note 本头文件只使用 <stdint.h>/<stdbool.h> 的纯 C 类型：厂商类型
 *       （TIM_TypeDef、TIM_TimeBaseInitTypeDef、TIM_OCInitTypeDef、
 *       TIM_BDTRInitTypeDef 等）与厂商函数一律只允许出现在下方"类型接入宏"的
 *       宏体（PLAT_TIMER_*_TYPE）或 platform_ch32v203_tim.c 内：.c 里只书写
 *       PLAT_* 名字，厂商名字只收敛在本头的宏体里。原因是本头文件经 hal_tim.h
 *       被 hal.h 引用，可能在 ch32v20x_conf.h 之前完成解析，此时厂商类型尚不可见，
 *       写出来会报 unknown type name；而宏体只在展开处求值，是安全的。
 */
#ifndef __PLATFORM_CH32V203_TIM_H__
#define __PLATFORM_CH32V203_TIM_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/** @brief 计数模式（TIM_CounterMode_x）。 */
#define PLAT_TIMER_COUNTER_MODE_UP                TIM_CounterMode_Up
#define PLAT_TIMER_COUNTER_MODE_DOWN              TIM_CounterMode_Down
#define PLAT_TIMER_COUNTER_MODE_CENTER_ALIGNED_1  TIM_CounterMode_CenterAligned1
#define PLAT_TIMER_COUNTER_MODE_CENTER_ALIGNED_2  TIM_CounterMode_CenterAligned2
#define PLAT_TIMER_COUNTER_MODE_CENTER_ALIGNED_3  TIM_CounterMode_CenterAligned3

/** @brief 时钟分频（TIM_CKD_x），决定死区时间的时基 t_DTS。 */
#define PLAT_TIMER_CLOCK_DIV_1                    TIM_CKD_DIV1
#define PLAT_TIMER_CLOCK_DIV_2                    TIM_CKD_DIV2
#define PLAT_TIMER_CLOCK_DIV_4                    TIM_CKD_DIV4

/** @brief 输出比较模式（TIM_OCMode_x）。 */
#define PLAT_TIMER_OC_MODE_TIMING                 TIM_OCMode_Timing
#define PLAT_TIMER_OC_MODE_ACTIVE                 TIM_OCMode_Active
#define PLAT_TIMER_OC_MODE_INACTIVE               TIM_OCMode_Inactive
#define PLAT_TIMER_OC_MODE_TOGGLE                 TIM_OCMode_Toggle
#define PLAT_TIMER_OC_MODE_PWM1                   TIM_OCMode_PWM1
#define PLAT_TIMER_OC_MODE_PWM2                   TIM_OCMode_PWM2

/** @brief 主输出（OCx）使能。 */
#define PLAT_TIMER_OUTPUT_STATE_DISABLE           TIM_OutputState_Disable
#define PLAT_TIMER_OUTPUT_STATE_ENABLE            TIM_OutputState_Enable

/** @brief 互补输出（OCxN）使能，仅 TIM1（高级定时器）有效。 */
#define PLAT_TIMER_OUTPUT_N_STATE_DISABLE         TIM_OutputNState_Disable
#define PLAT_TIMER_OUTPUT_N_STATE_ENABLE          TIM_OutputNState_Enable

/** @brief 主输出极性。 */
#define PLAT_TIMER_OC_POLARITY_HIGH               TIM_OCPolarity_High
#define PLAT_TIMER_OC_POLARITY_LOW                TIM_OCPolarity_Low

/** @brief 互补输出极性，仅 TIM1（高级定时器）有效。 */
#define PLAT_TIMER_OCN_POLARITY_HIGH              TIM_OCNPolarity_High
#define PLAT_TIMER_OCN_POLARITY_LOW               TIM_OCNPolarity_Low

/** @brief 空闲（IDLE）状态下主输出电平，仅 TIM1（高级定时器）有效。 */
#define PLAT_TIMER_OC_IDLE_STATE_SET              TIM_OCIdleState_Set
#define PLAT_TIMER_OC_IDLE_STATE_RESET            TIM_OCIdleState_Reset

/** @brief 空闲（IDLE）状态下互补输出电平，仅 TIM1（高级定时器）有效。 */
#define PLAT_TIMER_OCN_IDLE_STATE_SET             TIM_OCNIdleState_Set
#define PLAT_TIMER_OCN_IDLE_STATE_RESET           TIM_OCNIdleState_Reset

/** @brief 运行模式下的关闭状态选择（BDTR->OSSR），仅 TIM1 有效。 */
#define PLAT_TIMER_OSSR_STATE_DISABLE             TIM_OSSRState_Disable
#define PLAT_TIMER_OSSR_STATE_ENABLE              TIM_OSSRState_Enable

/** @brief 空闲模式下的关闭状态选择（BDTR->OSSI），仅 TIM1 有效。 */
#define PLAT_TIMER_OSSI_STATE_DISABLE             TIM_OSSIState_Disable
#define PLAT_TIMER_OSSI_STATE_ENABLE              TIM_OSSIState_Enable

/** @brief 寄存器锁级别（BDTR->LOCK），仅 TIM1 有效。 */
#define PLAT_TIMER_LOCK_LEVEL_OFF                 TIM_LOCKLevel_OFF
#define PLAT_TIMER_LOCK_LEVEL_1                   TIM_LOCKLevel_1
#define PLAT_TIMER_LOCK_LEVEL_2                   TIM_LOCKLevel_2
#define PLAT_TIMER_LOCK_LEVEL_3                   TIM_LOCKLevel_3

/** @brief 刹车（BREAK）输入使能，仅 TIM1 有效。本工程不使用刹车输入。 */
#define PLAT_TIMER_BREAK_DISABLE                  TIM_Break_Disable
#define PLAT_TIMER_BREAK_ENABLE                   TIM_Break_Enable

/** @brief 刹车输入极性，仅 TIM1 有效。 */
#define PLAT_TIMER_BREAK_POLARITY_LOW             TIM_BreakPolarity_Low
#define PLAT_TIMER_BREAK_POLARITY_HIGH            TIM_BreakPolarity_High

/** @brief 自动输出（AOE）使能，仅 TIM1 有效。 */
#define PLAT_TIMER_AUTOMATIC_OUTPUT_DISABLE       TIM_AutomaticOutput_Disable
#define PLAT_TIMER_AUTOMATIC_OUTPUT_ENABLE        TIM_AutomaticOutput_Enable

/** @brief 比较寄存器预装载（TIM_OCPreload_x）。 */
#define PLAT_TIMER_OC_PRELOAD_DISABLE             TIM_OCPreload_Disable
#define PLAT_TIMER_OC_PRELOAD_ENABLE              TIM_OCPreload_Enable

/** @brief 主输出触发（TRGO）源（TIM_TRGOSource_x）。 */
#define PLAT_TIMER_TRGO_SOURCE_RESET              TIM_TRGOSource_Reset
#define PLAT_TIMER_TRGO_SOURCE_ENABLE             TIM_TRGOSource_Enable
#define PLAT_TIMER_TRGO_SOURCE_UPDATE             TIM_TRGOSource_Update
#define PLAT_TIMER_TRGO_SOURCE_OC1                TIM_TRGOSource_OC1
#define PLAT_TIMER_TRGO_SOURCE_OC1REF             TIM_TRGOSource_OC1Ref
#define PLAT_TIMER_TRGO_SOURCE_OC2REF             TIM_TRGOSource_OC2Ref
#define PLAT_TIMER_TRGO_SOURCE_OC3REF             TIM_TRGOSource_OC3Ref
#define PLAT_TIMER_TRGO_SOURCE_OC4REF             TIM_TRGOSource_OC4Ref

/** @brief 软件事件源（TIM_GenerateEvent 的入参，TIM_EventSource_x）。 */
#define PLAT_TIMER_EVENT_SOURCE_UPDATE            TIM_EventSource_Update

/** @brief 定时器通道号（本层自定义的 1 起始编号，内部再翻译成 TIM_Channel_x）。 */
#define PLAT_TIMER_CHANNEL_1                      (1U)
#define PLAT_TIMER_CHANNEL_2                      (2U)
#define PLAT_TIMER_CHANNEL_3                      (3U)
#define PLAT_TIMER_CHANNEL_4                      (4U)
#define PLAT_TIMER_CHANNEL_MAX                    (4U)

/** @brief 定时器中断源（TIM_IT_x）。 */
#define PLAT_TIMER_IT_UPDATE                      TIM_IT_Update
#define PLAT_TIMER_IT_CC1                         TIM_IT_CC1
#define PLAT_TIMER_IT_CC2                         TIM_IT_CC2
#define PLAT_TIMER_IT_CC3                         TIM_IT_CC3
#define PLAT_TIMER_IT_CC4                         TIM_IT_CC4
#define PLAT_TIMER_IT_COM                         TIM_IT_COM
#define PLAT_TIMER_IT_TRIGGER                     TIM_IT_Trigger
#define PLAT_TIMER_IT_BREAK                       TIM_IT_Break

/** @brief 定时器状态标志（TIM_FLAG_x）。 */
#define PLAT_TIMER_FLAG_UPDATE                    TIM_FLAG_Update
#define PLAT_TIMER_FLAG_CC1                       TIM_FLAG_CC1
#define PLAT_TIMER_FLAG_CC2                       TIM_FLAG_CC2
#define PLAT_TIMER_FLAG_CC3                       TIM_FLAG_CC3
#define PLAT_TIMER_FLAG_CC4                       TIM_FLAG_CC4

/** @brief 定时器中断线（供 pf_irq_enable 使用）。 */
#define PLAT_TIMER_IRQ_TIM1_UPDATE                PLAT_IRQ_TIM1_UPDATE
#define PLAT_TIMER_IRQ_TIM1_CAPTURE_COMPARE       PLAT_IRQ_TIM1_CAPTURE_COMPARE
#define PLAT_TIMER_IRQ_TIM2                       PLAT_IRQ_TIM2
#define PLAT_TIMER_IRQ_TIM3                       PLAT_IRQ_TIM3
#define PLAT_TIMER_IRQ_TIM4                       PLAT_IRQ_TIM4

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_tim.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_TIMER_INSTANCE_TYPE                  TIM_TypeDef
#define PLAT_TIMER_TIME_BASE_INIT_TYPE            TIM_TimeBaseInitTypeDef
#define PLAT_TIMER_OC_INIT_TYPE                   TIM_OCInitTypeDef
#define PLAT_TIMER_BDTR_INIT_TYPE                 TIM_BDTRInitTypeDef

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 定时器时基配置（对应标准库 TIM_TimeBaseInitTypeDef 的可移植子集）。 */
typedef struct {
    uint16_t period;              /**< 自动重装载值 ARR，中心对齐 PWM 时为半个载波周期的计数 */
    uint16_t prescaler;           /**< 预分频值 PSC（写入寄存器，非分频系数） */
    uint16_t clock_division;      /**< PLAT_TIMER_CLOCK_DIV_x，影响死区时间时基 t_DTS */
    uint16_t counter_mode;        /**< PLAT_TIMER_COUNTER_MODE_x */
    uint8_t repetition_counter;   /**< 重复计数 RCR，仅 TIM1 有效 */
} platform_ch32v203_tim_time_base_config_t;

/** @brief 定时器输出比较（PWM）通道配置（对应 TIM_OCInitTypeDef 的可移植子集）。 */
typedef struct {
    uint16_t oc_mode;             /**< PLAT_TIMER_OC_MODE_x */
    uint16_t output_state;        /**< PLAT_TIMER_OUTPUT_STATE_x */
    uint16_t output_n_state;      /**< PLAT_TIMER_OUTPUT_N_STATE_x，仅 TIM1 有效 */
    uint16_t pulse;               /**< 比较值 CCR，决定初始占空比 */
    uint16_t oc_polarity;         /**< PLAT_TIMER_OC_POLARITY_x */
    uint16_t ocn_polarity;        /**< PLAT_TIMER_OCN_POLARITY_x，仅 TIM1 有效 */
    uint16_t oc_idle_state;       /**< PLAT_TIMER_OC_IDLE_STATE_x，仅 TIM1 有效 */
    uint16_t ocn_idle_state;      /**< PLAT_TIMER_OCN_IDLE_STATE_x，仅 TIM1 有效 */
} platform_ch32v203_tim_oc_config_t;

/** @brief 死区与刹车配置（对应 TIM_BDTRInitTypeDef 的可移植子集，仅 TIM1 有效）。 */
typedef struct {
    uint16_t ossr_state;          /**< PLAT_TIMER_OSSR_STATE_x */
    uint16_t ossi_state;          /**< PLAT_TIMER_OSSI_STATE_x */
    uint16_t lock_level;          /**< PLAT_TIMER_LOCK_LEVEL_x */
    uint8_t dead_time;            /**< 死区发生器 DTG[7:0]，0..255 */
    uint16_t break_state;         /**< PLAT_TIMER_BREAK_x */
    uint16_t break_polarity;      /**< PLAT_TIMER_BREAK_POLARITY_x */
    uint16_t automatic_output;    /**< PLAT_TIMER_AUTOMATIC_OUTPUT_x */
} platform_ch32v203_tim_bdtr_config_t;

/**
 * @brief CH32V203 定时器操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 返回 bool，true 表示成功；取值类函数直接返回寄存器值。
 *          通道号统一使用本层自定义的 PLAT_TIMER_CHANNEL_1..4（1 起始），
 *          由本层翻译成标准库的 TIM_Channel_x，调用方无需关心厂商编码。
 */
typedef struct {
    /** @brief 使能定时器实例时钟（TIM1 走 APB2，TIM2..TIM5 走 APB1）。 */
    bool (*pf_clock_enable)(uintptr_t instance);
    /** @brief 复位定时器外设（TIM_DeInit）。 */
    bool (*pf_deinit)(uintptr_t instance);
    /** @brief 配置时基（TIM_TimeBaseInit）。 */
    bool (*pf_time_base_config)(
        uintptr_t instance,
        const platform_ch32v203_tim_time_base_config_t *p_config);
    /** @brief 配置指定通道的 PWM 输出（TIM_OC1/2/3/4Init）。 */
    bool (*pf_oc_channel_config)(uintptr_t instance, uint8_t channel,
                                 const platform_ch32v203_tim_oc_config_t *p_config);
    /** @brief 配置死区与刹车（TIM_BDTRConfig；仅高级定时器 TIM1，其它实例返回 false）。 */
    bool (*pf_bdtr_config)(uintptr_t instance,
                           const platform_ch32v203_tim_bdtr_config_t *p_config);
    /** @brief 使能/禁止指定通道的比较寄存器预装载（TIM_OCxPreloadConfig）。 */
    bool (*pf_oc_preload_config)(uintptr_t instance, uint8_t channel, bool enable);
    /** @brief 使能/禁止自动重装载寄存器预装载（TIM_ARRPreloadConfig）。 */
    bool (*pf_arr_preload_config)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止主输出 MOE（TIM_CtrlPWMOutputs；仅 TIM1，其它实例返回 false）。 */
    bool (*pf_pwm_outputs_cmd)(uintptr_t instance, bool enable);
    /** @brief 选择主输出触发（TRGO）源（TIM_SelectOutputTrigger）。 */
    bool (*pf_select_output_trigger)(uintptr_t instance, uint16_t trgo_source);
    /** @brief 使能/禁止计数器（TIM_Cmd）。 */
    bool (*pf_cmd)(uintptr_t instance, bool enable);
    /** @brief 置指定通道的比较值（TIM_SetCompare1/2/3/4）。 */
    bool (*pf_set_compare)(uintptr_t instance, uint8_t channel,
                           uint16_t compare_value);
    /** @brief 读当前计数值（TIM_GetCounter）。 */
    uint16_t (*pf_get_counter)(uintptr_t instance);
    /** @brief 使能/禁止指定的定时器中断源（TIM_ITConfig）。 */
    bool (*pf_it_config)(uintptr_t instance, uint16_t it, bool enable);
    /** @brief 清中断挂起位（TIM_ClearITPendingBit）。 */
    bool (*pf_clear_it_pending)(uintptr_t instance, uint16_t it);
    /** @brief 查询中断状态（TIM_GetITStatus）。 */
    bool (*pf_get_it_status)(uintptr_t instance, uint16_t it);
    /** @brief 软件产生事件（TIM_GenerateEvent），如 PLAT_TIMER_EVENT_SOURCE_UPDATE。 */
    bool (*pf_generate_event)(uintptr_t instance, uint16_t event_source);
    /** @brief 配置并使能定时器中断线（PLAT_PFIC_INIT）。 */
    bool (*pf_irq_enable)(uint32_t irq_channel, uint8_t preemption_priority,
                          uint8_t sub_priority);
} platform_ch32v203_tim_ops_t;

/** @brief CH32V203 定时器操作函数表指针（在 platform_ch32v203_tim.c 中组装）。 */
extern const platform_ch32v203_tim_ops_t *const plat_ch32v203_tim_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_TIM_H__ */
