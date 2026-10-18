/**
 * @file hal_tim.h
 * @brief 定时器/三相 PWM 硬件抽象接口（含 CH32V203 板级实例与 HAL 层函数表）。
 * @details 与 GPIO/ADC 同构：对象操作函数同层实现，硬件访问统一经平台定时器操作
 *          函数表指针 plat_ch32v203_tim_ops 完成，引脚组构建经 HAL GPIO 函数表指针
 *          hal_gpio_ops 完成，本层不含厂商库符号。
 *          板级实例按"用途"区分（参数取自参考实现 drv_tim.c）：
 *          - hal_tim_motor：TIM1 三相中心对齐互补 PWM + 死区，PA8/9/10 高端与
 *            PB13/14/15 低端，对应 drv_TIM1_CenterPWM_init()；其 TRGO 取"更新事件"，
 *            是本工程 ADC2 注入通道（hal_adc_injected）的外部触发源；
 *          - hal_tim_timebase：TIM2 向上计数基础时基（不开中断），对应
 *            drv_TIM2_Count_Init()；
 *          - hal_tim_update_irq：TIM3 向上计数 + 更新中断时基，对应
 *            drv_TIM3_UpdateIT_Init()。注意参考实现的 Drv_TIM_Init() 里 TIM3 初始化
 *            是被注释掉的，且本工程启动文件对所有 IRQ 只提供 `.weak` 默认处理函数
 *            （内容为 `1: j 1b` 死循环）：**构建该实例时不会使能更新中断与中断线**，
 *            必须由调用方在实现好定时器更新中断服务函数后，再显式调用
 *            hal_tim_config_update_irq() 打开，否则一进中断即跑飞。
 * @note 本层不引入组配置/多视图机制：一个 hal_tim_t 直接对应一个定时器实例。
 * @note 本层文件不出现任何厂商符号，配置字段取值全部来自 MCU 抽象接口
 *       platform_ch32v203_tim.h 的 PLAT_TIMER_* 宏。
 */
#ifndef __HAL_TIM_H__
#define __HAL_TIM_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_TIMER_* 模式/极性/通道/中断/TRGO 取值宏 */
#include "platform_ch32v203_tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 定时器对象前向声明。 */
typedef struct hal_tim hal_tim_t;

/** @brief 定时器对象的生命周期状态。 */
typedef enum {
    hal_tim_state_reset = 0,
    hal_tim_state_ready,
    hal_tim_state_enabled,
    hal_tim_state_disabled,
    hal_tim_state_error
} hal_tim_state_enum;

/** @brief 定时器接口调用返回的状态码。 */
typedef enum {
    hal_tim_ok = 0,
    hal_tim_err_param = -1,
    hal_tim_err_state = -2,
    hal_tim_err_unsupported = -3,
    hal_tim_err_timeout = -4
} hal_tim_status_enum;

/** @brief 定时器板级用途。 */
typedef enum {
    hal_tim_role_motor_pwm = 0, /**< TIM1：三相中心对齐互补 PWM + 死区 + TRGO=更新事件 */
    hal_tim_role_timebase,      /**< TIM2：向上计数基础时基（无中断） */
    hal_tim_role_update_irq     /**< TIM3：向上计数时基（中断默认不使能） */
} hal_tim_role_enum;

/** @brief 定时器时基配置（对应平台层 platform_ch32v203_tim_time_base_config_t）。 */
typedef struct {
    uint16_t period;            /**< 自动重装载值 ARR */
    uint16_t prescaler;         /**< 预分频值 PSC（写入寄存器，非分频系数） */
    uint16_t clock_division;    /**< PLAT_TIMER_CLOCK_DIV_x */
    uint16_t counter_mode;      /**< PLAT_TIMER_COUNTER_MODE_x */
    uint8_t repetition_counter; /**< 重复计数 RCR */
} hal_tim_time_base_config_t;

/** @brief 定时器 PWM 输出通道配置（对应平台层 platform_ch32v203_tim_oc_config_t）。 */
typedef struct {
    uint16_t oc_mode;        /**< PLAT_TIMER_OC_MODE_x */
    uint16_t output_state;   /**< PLAT_TIMER_OUTPUT_STATE_x */
    uint16_t output_n_state; /**< PLAT_TIMER_OUTPUT_N_STATE_x（互补输出） */
    uint16_t pulse;          /**< 初始比较值 CCR（初始占空比） */
    uint16_t oc_polarity;    /**< PLAT_TIMER_OC_POLARITY_x */
    uint16_t ocn_polarity;   /**< PLAT_TIMER_OCN_POLARITY_x */
    uint16_t oc_idle_state;  /**< PLAT_TIMER_OC_IDLE_STATE_x */
    uint16_t ocn_idle_state; /**< PLAT_TIMER_OCN_IDLE_STATE_x */
} hal_tim_pwm_config_t;

/** @brief 死区与刹车配置（对应平台层 platform_ch32v203_tim_bdtr_config_t）。 */
typedef struct {
    uint16_t ossr_state;       /**< PLAT_TIMER_OSSR_STATE_x */
    uint16_t ossi_state;       /**< PLAT_TIMER_OSSI_STATE_x */
    uint16_t lock_level;       /**< PLAT_TIMER_LOCK_LEVEL_x */
    uint8_t dead_time;         /**< 死区发生器值 DTG[7:0] */
    uint16_t break_state;      /**< PLAT_TIMER_BREAK_x */
    uint16_t break_polarity;   /**< PLAT_TIMER_BREAK_POLARITY_x */
    uint16_t automatic_output; /**< PLAT_TIMER_AUTOMATIC_OUTPUT_x */
} hal_tim_dead_time_config_t;

/**
 * @brief 定时器完整配置：时基 + PWM 通道 + 死区 + 触发/预装载/中断相关开关。
 * @details 各开关的语义：
 *          - channel_count == 0 表示该实例不使用 PWM 通道（纯时基定时器）；
 *          - is_complementary_enabled 为 true 时才配置死区/刹车寄存器（仅高级定时器）；
 *          - is_trgo_enabled 为 true 时才会选择 TRGO 源（TIM1 用更新事件触发 ADC 注入）；
 *          - is_main_output_enabled 为 true 时使能主输出 MOE（PWM 引脚才真正输出）；
 *          - irq_channel 仅在需要更新中断的实例上填写（PLAT_TIMER_IRQ_x），
 *            构建时不会使能它，需调用方显式调用 hal_tim_config_update_irq()。
 */
typedef struct {
    hal_tim_time_base_config_t time_base; /**< 时基配置 */
    hal_tim_pwm_config_t pwm;             /**< PWM 通道配置（各通道共用） */
    hal_tim_dead_time_config_t dead_time; /**< 死区与刹车配置 */
    uint16_t trgo_source;                 /**< PLAT_TIMER_TRGO_SOURCE_x */
    uint8_t channel_count;                /**< 需要配置的 PWM 通道数，0..4 */
    bool is_pulse_preload_enabled;        /**< 比较寄存器预装载 */
    bool is_period_preload_enabled;       /**< 自动重装载寄存器预装载 */
    bool is_complementary_enabled;        /**< 是否配置死区/刹车（高级定时器） */
    bool is_trgo_enabled;                 /**< 是否选择 TRGO 源 */
    bool is_main_output_enabled;          /**< 是否使能主输出 MOE */
    uint32_t irq_channel;                 /**< PLAT_TIMER_IRQ_x，无中断时填 0 */
    uint32_t timeout_count;               /**< 保留：等待超时计数 */
    void *p_user_data;                    /**< 用户私有数据 */
} hal_tim_config_t;

/** @brief 定时器对象定义（状态、实例基地址与配置）。 */
struct hal_tim {
    hal_tim_state_enum state;
    uintptr_t base_address;
    hal_tim_config_t config;
};

/* --------------------------------------------------------------------------
 * 板级常量：参数取自参考实现 drv_tim.c
 * -------------------------------------------------------------------------- */

/** @brief PWM 载波频率（drv_tim.c 注释标注 6kHz）。 */
#define HAL_TIM_PWM_FREQUENCY_HZ    (12000U)
/** @brief 定时器输入时钟（本工程系统时钟 144MHz，APB2 定时器时钟 144MHz）。 */
#define HAL_TIM_PWM_CLOCK_HZ        (144000000U)
/** @brief 中心对齐 PWM 的预分频值（0 即 1 分频）。 */
#define HAL_TIM_PWM_PRESCALER       (0U)

/**
 * @brief 中心对齐 PWM 的自动重装载值。
 * @details 参考实现写 TIM_Period = PWM_PeriodValue - 1、TIM_Prescaler =
 *          MC_PWM_PrescalerValue（两个宏在参考工程的头文件里，本工程没有）。
 *          本工程按 6kHz 载波 @144MHz、中心对齐折算：
 *          ARR = f_tim / (2 * f_pwm) - 1 = 144e6 / (2 * 6e3) - 1 = 11999。
 */
#define HAL_TIM_PWM_PERIOD          (HAL_TIM_PWM_CLOCK_HZ/(HAL_TIM_PWM_FREQUENCY_HZ * 2)/(HAL_TIM_PWM_PRESCALER + 1))

/**
 * @brief 互补输出死区发生器值 DTG[7:0]。
 * @details 取自参考实现 drv_TIM1_CenterPWM_init(TIM1, 50) 的 PWM_Dead_Time = 50。
 *          实际死区时间 = DTG 编码值 × t_DTS（TIM_CKD_DIV1 时 t_DTS = 1/144MHz
 *          ≈ 6.94ns），本配置约 0.35us；参考注释标注"≈50ns"，两者不一致，
 *          实际值以实测为准（需结合 MOS 驱动与管子开关时间整定）。
 */
#define HAL_TIM_PWM_DEAD_TIME       (50U)
/** @brief 电机实例使用的 PWM 通道数（U/V/W 三相）。 */
#define HAL_TIM_MOTOR_CHANNEL_COUNT (3U)
/** @brief TIM2 基础时基预分频值（drv_TIM2_Count_Init：72 - 1）。 */
#define HAL_TIM_TIMEBASE_PRESCALER  (71U)
/** @brief TIM2 基础时基周期值（drv_TIM2_Count_Init：10000 - 1）。 */
#define HAL_TIM_TIMEBASE_PERIOD     (9999U)
/** @brief TIM3 更新中断时基预分频值（drv_TIM3_UpdateIT_Init：144 - 1）。 */
#define HAL_TIM_UPDATE_IRQ_PRESCALER (143U)
/** @brief TIM3 更新中断时基周期值（drv_TIM3_UpdateIT_Init：1000 - 1）。 */
#define HAL_TIM_UPDATE_IRQ_PERIOD    (999U)

/* --------------------------------------------------------------------------
 * 对象操作
 * -------------------------------------------------------------------------- */

/**
 * @brief 绑定定时器对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（PLAT_PERIPH_TIMx）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效或必需指针为空。
 */
hal_tim_status_enum hal_tim_bind(hal_tim_t *p_object,
                                 uintptr_t base_address,
                                 const hal_tim_config_t *p_config);

/**
 * @brief 初始化定时器：时钟 + 复位 + 时基 + PWM 通道 + 死区 + 预装载 + TRGO + 启动。
 * @details 顺序与参考实现 drv_TIM1_CenterPWM_init() 一致：先配置时基与各通道，
 *          再配置死区/刹车，然后打开比较寄存器与自动重装载预装载、选择 TRGO 源、
 *          使能主输出 MOE，最后启动计数器。TRGO 的设置在启动之前完成，保证
 *          ADC 注入通道一旦使能外部触发就能立刻被更新事件触发。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定、实例不受支持或配置非法。
 */
hal_tim_status_enum hal_tim_init(hal_tim_t *p_object);

/**
 * @brief 关闭并复位定时器外设（先停计数器与主输出，再复位外设）。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或实例不受支持。
 */
hal_tim_status_enum hal_tim_deinit(hal_tim_t *p_object);

/**
 * @brief 启动定时器计数。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或实例不受支持。
 */
hal_tim_status_enum hal_tim_enable(hal_tim_t *p_object);

/**
 * @brief 停止定时器计数。
 * @param p_object 模块对象指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定或实例不受支持。
 */
hal_tim_status_enum hal_tim_disable(hal_tim_t *p_object);

/**
 * @brief 使能/禁止三相 PWM 主输出 MOE（急停时先关 MOE 再停计数器）。
 * @note 仅三相 PWM 实例（配置 is_main_output_enabled == true，即高级定时器）支持，
 *       纯时基实例返回 hal_tim_err_unsupported。
 * @param p_object 模块对象指针。
 * @param enable true 使能主输出，false 关闭主输出（引脚回到空闲电平）。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 对象未绑定。
 * @retval hal_tim_err_unsupported 该实例未使用主输出。
 * @retval hal_tim_err_state 平台层拒绝。
 */
hal_tim_status_enum hal_tim_output_enable(hal_tim_t *p_object, bool enable);

/**
 * @brief 设置指定通道的比较值（占空比）。
 * @param p_object 模块对象指针。
 * @param channel 通道号，取 PLAT_TIMER_CHANNEL_x（1 起始）。
 * @param compare_value 比较值；中心对齐模式下 0..period。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 * @retval hal_tim_err_state 平台层拒绝（通道号越界）。
 */
hal_tim_status_enum hal_tim_set_compare(hal_tim_t *p_object,
                                       uint8_t channel,
                                       uint16_t compare_value);

/**
 * @brief 读取定时器当前计数值。
 * @param p_object 模块对象指针。
 * @param p_counter 输出计数值指针。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 */
hal_tim_status_enum hal_tim_get_counter(hal_tim_t *p_object,
                                       uint16_t *p_counter);

/**
 * @brief 配置更新中断（可选接口，默认不使能）。
 * @details enable = true 时依次清更新挂起位、使能更新中断源、使能对象配置中的
 *          中断线；enable = false 时只关闭更新中断源（中断线保持使能，无中断标志
 *          即不会进入中断服务函数）。
 * @warning 本工程启动文件对所有 IRQ 只提供 `.weak` 默认处理函数（内容为死循环），
 *          调用本接口使能中断前，必须已经实现对应的定时器更新中断服务函数
 *          （例如 TIM3_IRQHandler），否则一进中断即跑飞。
 * @param p_object 模块对象指针。
 * @param enable true 使能更新中断，false 关闭更新中断。
 * @param preemption_priority 抢占优先级（仅使能时使用）。
 * @param sub_priority 子优先级（仅使能时使用）。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效。
 * @retval hal_tim_err_unsupported 对象未配置中断线（irq_channel == 0）。
 * @retval hal_tim_err_state 平台层拒绝。
 */
hal_tim_status_enum hal_tim_config_update_irq(hal_tim_t *p_object,
                                             bool enable,
                                             uint8_t preemption_priority,
                                             uint8_t sub_priority);

/** @brief 查询定时器对象当前状态。 */
hal_tim_state_enum hal_tim_get_state(const hal_tim_t *p_object);

/* --------------------------------------------------------------------------
 * 板级实例：TIM1 三相 PWM、TIM2 基础时基、TIM3 更新中断时基
 * -------------------------------------------------------------------------- */

/** @brief TIM1 三相 PWM 实例（PA8/9/10 + PB13/14/15，TRGO=更新事件）。 */
extern hal_tim_t hal_tim_motor;
/** @brief TIM2 基础时基实例（向上计数，无中断）。 */
extern hal_tim_t hal_tim_timebase;
/** @brief TIM3 更新中断时基实例（中断默认不使能，见 hal_tim_config_update_irq）。 */
extern hal_tim_t hal_tim_update_irq;

/**
 * @brief 构建板级定时器实例（引脚组 + 配置 + 启动）。
 * @details hal_tim_role_motor_pwm 会先经 hal_gpio_ops->pf_build() 构建
 *          hal_gpio_tim 引脚组（PA8/9/10 + PB13/14/15，交替功能推挽），再配置并
 *          启动 TIM1；另两个实例不占用新引脚，直接配置并启动对应定时器。
 *          由于 TIM1 的 TRGO（更新事件）是 ADC2 注入通道的触发源，电机实例必须在
 *          hal_adc_build(&hal_adc_injected, hal_adc_role_injected) 之前构建。
 * @param p_object 定时器对象指针。
 * @param role 板级用途，取 hal_tim_role_x。
 * @retval hal_tim_ok 操作成功。
 * @retval hal_tim_err_param 参数无效或用途不支持。
 * @retval hal_tim_err_state 引脚组构建或平台层配置失败。
 */
hal_tim_status_enum hal_tim_build(hal_tim_t *p_object, hal_tim_role_enum role);

/**
 * @brief HAL 层定时器函数表：板级构建 / 启停 / 主输出 / 比较值 / 计数 / 中断 / 状态 / 备用接口（停用·启停·主输出）。
 * @details 表中的函数指针在 hal_tim.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_tim_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级定时器实例（见 hal_tim_build）。 */
    hal_tim_status_enum (*pf_build)(hal_tim_t *p_object, hal_tim_role_enum role);
    // /** @brief 初始化定时器（见 hal_tim_init）。 */
    // hal_tim_status_enum (*pf_init)(hal_tim_t *p_object);
    /* --- 备用接口：与同名 HAL 函数一一对应，供外部按需接入 --- */
    /** @brief 关闭并复位定时器（见 hal_tim_deinit）。 */
    hal_tim_status_enum (*pf_deinit)(hal_tim_t *p_object);
    /** @brief 启动定时器计数（见 hal_tim_enable）。 */
    hal_tim_status_enum (*pf_enable)(hal_tim_t *p_object);
    /** @brief 停止定时器计数（见 hal_tim_disable）。 */
    hal_tim_status_enum (*pf_disable)(hal_tim_t *p_object);
    /** @brief 使能/禁止 PWM 主输出 MOE（见 hal_tim_output_enable）。 */
    hal_tim_status_enum (*pf_output_enable)(hal_tim_t *p_object, bool enable);
    // /** @brief 设置通道比较值（见 hal_tim_set_compare）。 */
    hal_tim_status_enum (*pf_set_compare)(hal_tim_t *p_object, uint8_t channel,
                                          uint16_t compare_value);
    /** @brief 读当前计数值（见 hal_tim_get_counter）。 */
    hal_tim_status_enum (*pf_get_counter)(hal_tim_t *p_object,
                                          uint16_t *p_counter);
    // hal_tim_status_enum (*pf_motor_pwm_config)(hal_tim_t *p_object);
    // hal_tim_status_enum (*pf_timebase_config)(hal_tim_t *p_object);
    // /** @brief 配置更新中断（见 hal_tim_config_update_irq）。 */
    // hal_tim_status_enum (*pf_update_irq_config)(hal_tim_t *p_object);
    /** @brief 查询对象状态（见 hal_tim_get_state）。 */
    hal_tim_state_enum (*pf_get_state)(const hal_tim_t *p_object);
} hal_tim_ops_t;

/** @brief HAL 层定时器函数表指针（在 hal_tim.c 中定义并绑定）。 */
extern const hal_tim_ops_t *const hal_tim_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_TIM_H__ */
