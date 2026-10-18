/**
 * @file hal_adc.h
 * @brief ADC 硬件抽象接口（含 CH32V203 板级实例与 HAL 层函数表）。
 * @details 与 GPIO/SPI 同构：对象操作函数同层实现，硬件访问统一经平台 ADC 操作
 *          函数表指针 plat_ch32v203_adc_ops 完成，本层不含厂商库符号。
 *          板级实例按"用途"区分：
 *          - hal_adc_reguler：ADC1 规则通道扫描 + 循环 DMA 搬运（通道号经
 *            plat_ch32v203_dma_ops->pf_channel_get() 取得，本层不关心具体通道）
 *            （PA0 母线电压、PA1 控制电位器、PA2 NTC），对应参考实现 drv_ADCtoDMA_Regular_init()；
 *          - hal_adc_injected：ADC2 注入通道 + TIM1_TRGO 触发（PB1 运放1 输出、
 *            PA4 运放2 输出），对应参考实现 drv_ADC_Inject_init()。
 *          注入通道的引脚（PA4/PB1）由运放引脚组配置，故注入实例应在
 *          hal_opamp_build() 之后构建。
 * @note 本层不引入组配置/多视图机制：一个 hal_adc_t 直接对应一个 ADC 实例。
 */
#ifndef __HAL_ADC_H__
#define __HAL_ADC_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_ADC_* 模式/触发/对齐/通道/采样时间/标志取值宏 */
#include "platform_ch32v203_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ADC 对象前向声明。 */
typedef struct hal_adc hal_adc_t;

/** @brief ADC对象的生命周期状态。 */
typedef enum {
    hal_adc_state_reset = 0,
    hal_adc_state_ready,
    hal_adc_state_enabled,
    hal_adc_state_disabled,
    hal_adc_state_error
} hal_adc_state_enum;

/** @brief ADC接口调用返回的状态码。 */
typedef enum {
    hal_adc_ok = 0,
    hal_adc_err_param = -1,
    hal_adc_err_state = -2,
    hal_adc_err_unsupported = -3,
    hal_adc_err_timeout = -4
} hal_adc_status_enum;

/** @brief ADC 板级用途。 */
typedef enum {
    hal_adc_role_reguler = 0,   /**< ADC1：规则通道扫描 + DMA 循环搬运 */
    hal_adc_role_injected       /**< ADC2：注入通道 + TIM1_TRGO 触发 */
} hal_adc_role_enum;

/** @brief 序列中单个通道的配置（通道号 / 序号 / 采样时间）。 */
typedef struct {
    uint8_t channel;        /**< PLAT_ADC_CHANNEL_x */
    uint8_t rank;           /**< 序列序号，从 1 开始 */
    uint8_t sample_time;    /**< PLAT_ADC_SAMPLE_TIME_x */
} hal_adc_channel_config_t;

/** @brief ADC 工作模式、扫描/连续转换、触发源、数据对齐与序列长度配置。 */
typedef struct {
    uint32_t mode;              /**< PLAT_ADC_MODE_x */
    bool is_scan_enabled;       /**< 扫描模式 */
    bool is_continuous_enabled; /**< 连续转换 */
    uint32_t external_trigger;  /**< 规则通道触发源，PLAT_ADC_EXTERNAL_TRIG_x */
    uint32_t data_align;        /**< PLAT_ADC_DATA_ALIGN_x */
    uint8_t channel_count;      /**< 序列长度 */
    uint32_t timeout_count;     /**< 保留：采样等待超时计数 */
    void *p_user_data;
} hal_adc_config_t;

/** @brief ADC 对象定义（状态、实例基地址与配置）。 */
struct hal_adc {
    hal_adc_state_enum state;
    uintptr_t base_address;
    hal_adc_config_t config;
};

/**
 * @brief 绑定ADC对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（PLAT_PERIPH_ADCx）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效或必需指针为空。
 */
hal_adc_status_enum hal_adc_bind(hal_adc_t *p_object,
                                 uintptr_t base_address,
                                 const hal_adc_config_t *p_config);

/** @brief 初始化ADC：时钟 + 配置 + 内部缓冲关闭 + 使能 + 校准。 */
hal_adc_status_enum hal_adc_init(hal_adc_t *p_object);

/** @brief 关闭并复位ADC外设。 */
hal_adc_status_enum hal_adc_deinit(hal_adc_t *p_object);

/** @brief 使能ADC转换。 */
hal_adc_status_enum hal_adc_enable(hal_adc_t *p_object);

/** @brief 禁止ADC转换。 */
hal_adc_status_enum hal_adc_disable(hal_adc_t *p_object);

/**
 * @brief 启动规则通道转换（软件触发）。
 * @param p_object 模块对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 * @retval hal_adc_err_state 平台层拒绝（实例不受支持）。
 */
hal_adc_status_enum hal_adc_start(hal_adc_t *p_object);

/**
 * @brief 装配指定缓冲区的循环 DMA 并启动规则通道连续转换。
 * @param p_object 模块对象指针。
 * @param p_buffer 目标缓冲区（半字数组）。
 * @param buffer_size 缓冲区长度（按半字计）。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 * @retval hal_adc_err_state 平台层拒绝（实例不受支持）。
 */
hal_adc_status_enum hal_adc_start_dma(hal_adc_t *p_object,
                                      void *p_buffer,
                                      uint16_t buffer_size);

/**
 * @brief 读取规则通道数据寄存器中的最新转换值。
 * @param p_object 模块对象指针。
 * @param p_value 输出数据指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 */
hal_adc_status_enum hal_adc_get_value(hal_adc_t *p_object, uint16_t *p_value);

/**
 * @brief 读取注入通道转换值（电流采样路径）。
 * @param p_object 模块对象指针。
 * @param rank 注入序号，从 1 开始。
 * @param p_value 输出数据指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效或序号越界。
 */
hal_adc_status_enum hal_adc_get_injected_value(hal_adc_t *p_object,
                                               uint8_t rank,
                                               uint16_t *p_value);

/**
 * @brief 使能/禁止注入组转换完成中断源（JEOC）。
 * @warning 使能前必须已实现 ADC1_2_IRQHandler：本工程启动文件对该向量只提供
 *          `.weak` 默认处理函数（内容为死循环），未实现就打开中断会直接跑飞。
 *          中断线由 hal_adc_build(..., hal_adc_role_injected) 配置，无需重复配置。
 * @param p_object 模块对象指针。
 * @param enable true 使能 JEOC 中断源，false 关闭。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定。
 * @retval hal_adc_err_state 平台层拒绝（实例不受支持）。
 */
hal_adc_status_enum hal_adc_enable_injected_irq(hal_adc_t *p_object,
                                                bool enable);

/**
 * @brief 查询 ADC 状态/中断标志位（JEOC / EOC）。
 * @details 经平台层标志查询接口读取标志寄存器，供中断服务函数判断是否本外设事件，
 *          典型用法：PLAT_ISR_DEFINE(PLAT_ISR_ADC1_2) { … pf_get_flag(&hal_adc_injected,
 *          PLAT_ADC_FLAG_JEOC, &is_set) … }；标志的清理由平台层外设操作流程负责，
 *          本层不提供清标志接口。
 * @param p_object 模块对象指针。
 * @param flag 标志位，取 PLAT_ADC_FLAG_x（如 PLAT_ADC_FLAG_JEOC）。
 * @param p_is_set 输出：true = 置位，false = 未置位。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定或输出指针为空。
 */
hal_adc_status_enum hal_adc_get_flag(const hal_adc_t *p_object,
                                     uint8_t flag,
                                     bool *p_is_set);

/** @brief 查询ADC对象当前状态。 */
hal_adc_state_enum hal_adc_get_state(const hal_adc_t *p_object);

/* --------------------------------------------------------------------------
 * 板级实例：ADC1 规则通道+DMA、ADC2 注入通道
 * -------------------------------------------------------------------------- */

/** @brief ADC1 规则通道实例（PA0 电位器、PA1 母线电压、PA2 NTC + 循环 DMA 搬运）。 */
extern hal_adc_t hal_adc_reguler;
/** @brief ADC2 注入通道实例（PB1 运放1 输出、PA4 运放2 输出 + TIM1_TRGO）。 */
extern hal_adc_t hal_adc_injected;

/** @brief 规则通道数（DMA 缓存长度）。 */
#define HAL_ADC_REGULER_CHANNEL_COUNT  (3U)
/** @brief 注入通道数。 */
#define HAL_ADC_INJECTED_CHANNEL_COUNT (2U)

/** @brief 规则通道 DMA 采样缓存（长度 = HAL_ADC_REGULER_CHANNEL_COUNT，由 DMA 更新）。 */
extern uint16_t hal_adc_reguler_dma_value[HAL_ADC_REGULER_CHANNEL_COUNT];

/**
 * @brief 构建板级 ADC 实例（引脚组 + 配置 + 通道序列 + 触发/DMA + 启动）。
 * @details 规则实例会构建 PA0/PA1 引脚组并启动 DMA 连续转换；
 *          注入实例使用运放引脚组配置的 PA4/PB1，并使能 TIM1_TRGO 触发，
 *          同时按板级常量配置并使能 ADC1/ADC2 共用中断线（注入中断源 JEOC
 *          默认不使能，业务层实现 ADC1_2_IRQHandler 后可用
 *          hal_adc_ops->pf_enable_injected_irq() 按需打开）。
 * @note 无论构建哪个实例，都会在 ADC 使能/校准之前把共享的 ADCCLK 配置为
 *       PCLK2/8（72MHz 系统下 = 9MHz，满足数据手册上限 fADC ≤ 14MHz）；
 *       该分频器由 ADC1/ADC2 共用，重复配置同一分频值幂等。
 * @param p_object ADC 对象指针。
 * @param role 板级用途，取 hal_adc_role_x。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 * @retval hal_adc_err_state 引脚组构建或平台层配置/校准失败。
 */
hal_adc_status_enum hal_adc_build(hal_adc_t *p_object, hal_adc_role_enum role);

/**
 * @brief HAL 层 ADC 函数表：板级构建 / 启动 / 取值 / 标志查询 / 状态 / 备用接口（停用·使能·禁止）。
 * @details 表中的函数指针在 hal_adc.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_adc_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级 ADC 实例（见 hal_adc_build）。 */
    hal_adc_status_enum (*pf_build)(hal_adc_t *p_object, hal_adc_role_enum role);
    /** @brief 启动规则通道转换（见 hal_adc_start）。 */
    hal_adc_status_enum (*pf_start)(hal_adc_t *p_object);
    /** @brief 读取规则通道值（见 hal_adc_get_value）。 */
    hal_adc_status_enum (*pf_get_value)(hal_adc_t *p_object, uint16_t *p_value);
    /** @brief 读取注入通道值（见 hal_adc_get_injected_value）。 */
    hal_adc_status_enum (*pf_get_injected_value)(hal_adc_t *p_object,
                                                 uint8_t rank,
                                                 uint16_t *p_value);
    /** @brief 使能/禁止注入组 JEOC 中断源（见 hal_adc_enable_injected_irq）。 */
    hal_adc_status_enum (*pf_enable_injected_irq)(hal_adc_t *p_object,
                                                  bool enable);
    /** @brief 查询 ADC 标志位（见 hal_adc_get_flag）。 */
    hal_adc_status_enum (*pf_get_flag)(const hal_adc_t *p_object,
                                       uint8_t flag,
                                       bool *p_is_set);
    /** @brief 查询对象状态（见 hal_adc_get_state）。 */
    hal_adc_state_enum (*pf_get_state)(const hal_adc_t *p_object);

    /* --- 备用接口：与同名 HAL 函数一一对应，供外部按需接入 --- */
    /** @brief 关闭并复位ADC外设（见 hal_adc_deinit）。 */
    hal_adc_status_enum (*pf_deinit)(hal_adc_t *p_object);
    /** @brief 使能ADC转换（见 hal_adc_enable）。 */
    hal_adc_status_enum (*pf_enable)(hal_adc_t *p_object);
    /** @brief 禁止ADC转换（见 hal_adc_disable）。 */
    hal_adc_status_enum (*pf_disable)(hal_adc_t *p_object);
} hal_adc_ops_t;

/** @brief HAL 层 ADC 函数表指针（在 hal_adc.c 中定义并绑定）。 */
extern const hal_adc_ops_t *const hal_adc_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_ADC_H__ */
