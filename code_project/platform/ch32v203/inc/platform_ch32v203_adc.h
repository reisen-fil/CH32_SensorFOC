/**
 * @file platform_ch32v203_adc.h
 * @brief CH32V203 ADC 的 MCU 抽象接口（取值宏 + 操作函数表）。
 * @details 与 GPIO/SPI 平台层同构：
 *          - 取值宏：工作模式 / 外部触发 / 数据对齐 / 通道 / 采样时间 / 标志 / 中断线；
 *          - 操作函数表 platform_ch32v203_adc_ops_t：把标准库 ADC 接口
 *            （时钟使能、ADC_Init、使能、内部缓冲开关、校准、规则与注入通道配置、
 *            注入触发、软件启动、中断使能、DMA 请求开关、数据寄存器地址查询、
 *            取值、标志查询）逐个做简单函数封装后绑定，并通过 plat_ch32v203_adc_ops
 *            指针对外外露。
 *          ADC 的对象模型、板级实例（ADC1 规则+DMA / ADC2 注入）与收发流程实现于
 *          hal_adc.c/h。
 * @note 本层不提供任何业务语义（母线电压、相电流等），只做 MCU 抽象。
 * @note 本层只保留"外设侧"的 DMA 相关操作（ADC_DMACmd 请求开关与 RDATAR 地址查询）；
 *       DMA 通道的编号、装配、使能与状态一律由 platform_ch32v203_dma.c 统一提供，
 *       本层不再暴露任何 DMA 通道符号与通道配置接口。
 */
#ifndef __PLATFORM_CH32V203_ADC_H__
#define __PLATFORM_CH32V203_ADC_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/** @brief ADC 工作模式。 */
#define PLAT_ADC_MODE_INDEPENDENT              ADC_Mode_Independent

/** @brief 规则通道外部触发：软件触发。 */
#define PLAT_ADC_EXTERNAL_TRIG_NONE            ADC_ExternalTrigConv_None

/** @brief 注入通道外部触发：TIM1 TRGO（电流采样与 PWM 同步）。 */
#define PLAT_ADC_EXTERNAL_TRIG_INJEC_T1_TRGO   ADC_ExternalTrigInjecConv_T1_TRGO

/** @brief ADC 数据对齐方式。 */
#define PLAT_ADC_DATA_ALIGN_RIGHT              ADC_DataAlign_Right
#define PLAT_ADC_DATA_ALIGN_LEFT               ADC_DataAlign_Left

/** @brief ADC 通道（CH32V203 通道 0..9）。 */
#define PLAT_ADC_CHANNEL_0                     ADC_Channel_0
#define PLAT_ADC_CHANNEL_1                     ADC_Channel_1
#define PLAT_ADC_CHANNEL_2                     ADC_Channel_2
#define PLAT_ADC_CHANNEL_3                     ADC_Channel_3
#define PLAT_ADC_CHANNEL_4                     ADC_Channel_4
#define PLAT_ADC_CHANNEL_5                     ADC_Channel_5
#define PLAT_ADC_CHANNEL_6                     ADC_Channel_6
#define PLAT_ADC_CHANNEL_7                     ADC_Channel_7
#define PLAT_ADC_CHANNEL_8                     ADC_Channel_8
#define PLAT_ADC_CHANNEL_9                     ADC_Channel_9

/** @brief ADC 采样时间。 */
#define PLAT_ADC_SAMPLE_TIME_1CYCLES5          ADC_SampleTime_1Cycles5
#define PLAT_ADC_SAMPLE_TIME_7CYCLES5          ADC_SampleTime_7Cycles5
#define PLAT_ADC_SAMPLE_TIME_13CYCLES5         ADC_SampleTime_13Cycles5
#define PLAT_ADC_SAMPLE_TIME_28CYCLES5         ADC_SampleTime_28Cycles5
#define PLAT_ADC_SAMPLE_TIME_55CYCLES5         ADC_SampleTime_55Cycles5
#define PLAT_ADC_SAMPLE_TIME_71CYCLES5         ADC_SampleTime_71Cycles5
#define PLAT_ADC_SAMPLE_TIME_239CYCLES5        ADC_SampleTime_239Cycles5

/** @brief ADC 状态标志。 */
#define PLAT_ADC_FLAG_EOC                      ADC_FLAG_EOC
#define PLAT_ADC_FLAG_JEOC                     ADC_FLAG_JEOC

/** @brief ADC 中断源（供 pf_it_config 使用）。 */
#define PLAT_ADC_IT_EOC                        ADC_IT_EOC
#define PLAT_ADC_IT_JEOC                       ADC_IT_JEOC

/** @brief ADC1/ADC2 共用中断线（供 pf_irq_enable 使用）。 */
#define PLAT_ADC_IRQ_ADC1_2                    PLAT_IRQ_ADC

/** @brief ADC 校准等待次数上限（防止硬件异常时死等）。 */
#define PLAT_ADC_CALIBRATION_TIMEOUT           (100000U)

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_adc.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_ADC_INSTANCE_TYPE                 ADC_TypeDef
#define PLAT_ADC_INIT_TYPE                     ADC_InitTypeDef

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ADC 平台配置（对应标准库 ADC_InitTypeDef 的可移植子集）。 */
typedef struct {
    uint32_t mode;              /**< PLAT_ADC_MODE_INDEPENDENT */
    bool is_scan_enabled;       /**< 扫描模式 */
    bool is_continuous_enabled; /**< 连续转换 */
    uint32_t external_trigger;  /**< 规则通道外部触发，PLAT_ADC_EXTERNAL_TRIG_x */
    uint32_t data_align;        /**< PLAT_ADC_DATA_ALIGN_x */
    uint8_t channel_count;      /**< 规则序列长度 */
} platform_ch32v203_adc_config_t;

/**
 * @brief CH32V203 ADC 操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 返回 bool，true 表示成功；取值类函数直接返回寄存器值。
 *          ADC 规则通道的数据搬运由通用 DMA 完成，其通道编号与通道配置由
 *          plat_ch32v203_dma_ops（platform_ch32v203_dma.c）统一提供；本层只保留
 *          "外设侧"的两个 DMA 相关操作：DMA 请求开关与数据寄存器地址查询。
 */
typedef struct {
    /** @brief 使能 ADC 实例时钟与 DMA1 时钟。 */
    bool (*pf_clock_enable)(uintptr_t instance);
    /** @brief 复位 ADC 外设（ADC_DeInit）。 */
    bool (*pf_deinit)(uintptr_t instance);
    /** @brief 配置 ADC（ADC_Init）。 */
    bool (*pf_config)(uintptr_t instance,
                      const platform_ch32v203_adc_config_t *p_config);
    /** @brief 使能/禁止 ADC（ADC_Cmd）。 */
    bool (*pf_cmd)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止 ADC 内部缓冲（ADC_BufferCmd）。 */
    bool (*pf_buffer_cmd)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止 ADC 的 DMA 请求（ADC_DMACmd，外设侧）。 */
    bool (*pf_dma_request_cmd)(uintptr_t instance, bool enable);
    /**
     * @brief 取 ADC 规则数据寄存器地址（&ADCx->RDATAR，外设侧）。
     * @param instance 实例基地址，取 PLAT_PERIPH_ADC1 / PLAT_PERIPH_ADC2。
     * @return 数据寄存器地址；实例不受支持时返回 0。
     * @note 供上层拼装 DMA 通道配置（DMA 外设地址）使用，HAL 不得自行书写寄存器符号。
     */
    uintptr_t (*pf_get_data_register_address)(uintptr_t instance);
    /** @brief 复位并启动校准，等待校准完成（带超时）。 */
    bool (*pf_calibrate)(uintptr_t instance);
    /** @brief 配置规则通道的通道号/序号/采样时间。 */
    bool (*pf_regular_channel_config)(uintptr_t instance, uint8_t channel,
                                      uint8_t rank, uint8_t sample_time);
    /** @brief 配置注入序列长度。 */
    bool (*pf_injected_sequencer_length)(uintptr_t instance, uint8_t length);
    /** @brief 配置注入通道的通道号/序号/采样时间。 */
    bool (*pf_injected_channel_config)(uintptr_t instance, uint8_t channel,
                                       uint8_t rank, uint8_t sample_time);
    /** @brief 配置并使能/禁止注入通道外部触发。 */
    bool (*pf_external_trig_injected)(uintptr_t instance, uint32_t trigger,
                                      bool enable);
    /** @brief 使能/禁止规则通道软件启动转换。 */
    bool (*pf_software_start)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止指定的 ADC 中断源。 */
    bool (*pf_it_config)(uintptr_t instance, uint16_t it, bool enable);
    /** @brief 配置并使能中断线（ADC1/ADC2 共用）。 */
    bool (*pf_irq_enable)(uint32_t irq_channel, uint8_t preemption_priority,
                          uint8_t sub_priority);
    /** @brief 读规则通道转换值（ADC_GetConversionValue）。 */
    uint16_t (*pf_get_value)(uintptr_t instance);
    /** @brief 读注入通道转换值（ADC_GetInjectedConversionValue）。 */
    uint16_t (*pf_get_injected_value)(uintptr_t instance, uint8_t rank);
    /** @brief 查询状态标志（PLAT_ADC_FLAG_x）。 */
    bool (*pf_get_flag)(uintptr_t instance, uint8_t flag);
} platform_ch32v203_adc_ops_t;

/** @brief CH32V203 ADC 操作函数表指针（在 platform_ch32v203_adc.c 中组装）。 */
extern const platform_ch32v203_adc_ops_t *const plat_ch32v203_adc_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_ADC_H__ */
