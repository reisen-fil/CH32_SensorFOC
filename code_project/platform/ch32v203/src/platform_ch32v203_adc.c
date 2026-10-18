/**
 * @file platform_ch32v203_adc.c
 * @brief CH32V203 ADC 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 ADC/DMA 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_adc_ops 函数表对外绑定；hal_adc.c 通过该表访问 ADC 硬件。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；厂商类型名一律经
 *       platform_ch32v203_adc.h 的"类型接入宏"（PLAT_ADC_*_TYPE）书写，
 *       本文件内不出现任何厂商类型名。时钟使能沿用平台时钟宏
 *       PLAT_RCC_APB2_CLOCK_ENABLE / PLAT_RCC_AHB_CLOCK_ENABLE，中断配置沿用
 *       PLAT_PFIC_INIT。本文件只保留"外设侧"的 DMA 相关操作（ADC_DMACmd 请求开关与
 *       ADC 的 RDATAR 寄存器地址查询）；DMA 通道的编号、装配、使能与状态统一由
 *       platform_ch32v203_dma.c 提供，故本文件不再书写任何厂商 DMA 通道符号。
 */
#include "platform_ch32v203_adc.h"
#include <stddef.h>

/**
 * @brief 判断实例是否为受支持的 ADC 外设。
 * @param instance 实例基地址，取 PLAT_PERIPH_ADC1 / PLAT_PERIPH_ADC2。
 * @retval true 受支持。
 * @retval false 其它外设。
 */
static bool plat_ch32v203_adc_is_valid(uintptr_t instance)
{
    PLAT_ADC_INSTANCE_TYPE *p_adc = (PLAT_ADC_INSTANCE_TYPE *)(instance);

    return (p_adc == PLAT_PERIPH_ADC1) || (p_adc == PLAT_PERIPH_ADC2);
}

/**
 * @brief 使能 ADC 实例时钟与 DMA1 时钟。
 * @note 时钟使能是本文件通过平台宏完成的动作之一。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_clock_enable(uintptr_t instance)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    if ((PLAT_ADC_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_ADC1) {
        PLAT_RCC_APB2_CLOCK_ENABLE(PLAT_RCC_APB2_ADC1);
    } else {
        PLAT_RCC_APB2_CLOCK_ENABLE(PLAT_RCC_APB2_ADC2);
    }
    PLAT_RCC_AHB_CLOCK_ENABLE(PLAT_RCC_AHB_DMA1);

    return true;
}

/**
 * @brief 复位 ADC 外设（ADC_DeInit）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_deinit(uintptr_t instance)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_DeInit((PLAT_ADC_INSTANCE_TYPE *)(instance));
    return true;
}

/**
 * @brief 配置 ADC（ADC_Init）。
 * @param instance 实例基地址。
 * @param p_config 平台 ADC 配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持或配置为空。
 */
static bool plat_ch32v203_adc_config(
    uintptr_t instance,
    const platform_ch32v203_adc_config_t *p_config)
{
    PLAT_ADC_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_adc_is_valid(instance))) {
        return false;
    }

    init_config.ADC_Mode = p_config->mode;
    init_config.ADC_ScanConvMode = p_config->is_scan_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.ADC_ContinuousConvMode =
        p_config->is_continuous_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.ADC_ExternalTrigConv = p_config->external_trigger;
    init_config.ADC_DataAlign = p_config->data_align;
    init_config.ADC_NbrOfChannel = p_config->channel_count;

    ADC_Init((PLAT_ADC_INSTANCE_TYPE *)(instance), &init_config);
    return true;
}

/**
 * @brief 使能/禁止 ADC（ADC_Cmd）。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_Cmd((PLAT_ADC_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止 ADC 内部缓冲（ADC_BufferCmd）。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_buffer_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_BufferCmd((PLAT_ADC_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止 ADC 的 DMA 请求（ADC_DMACmd，外设侧）。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_dma_request_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_DMACmd((PLAT_ADC_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 取 ADC 规则数据寄存器地址（&ADCx->RDATAR，外设侧）。
 * @param instance 实例基地址。
 * @return 数据寄存器地址；实例不受支持时返回 0。
 */
static uintptr_t plat_ch32v203_adc_get_data_register_address(uintptr_t instance)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return 0U;
    }

    return (uintptr_t)(&((PLAT_ADC_INSTANCE_TYPE *)(instance))->RDATAR);
}

/**
 * @brief 复位并启动 ADC 校准，等待校准完成（带超时）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持或等待校准超时。
 */
static bool plat_ch32v203_adc_calibrate(uintptr_t instance)
{
    PLAT_ADC_INSTANCE_TYPE *p_adc;
    uint32_t timeout;

    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    p_adc = (PLAT_ADC_INSTANCE_TYPE *)(instance);

    ADC_ResetCalibration(p_adc);
    timeout = PLAT_ADC_CALIBRATION_TIMEOUT;
    while (ADC_GetResetCalibrationStatus(p_adc) != PLAT_RESET) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    ADC_StartCalibration(p_adc);
    timeout = PLAT_ADC_CALIBRATION_TIMEOUT;
    while (ADC_GetCalibrationStatus(p_adc) != PLAT_RESET) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 配置规则通道的通道号/序号/采样时间。
 * @param instance 实例基地址。
 * @param channel 通道号，取 PLAT_ADC_CHANNEL_x。
 * @param rank 序列序号（从 1 开始）。
 * @param sample_time 采样时间，取 PLAT_ADC_SAMPLE_TIME_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_regular_channel_config(uintptr_t instance,
                                                     uint8_t channel,
                                                     uint8_t rank,
                                                     uint8_t sample_time)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_RegularChannelConfig((PLAT_ADC_INSTANCE_TYPE *)(instance), channel, rank,
                             sample_time);
    return true;
}

/**
 * @brief 配置注入序列长度。
 * @param instance 实例基地址。
 * @param length 注入序列长度。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_injected_sequencer_length(uintptr_t instance,
                                                        uint8_t length)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_InjectedSequencerLengthConfig((PLAT_ADC_INSTANCE_TYPE *)(instance), length);
    return true;
}

/**
 * @brief 配置注入通道的通道号/序号/采样时间。
 * @param instance 实例基地址。
 * @param channel 通道号，取 PLAT_ADC_CHANNEL_x。
 * @param rank 注入序号（从 1 开始）。
 * @param sample_time 采样时间，取 PLAT_ADC_SAMPLE_TIME_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_injected_channel_config(uintptr_t instance,
                                                      uint8_t channel,
                                                      uint8_t rank,
                                                      uint8_t sample_time)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_InjectedChannelConfig((PLAT_ADC_INSTANCE_TYPE *)(instance), channel, rank,
                              sample_time);
    return true;
}

/**
 * @brief 配置并使能/禁止注入通道外部触发。
 * @param instance 实例基地址。
 * @param trigger 触发源，取 PLAT_ADC_EXTERNAL_TRIG_INJEC_x。
 * @param enable true 使能外部触发，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_external_trig_injected(uintptr_t instance,
                                                     uint32_t trigger,
                                                     bool enable)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_ExternalTrigInjectedConvConfig((PLAT_ADC_INSTANCE_TYPE *)(instance), trigger);
    ADC_ExternalTrigInjectedConvCmd((PLAT_ADC_INSTANCE_TYPE *)(instance),
                                    enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止规则通道软件启动转换。
 * @param instance 实例基地址。
 * @param enable true 启动，false 停止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_software_start(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_SoftwareStartConvCmd((PLAT_ADC_INSTANCE_TYPE *)(instance),
                             enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止指定的 ADC 中断源。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_ADC_IT_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_adc_it_config(uintptr_t instance, uint16_t it,
                                        bool enable)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    ADC_ITConfig((PLAT_ADC_INSTANCE_TYPE *)(instance), it, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 配置并使能 ADC 中断线（ADC1/ADC2 共用）。
 * @param irq_channel 中断号，取 PLAT_ADC_IRQ_x。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_adc_irq_enable(uint32_t irq_channel,
                                         uint8_t preemption_priority,
                                         uint8_t sub_priority)
{
    PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority,
                   PLAT_ENABLE);
    return true;
}

/**
 * @brief 读规则通道转换值（ADC_GetConversionValue）。
 * @param instance 实例基地址。
 * @return 转换结果；实例非法时返回 0。
 */
static uint16_t plat_ch32v203_adc_get_value(uintptr_t instance)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return 0U;
    }

    return ADC_GetConversionValue((PLAT_ADC_INSTANCE_TYPE *)(instance));
}

/**
 * @brief 读注入通道转换值（ADC_GetInjectedConversionValue）。
 * @param instance 实例基地址。
 * @param rank 注入序号（从 1 开始）。
 * @return 转换结果；实例非法时返回 0。
 */
static uint16_t plat_ch32v203_adc_get_injected_value(uintptr_t instance,
                                                     uint8_t rank)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return 0U;
    }

    return ADC_GetInjectedConversionValue((PLAT_ADC_INSTANCE_TYPE *)(instance), rank);
}

/**
 * @brief 查询状态标志（ADC_GetFlagStatus）。
 * @param instance 实例基地址。
 * @param flag 标志，取 PLAT_ADC_FLAG_x。
 * @retval true 标志置位。
 * @retval false 标志复位或实例非法。
 */
static bool plat_ch32v203_adc_get_flag(uintptr_t instance, uint8_t flag)
{
    if (!plat_ch32v203_adc_is_valid(instance)) {
        return false;
    }

    return (ADC_GetFlagStatus((PLAT_ADC_INSTANCE_TYPE *)(instance), flag) != PLAT_RESET);
}

/** @brief CH32V203 ADC 操作函数表（本文件私有）。 */
static const platform_ch32v203_adc_ops_t s_adc_ops = {
    .pf_clock_enable = plat_ch32v203_adc_clock_enable,
    .pf_deinit = plat_ch32v203_adc_deinit,
    .pf_config = plat_ch32v203_adc_config,
    .pf_cmd = plat_ch32v203_adc_cmd,
    .pf_buffer_cmd = plat_ch32v203_adc_buffer_cmd,
    .pf_dma_request_cmd = plat_ch32v203_adc_dma_request_cmd,
    .pf_get_data_register_address = plat_ch32v203_adc_get_data_register_address,
    .pf_calibrate = plat_ch32v203_adc_calibrate,
    .pf_regular_channel_config = plat_ch32v203_adc_regular_channel_config,
    .pf_injected_sequencer_length = plat_ch32v203_adc_injected_sequencer_length,
    .pf_injected_channel_config = plat_ch32v203_adc_injected_channel_config,
    .pf_external_trig_injected = plat_ch32v203_adc_external_trig_injected,
    .pf_software_start = plat_ch32v203_adc_software_start,
    .pf_it_config = plat_ch32v203_adc_it_config,
    .pf_irq_enable = plat_ch32v203_adc_irq_enable,
    .pf_get_value = plat_ch32v203_adc_get_value,
    .pf_get_injected_value = plat_ch32v203_adc_get_injected_value,
    .pf_get_flag = plat_ch32v203_adc_get_flag
};

/** @brief CH32V203 ADC 操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_adc_ops_t *const plat_ch32v203_adc_ops = &s_adc_ops;
