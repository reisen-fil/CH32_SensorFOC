/**
 * @file platform_ch32v203_dma.c
 * @brief CH32V203 DMA 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 DMA 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_dma_ops 函数表对外绑定；hal_adc.c / hal_uart.c 等上层
 *          通过该表访问 DMA 硬件，各外设平台层不再各自实现通道配置。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；仅时钟使能沿用平台时钟宏
 *       PLAT_RCC_AHB_CLOCK_ENABLE，中断线配置沿用 PLAT_PFIC_INIT。
 * @note 厂商类型名（DMA_Channel_TypeDef / DMA_InitTypeDef）不直接书写于本文件，
 *       而是经 platform_ch32v203_dma.h 的 PLAT_DMA_CHANNEL_INSTANCE_TYPE /
 *       PLAT_DMA_INIT_TYPE 类型接入宏引入；厂商符号（DMA1_Channelx /
 *       DMA1_FLAG_xxxn / DMA1_IT_xxxn / DMA1_Channelx_IRQn）只出现在本文件，
 *       对外的通道号与事件类别均为 platform_ch32v203_dma.h 中的抽象枚举。
 * @note 本文件只做 MCU 抽象，不含任何业务语义（采样序列、收帧、总线时序均在上层）。
 */
#include "platform_ch32v203_dma.h"
#include <stddef.h>

/* ==========================================================================
 * 1) 抽象通道号 → DMA1 通道指针
 * ========================================================================== */

/** @brief 通道号索引表（下标 0 保留为"无通道"）。 */
static PLAT_DMA_CHANNEL_INSTANCE_TYPE *const s_dma_channels[8] = {
    NULL,
    DMA1_Channel1,
    DMA1_Channel2,
    DMA1_Channel3,
    DMA1_Channel4,
    DMA1_Channel5,
    DMA1_Channel6,
    DMA1_Channel7
};

/**
 * @brief 判断通道号是否受支持。
 * @param channel 抽象通道号。
 * @retval true 受支持（CH1..CH7）。
 * @retval false 非法通道或 channel_none。
 */
static bool plat_ch32v203_dma_is_valid(uint8_t channel)
{
    return (channel >= (uint8_t)platform_ch32v203_dma_channel_1) &&
           (channel <= (uint8_t)platform_ch32v203_dma_channel_7);
}

/**
 * @brief 取通道指针。
 * @param channel 抽象通道号。
 * @return 通道指针；非法通道返回 NULL。
 */
static PLAT_DMA_CHANNEL_INSTANCE_TYPE *plat_ch32v203_dma_channel_pointer(uint8_t channel)
{
    if (!plat_ch32v203_dma_is_valid(channel)) {
        return NULL;
    }

    return s_dma_channels[channel];
}

/* ==========================================================================
 * 2) (通道号, 事件类别) → 厂商标志/中断位
 *
 * CH32V203 的 INTFR / INTENR 每通道占 4 位：bit0=GL、bit1=TC、bit2=HT、bit3=TE，
 * 故标志值 = 1 << ((通道号 - 1) * 4 + 事件类别)。下面用编译期断言把该假设与
 * 厂商宏钉死，避免将来换库时静默错位。
 * ========================================================================== */

typedef char plat_dma_check_gl1[(DMA1_FLAG_GL1 == (1U << 0)) ? 1 : -1];
typedef char plat_dma_check_tc2[(DMA1_FLAG_TC2 == (1U << 5)) ? 1 : -1];
typedef char plat_dma_check_ht3[(DMA1_FLAG_HT3 == (1U << 10)) ? 1 : -1];
typedef char plat_dma_check_te4[(DMA1_FLAG_TE4 == (1U << 15)) ? 1 : -1];
typedef char plat_dma_check_tc5[(DMA1_FLAG_TC5 == (1U << 17)) ? 1 : -1];
typedef char plat_dma_check_gl6[(DMA1_FLAG_GL6 == (1U << 20)) ? 1 : -1];
typedef char plat_dma_check_te7[(DMA1_FLAG_TE7 == (1U << 27)) ? 1 : -1];
typedef char plat_dma_check_it_tc2[(DMA1_IT_TC2 == DMA1_FLAG_TC2) ? 1 : -1];
typedef char plat_dma_check_it_te5[(DMA1_IT_TE5 == DMA1_FLAG_TE5) ? 1 : -1];

/**
 * @brief 由抽象通道号与事件类别组合出厂商标志/中断位值。
 * @param channel 抽象通道号。
 * @param event 事件类别，取 platform_ch32v203_dma_event_x。
 * @return 标志/中断位值；参数非法时返回 0。
 */
static uint32_t plat_ch32v203_dma_event_value(uint8_t channel, uint8_t event)
{
    if ((!plat_ch32v203_dma_is_valid(channel)) ||
        (event > (uint8_t)platform_ch32v203_dma_event_transfer_error)) {
        return 0U;
    }

    return (uint32_t)(1UL << ((((uint32_t)channel - 1U) * 4U) + (uint32_t)event));
}

/* ==========================================================================
 * 3) 函数封装（每个标准库调用一个薄封装）
 * ========================================================================== */

/**
 * @brief 使能 DMA1 时钟（AHB）。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_dma_clock_enable(void)
{
    PLAT_RCC_AHB_CLOCK_ENABLE(PLAT_RCC_AHB_DMA1);
    return true;
}

/**
 * @brief 取某外设某方向的 DMA 抽象通道号（CH32V203 DMA 请求映射固定）。
 * @param peripheral 请求方，取 platform_ch32v203_dma_peripheral_x。
 * @param is_tx true 取发送方向通道，false 取接收方向通道（ADC1 忽略该参数）。
 * @return 抽象通道号；无映射时返回 platform_ch32v203_dma_channel_none。
 */
static uint8_t plat_ch32v203_dma_channel_get(uint8_t peripheral, bool is_tx)
{
    switch (peripheral) {
    case (uint8_t)platform_ch32v203_dma_peripheral_adc1:
        /* ADC1 规则通道固定使用 DMA1_Channel1 */
        return (uint8_t)platform_ch32v203_dma_channel_1;
    case (uint8_t)platform_ch32v203_dma_peripheral_usart1:
        return is_tx ? (uint8_t)platform_ch32v203_dma_channel_4
                     : (uint8_t)platform_ch32v203_dma_channel_5;
    case (uint8_t)platform_ch32v203_dma_peripheral_usart2:
        return is_tx ? (uint8_t)platform_ch32v203_dma_channel_7
                     : (uint8_t)platform_ch32v203_dma_channel_6;
    case (uint8_t)platform_ch32v203_dma_peripheral_usart3:
        return is_tx ? (uint8_t)platform_ch32v203_dma_channel_2
                     : (uint8_t)platform_ch32v203_dma_channel_3;
    case (uint8_t)platform_ch32v203_dma_peripheral_i2c1:
        return is_tx ? (uint8_t)platform_ch32v203_dma_channel_6
                     : (uint8_t)platform_ch32v203_dma_channel_7;
    default:
        return (uint8_t)platform_ch32v203_dma_channel_none;
    }
}

/**
 * @brief 复位 DMA 通道（DMA_DeInit）。
 * @param channel 抽象通道号。
 * @retval true 操作成功。
 * @retval false 通道不受支持。
 */
static bool plat_ch32v203_dma_deinit(uint8_t channel)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);

    if (p_channel == NULL) {
        return false;
    }

    DMA_DeInit(p_channel);
    return true;
}

/**
 * @brief 装配 DMA 通道（DMA_Init）。
 * @param channel 抽象通道号。
 * @param p_config 可移植配置（对应 DMA_InitTypeDef）。
 * @retval true 操作成功。
 * @retval false 通道不受支持或配置为空。
 */
static bool plat_ch32v203_dma_config(uint8_t channel,
                                     const platform_ch32v203_dma_config_t *p_config)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);
    PLAT_DMA_INIT_TYPE init_config;

    if ((p_channel == NULL) || (p_config == NULL)) {
        return false;
    }

    init_config.DMA_PeripheralBaseAddr = (uint32_t)p_config->peripheral_address;
    init_config.DMA_MemoryBaseAddr = (uint32_t)p_config->memory_address;
    init_config.DMA_DIR = p_config->direction;
    init_config.DMA_BufferSize = (uint32_t)p_config->buffer_size;
    init_config.DMA_PeripheralInc = p_config->peripheral_increment;
    init_config.DMA_MemoryInc = p_config->memory_increment;
    init_config.DMA_PeripheralDataSize = p_config->peripheral_data_size;
    init_config.DMA_MemoryDataSize = p_config->memory_data_size;
    init_config.DMA_Mode = p_config->mode;
    init_config.DMA_Priority = p_config->priority;
    init_config.DMA_M2M = p_config->memory_to_memory;

    DMA_Init(p_channel, &init_config);
    return true;
}

/**
 * @brief 使能/禁止 DMA 通道（DMA_Cmd）。
 * @param channel 抽象通道号。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 通道不受支持。
 */
static bool plat_ch32v203_dma_cmd(uint8_t channel, bool enable)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);

    if (p_channel == NULL) {
        return false;
    }

    DMA_Cmd(p_channel, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 设置 DMA 存储器地址（重复传输前改缓冲区用）。
 * @note 标准库没有单独的"改存储器地址"接口，此处等价于参考实现
 *       drv_uart.c / drv_i2c.c 里直接写 MADDR 的做法。
 * @param channel 抽象通道号。
 * @param memory_address 目标存储器地址。
 * @retval true 操作成功。
 * @retval false 通道不受支持或地址为 0。
 */
static bool plat_ch32v203_dma_set_memory_address(uint8_t channel,
                                                 uintptr_t memory_address)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);

    if ((p_channel == NULL) || (memory_address == 0U)) {
        return false;
    }

    p_channel->MADDR = (uint32_t)memory_address;
    return true;
}

/**
 * @brief 设置待传输数据项个数（DMA_SetCurrDataCounter）。
 * @param channel 抽象通道号。
 * @param data_length 数据项个数（0 会被标准库写入，等价于停止搬运）。
 * @retval true 操作成功。
 * @retval false 通道不受支持。
 */
static bool plat_ch32v203_dma_set_data_length(uint8_t channel,
                                              uint16_t data_length)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);

    if (p_channel == NULL) {
        return false;
    }

    DMA_SetCurrDataCounter(p_channel, data_length);
    return true;
}

/**
 * @brief 读剩余待传输数据项个数（DMA_GetCurrDataCounter）。
 * @param channel 抽象通道号。
 * @return 剩余数据项个数；通道不受支持时返回 0。
 */
static uint16_t plat_ch32v203_dma_get_data_length(uint8_t channel)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);

    if (p_channel == NULL) {
        return 0U;
    }

    return DMA_GetCurrDataCounter(p_channel);
}

/**
 * @brief 查询 DMA 事件标志（DMA_GetFlagStatus）。
 * @param channel 抽象通道号。
 * @param event 事件类别，取 platform_ch32v203_dma_event_x。
 * @retval true 标志已置位。
 * @retval false 标志未置位或参数非法。
 */
static bool plat_ch32v203_dma_get_event(uint8_t channel, uint8_t event)
{
    uint32_t flag = plat_ch32v203_dma_event_value(channel, event);

    if (flag == 0U) {
        return false;
    }

    return (DMA_GetFlagStatus(flag) != PLAT_RESET);
}

/**
 * @brief 清除 DMA 事件标志（DMA_ClearFlag）。
 * @param channel 抽象通道号。
 * @param event 事件类别，取 platform_ch32v203_dma_event_x。
 * @retval true 操作成功。
 * @retval false 参数非法。
 */
static bool plat_ch32v203_dma_clear_event(uint8_t channel, uint8_t event)
{
    uint32_t flag = plat_ch32v203_dma_event_value(channel, event);

    if (flag == 0U) {
        return false;
    }

    DMA_ClearFlag(flag);
    return true;
}

/**
 * @brief 使能/禁止 DMA 事件中断（DMA_ITConfig）。
 * @param channel 抽象通道号。
 * @param event 事件类别，取 platform_ch32v203_dma_event_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 参数非法。
 */
static bool plat_ch32v203_dma_it_config(uint8_t channel, uint8_t event,
                                        bool enable)
{
    PLAT_DMA_CHANNEL_INSTANCE_TYPE *p_channel = plat_ch32v203_dma_channel_pointer(channel);
    uint32_t it = plat_ch32v203_dma_event_value(channel, event);

    if ((p_channel == NULL) || (it == 0U)) {
        return false;
    }

    DMA_ITConfig(p_channel, it, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 查询 DMA 事件中断状态（DMA_GetITStatus）。
 * @param channel 抽象通道号。
 * @param event 事件类别，取 platform_ch32v203_dma_event_x。
 * @retval true 中断挂起。
 * @retval false 无挂起或参数非法。
 */
static bool plat_ch32v203_dma_get_it_status(uint8_t channel, uint8_t event)
{
    uint32_t it = plat_ch32v203_dma_event_value(channel, event);

    if (it == 0U) {
        return false;
    }

    return (DMA_GetITStatus(it) != PLAT_RESET);
}

/**
 * @brief 清除 DMA 事件中断挂起位（DMA_ClearITPendingBit）。
 * @param channel 抽象通道号。
 * @param event 事件类别，取 platform_ch32v203_dma_event_x。
 * @retval true 操作成功。
 * @retval false 参数非法。
 */
static bool plat_ch32v203_dma_clear_it_pending(uint8_t channel, uint8_t event)
{
    uint32_t it = plat_ch32v203_dma_event_value(channel, event);

    if (it == 0U) {
        return false;
    }

    DMA_ClearITPendingBit(it);
    return true;
}

/**
 * @brief 配置 DMA 通道中断线（通道号 → DMA1_Channelx_IRQn 在本层完成映射）。
 * @param channel 抽象通道号。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval true 操作成功。
 * @retval false 通道不受支持。
 */
static bool plat_ch32v203_dma_irq_enable(uint8_t channel,
                                         uint8_t preemption_priority,
                                         uint8_t sub_priority)
{
    uint32_t irq_channel;

    if (!plat_ch32v203_dma_is_valid(channel)) {
        return false;
    }

    /* DMA1 的 7 条通道中断线在向量表中连续排列 */
    irq_channel = (uint32_t)DMA1_Channel1_IRQn + ((uint32_t)channel - 1U);

    PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority,
                   PLAT_ENABLE);
    return true;
}

/* ==========================================================================
 * 4) 平台层 DMA 函数表
 * ========================================================================== */

/** @brief 平台层 DMA 函数表（本文件私有，绑定本层对外接口）。 */
static const platform_ch32v203_dma_ops_t s_dma_ops = {
    .pf_clock_enable = plat_ch32v203_dma_clock_enable,
    .pf_channel_get = plat_ch32v203_dma_channel_get,
    .pf_deinit = plat_ch32v203_dma_deinit,
    .pf_config = plat_ch32v203_dma_config,
    .pf_cmd = plat_ch32v203_dma_cmd,
    .pf_set_memory_address = plat_ch32v203_dma_set_memory_address,
    .pf_set_data_length = plat_ch32v203_dma_set_data_length,
    .pf_get_data_length = plat_ch32v203_dma_get_data_length,
    .pf_get_event = plat_ch32v203_dma_get_event,
    .pf_clear_event = plat_ch32v203_dma_clear_event,
    .pf_it_config = plat_ch32v203_dma_it_config,
    .pf_get_it_status = plat_ch32v203_dma_get_it_status,
    .pf_clear_it_pending = plat_ch32v203_dma_clear_it_pending,
    .pf_irq_enable = plat_ch32v203_dma_irq_enable
};

/** @brief 平台层 DMA 函数表指针（对外外露，声明于 platform_ch32v203_dma.h）。 */
const platform_ch32v203_dma_ops_t *const plat_ch32v203_dma_ops = &s_dma_ops;
