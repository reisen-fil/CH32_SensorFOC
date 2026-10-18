/**
 * @file platform_ch32v203_uart.c
 * @brief CH32V203 UART 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 USART 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_uart_ops 函数表对外绑定；hal_uart.c 通过该表访问 UART 硬件。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；仅时钟使能沿用平台时钟宏
 *       PLAT_RCC_APB1_CLOCK_ENABLE / PLAT_RCC_APB2_CLOCK_ENABLE，
 *       中断线配置沿用 PLAT_PFIC_INIT。
 * @note 收发通道的 DMA 配置与状态（装配、使能、长度、传输完成标志、优先级）
 *       已移入 platform_ch32v203_dma.c（plat_ch32v203_dma_ops），通道映射由该层
 *       pf_channel_get() 统一给出；本文件只保留"外设侧"的 DMA 相关操作，即
 *       USART_DMACmd（pf_dma_request_cmd）与 USART 数据寄存器地址查询
 *       （pf_get_data_register_address），故函数表既不暴露通道号也不暴露优先级。
 * @note 中断线映射仍按实例在本文件内部 fixed 确定（USART1/2/3 → USART1/2/3_IRQn）。
 * @note USART3 使用默认引脚映射（PB10/PB11），因此本层不做任何重映射动作。
 */
#include "platform_ch32v203_uart.h"
#include <stddef.h>

/**
 * @brief 判断实例是否为受支持的 UART 外设。
 * @param instance 实例基地址，取 PLAT_PERIPH_USART1/2/3。
 * @retval true 受支持。
 * @retval false 其它外设。
 */
static bool plat_ch32v203_uart_is_valid(uintptr_t instance)
{
    PLAT_UART_INSTANCE_TYPE *p_usart = (PLAT_UART_INSTANCE_TYPE *)(instance);

    return (p_usart == PLAT_PERIPH_USART1) || (p_usart == PLAT_PERIPH_USART2) ||
           (p_usart == PLAT_PERIPH_USART3);
}

/**
 * @brief 取 UART 的中断线号。
 * @param instance 实例基地址。
 * @return 中断号（USART1/2/3_IRQn）；实例不受支持时返回 0。
 */
static uint8_t plat_ch32v203_uart_irq_channel(uintptr_t instance)
{
    if ((PLAT_UART_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_USART1) {
        return (uint8_t)PLAT_IRQ_USART1;
    }
    if ((PLAT_UART_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_USART2) {
        return (uint8_t)PLAT_IRQ_USART2;
    }
    if ((PLAT_UART_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_USART3) {
        return (uint8_t)PLAT_IRQ_USART3;
    }

    return 0U;
}

/**
 * @brief 使能 UART 实例所在总线的外设时钟。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_clock_enable(uintptr_t instance)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    if ((PLAT_UART_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_USART1) {
        PLAT_RCC_APB2_CLOCK_ENABLE(PLAT_RCC_APB2_USART1);
    } else if ((PLAT_UART_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_USART2) {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_USART2);
    } else {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_USART3);
    }

    return true;
}

/**
 * @brief 复位 UART 外设（USART_DeInit）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_deinit(uintptr_t instance)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    USART_DeInit((PLAT_UART_INSTANCE_TYPE *)(instance));
    return true;
}

/**
 * @brief 配置 UART 帧格式与波特率（USART_Init）。
 * @param instance 实例基地址。
 * @param p_config 平台 UART 配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持或配置为空。
 */
static bool plat_ch32v203_uart_config(
    uintptr_t instance,
    const platform_ch32v203_uart_config_t *p_config)
{
    PLAT_UART_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_uart_is_valid(instance))) {
        return false;
    }

    init_config.USART_BaudRate = p_config->baud_rate;
    init_config.USART_WordLength = p_config->word_length;
    init_config.USART_StopBits = p_config->stop_bits;
    init_config.USART_Parity = p_config->parity;
    init_config.USART_Mode = p_config->mode;
    init_config.USART_HardwareFlowControl = p_config->flow_control;

    USART_Init((PLAT_UART_INSTANCE_TYPE *)(instance), &init_config);
    return true;
}

/**
 * @brief 使能/禁止 UART（USART_Cmd）。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    USART_Cmd((PLAT_UART_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止指定的 UART 中断源（USART_ITConfig）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_UART_IT_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_it_config(uintptr_t instance, uint16_t it,
                                         bool enable)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    USART_ITConfig((PLAT_UART_INSTANCE_TYPE *)(instance), it, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 配置并使能 UART 的中断线（PLAT_PFIC_INIT）。
 * @param instance 实例基地址。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_irq_enable(uintptr_t instance,
                                          uint8_t preemption_priority,
                                          uint8_t sub_priority)
{
    uint8_t irq_channel = plat_ch32v203_uart_irq_channel(instance);

    if (irq_channel == 0U) {
        return false;
    }

    PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority, PLAT_ENABLE);
    return true;
}

/**
 * @brief 使能/禁止 UART 的 DMA 传输请求（USART_DMACmd）。
 * @param instance 实例基地址。
 * @param request 传输请求，取 PLAT_UART_TRANSFER_REQUEST_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_dma_request_cmd(uintptr_t instance,
                                               uint16_t request, bool enable)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    USART_DMACmd((PLAT_UART_INSTANCE_TYPE *)(instance), request,
                 enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 取 UART 数据寄存器（DATAR）地址。
 * @details 取址本身为只读操作，不改动任何寄存器；上层用它拼通用 DMA 配置的
 *          外设侧地址（收发方向共用同一数据寄存器）。
 * @param instance 实例基地址。
 * @return 数据寄存器地址；实例不受支持时返回 0。
 */
static uintptr_t plat_ch32v203_uart_get_data_register_address(uintptr_t instance)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return 0U;
    }

    return (uintptr_t)(&((PLAT_UART_INSTANCE_TYPE *)(instance))->DATAR);
}

/**
 * @brief 查询 UART 状态标志（USART_GetFlagStatus）。
 * @param instance 实例基地址。
 * @param flag 标志，取 PLAT_UART_FLAG_x。
 * @retval true 标志置位。
 * @retval false 标志复位或实例不受支持。
 */
static bool plat_ch32v203_uart_get_flag(uintptr_t instance, uint16_t flag)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    return (USART_GetFlagStatus((PLAT_UART_INSTANCE_TYPE *)(instance), flag) != PLAT_RESET);
}

/**
 * @brief 清除 UART 状态标志（USART_ClearFlag）。
 * @param instance 实例基地址。
 * @param flag 标志，取 PLAT_UART_FLAG_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_clear_flag(uintptr_t instance, uint16_t flag)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    USART_ClearFlag((PLAT_UART_INSTANCE_TYPE *)(instance), flag);
    return true;
}

/**
 * @brief 查询 UART 中断状态（USART_GetITStatus）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_UART_IT_x。
 * @retval true 中断挂起。
 * @retval false 中断未挂起或实例不受支持。
 */
static bool plat_ch32v203_uart_get_it_status(uintptr_t instance, uint16_t it)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    return (USART_GetITStatus((PLAT_UART_INSTANCE_TYPE *)(instance), it) != PLAT_RESET);
}

/**
 * @brief 清除 IDLE 空闲中断标志。
 * @details 顺序固定为"先读状态寄存器再读数据寄存器"：USART_GetFlagStatus 读 STATR，
 *          USART_ReceiveData 读 DATAR，两者合起来即参考实现 usart3_dma_handle()
 *          中的清标志动作（先读 SR 再读 DR），全程只读、不引入寄存器直写。
 * @param instance 实例基地址。
 */
static void plat_ch32v203_uart_clear_idle(uintptr_t instance)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return;
    }

    (void)USART_GetFlagStatus((PLAT_UART_INSTANCE_TYPE *)(instance), USART_FLAG_IDLE);
    (void)USART_ReceiveData((PLAT_UART_INSTANCE_TYPE *)(instance));
}

/**
 * @brief 发送一个帧（USART_SendData）。
 * @param instance 实例基地址。
 * @param data 待发送数据。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_uart_send_data(uintptr_t instance, uint16_t data)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return false;
    }

    USART_SendData((PLAT_UART_INSTANCE_TYPE *)(instance), data);
    return true;
}

/**
 * @brief 接收一个帧（USART_ReceiveData）。
 * @param instance 实例基地址。
 * @return 接收到的数据；实例不受支持时返回 0。
 */
static uint16_t plat_ch32v203_uart_receive_data(uintptr_t instance)
{
    if (!plat_ch32v203_uart_is_valid(instance)) {
        return 0U;
    }

    return USART_ReceiveData((PLAT_UART_INSTANCE_TYPE *)(instance));
}

/** @brief CH32V203 UART 操作函数表（本文件私有）。 */
static const platform_ch32v203_uart_ops_t s_uart_ops = {
    .pf_clock_enable = plat_ch32v203_uart_clock_enable,
    .pf_deinit = plat_ch32v203_uart_deinit,
    .pf_config = plat_ch32v203_uart_config,
    .pf_cmd = plat_ch32v203_uart_cmd,
    .pf_it_config = plat_ch32v203_uart_it_config,
    .pf_irq_enable = plat_ch32v203_uart_irq_enable,
    .pf_dma_request_cmd = plat_ch32v203_uart_dma_request_cmd,
    .pf_get_data_register_address =
        plat_ch32v203_uart_get_data_register_address,
    .pf_get_flag = plat_ch32v203_uart_get_flag,
    .pf_clear_flag = plat_ch32v203_uart_clear_flag,
    .pf_get_it_status = plat_ch32v203_uart_get_it_status,
    .pf_clear_idle = plat_ch32v203_uart_clear_idle,
    .pf_send_data = plat_ch32v203_uart_send_data,
    .pf_receive_data = plat_ch32v203_uart_receive_data
};

/** @brief CH32V203 UART 操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_uart_ops_t *const plat_ch32v203_uart_ops = &s_uart_ops;
