/**
 * @file platform_ch32v203_can.c
 * @brief CH32V203 CAN 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 CAN 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_can_ops 函数表对外绑定；hal_can.c 通过该表访问 CAN 硬件。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；仅时钟使能沿用平台时钟宏
 *       PLAT_RCC_APB1_CLOCK_ENABLE，中断线配置沿用 PLAT_PFIC_INIT。
 *       厂商类型（CAN_TypeDef / CAN_InitTypeDef / CAN_FilterInitTypeDef /
 *       CanTxMsg / CanRxMsg）一律经平台头的"类型接入宏"接入：PLAT_CAN_INSTANCE_TYPE /
 *       PLAT_CAN_INIT_TYPE / PLAT_CAN_FILTER_INIT_TYPE / PLAT_CAN_TX_MESSAGE_TYPE /
 *       PLAT_CAN_RX_MESSAGE_TYPE，本文件内不再直接书写厂商类型；
 *       平台头文件对外只暴露纯 C 类型。
 * @note CH32V20x 标准库没有 CAN_Cmd：外设的"启动/停止"由 CAN_OperatingModeRequest
 *       表达（Normal = 启动并退出初始化/睡眠，Sleep = 停止），故本层提供
 *       pf_operating_mode_request 代替 CAN_Cmd。
 */
#include "platform_ch32v203_can.h"
#include <stddef.h>

/**
 * @brief 判断实例是否为受支持的 CAN 外设。
 * @param instance 实例基地址，取 PLAT_PERIPH_CAN1。
 * @retval true 受支持。
 * @retval false 其它外设。
 */
static bool plat_ch32v203_can_is_valid(uintptr_t instance)
{
    PLAT_CAN_INSTANCE_TYPE *p_can = (PLAT_CAN_INSTANCE_TYPE *)(instance);

    return (p_can == PLAT_PERIPH_CAN1);
}

/**
 * @brief 判断 FIFO 编号是否合法（CAN 接收 FIFO 0/1）。
 * @param fifo_number FIFO 编号，取 PLAT_CAN_FIFO_x。
 * @retval true 合法。
 * @retval false 非法。
 */
static bool plat_ch32v203_can_is_fifo_valid(uint8_t fifo_number)
{
    return (fifo_number == PLAT_CAN_FIFO_0) || (fifo_number == PLAT_CAN_FIFO_1);
}

/**
 * @brief 把平台报文转换为标准库发送报文（CanTxMsg，本文件内部使用）。
 * @param p_message 平台报文。
 * @param p_tx_message 输出：标准库发送报文。
 * @retval true 转换成功。
 * @retval false 参数为空、长度越界或标识符超出所声明帧类型的范围。
 */
static bool plat_ch32v203_can_message_to_tx(
    const platform_ch32v203_can_message_t *p_message,
    PLAT_CAN_TX_MESSAGE_TYPE *p_tx_message)
{
    uint8_t index;

    if ((p_message == NULL) || (p_tx_message == NULL) ||
        (p_message->data_length > PLAT_CAN_DATA_LENGTH_MAX)) {
        return false;
    }

    if (p_message->is_extended) {
        if (p_message->identifier > PLAT_CAN_EXT_ID_MAX) {
            return false;
        }
    } else if (p_message->identifier > PLAT_CAN_STD_ID_MAX) {
        return false;
    }

    p_tx_message->StdId = p_message->is_extended ? 0U : p_message->identifier;
    p_tx_message->ExtId = p_message->is_extended ? p_message->identifier : 0U;
    p_tx_message->IDE = p_message->is_extended ? CAN_Id_Extended : CAN_Id_Standard;
    p_tx_message->RTR = p_message->is_remote ? CAN_RTR_Remote : CAN_RTR_Data;
    p_tx_message->DLC = p_message->data_length;

    /* 超出 DLC 的字节补 0，避免把栈上残留数据写入邮箱 */
    for (index = 0U; index < PLAT_CAN_DATA_LENGTH_MAX; index++) {
        p_tx_message->Data[index] =
            (index < p_message->data_length) ? p_message->data[index] : 0U;
    }

    return true;
}

/**
 * @brief 把标准库接收报文（CanRxMsg，本文件内部使用）转换为平台报文。
 * @param p_rx_message 标准库接收报文。
 * @param p_message 输出：平台报文（超出 DLC 的数据字节清零）。
 */
static void plat_ch32v203_can_message_from_rx(
    const PLAT_CAN_RX_MESSAGE_TYPE *p_rx_message,
    platform_ch32v203_can_message_t *p_message)
{
    uint8_t index;

    p_message->is_extended = (p_rx_message->IDE == CAN_Id_Extended);
    p_message->is_remote = (p_rx_message->RTR == CAN_RTR_Remote);
    p_message->identifier = p_message->is_extended
                                ? p_rx_message->ExtId
                                : (p_rx_message->StdId & PLAT_CAN_STD_ID_MAX);
    p_message->data_length = (p_rx_message->DLC > PLAT_CAN_DATA_LENGTH_MAX)
                                 ? PLAT_CAN_DATA_LENGTH_MAX
                                 : p_rx_message->DLC;
    p_message->filter_index = p_rx_message->FMI;

    for (index = 0U; index < PLAT_CAN_DATA_LENGTH_MAX; index++) {
        p_message->data[index] = (index < p_message->data_length)
                                     ? p_rx_message->Data[index]
                                     : 0U;
    }
}

/**
 * @brief 使能 CAN 实例时钟（APB1）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_can_clock_enable(uintptr_t instance)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    PLAT_RCC_APB1_CLOCK_ENABLE(instance);

    return true;
}

/**
 * @brief 复位 CAN 外设（CAN_DeInit）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_can_deinit(uintptr_t instance)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    CAN_DeInit((PLAT_CAN_INSTANCE_TYPE *)(instance));
    return true;
}

/**
 * @brief 配置位时序与工作模式（CAN_Init）。
 * @param instance 实例基地址。
 * @param p_config 平台 CAN 配置。
 * @retval true CAN_Init 返回 PLAT_CAN_INIT_STATUS_SUCCESS。
 * @retval false 实例不受支持、配置为空或初始化模式切换失败。
 */
static bool plat_ch32v203_can_config(
    uintptr_t instance,
    const platform_ch32v203_can_config_t *p_config)
{
    PLAT_CAN_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_can_is_valid(instance))) {
        return false;
    }

    init_config.CAN_Prescaler = p_config->prescaler;
    init_config.CAN_Mode = p_config->mode;
    init_config.CAN_SJW = p_config->sync_jump_width;
    init_config.CAN_BS1 = p_config->bit_segment1;
    init_config.CAN_BS2 = p_config->bit_segment2;
    init_config.CAN_TTCM = p_config->is_time_trigger_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.CAN_ABOM = p_config->is_auto_bus_off_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.CAN_AWUM = p_config->is_auto_wakeup_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.CAN_NART =
        p_config->is_no_auto_retransmission_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.CAN_RFLM =
        p_config->is_receive_fifo_locked_enabled ? PLAT_ENABLE : PLAT_DISABLE;
    init_config.CAN_TXFP =
        p_config->is_transmit_fifo_priority_enabled ? PLAT_ENABLE : PLAT_DISABLE;

    return (CAN_Init((PLAT_CAN_INSTANCE_TYPE *)(instance), &init_config) ==
            PLAT_CAN_INIT_STATUS_SUCCESS);
}

/**
 * @brief 配置过滤器组（CAN_FilterInit）。
 * @note 过滤器为 CAN1/CAN2 共享资源，instance 仅用于实例合法性校验。
 * @param instance 实例基地址。
 * @param p_config 平台过滤器配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持、配置为空或过滤器组编号越界。
 */
static bool plat_ch32v203_can_filter_config(
    uintptr_t instance,
    const platform_ch32v203_can_filter_config_t *p_config)
{
    PLAT_CAN_FILTER_INIT_TYPE filter_config;

    if ((p_config == NULL) || (!plat_ch32v203_can_is_valid(instance)) ||
        (p_config->filter_number > PLAT_CAN_FILTER_NUMBER_MAX)) {
        return false;
    }

    filter_config.CAN_FilterNumber = (uint8_t)p_config->filter_number;
    filter_config.CAN_FilterMode = p_config->mode;
    filter_config.CAN_FilterScale = p_config->scale;
    filter_config.CAN_FilterIdHigh = p_config->id_high;
    filter_config.CAN_FilterIdLow = p_config->id_low;
    filter_config.CAN_FilterMaskIdHigh = p_config->mask_id_high;
    filter_config.CAN_FilterMaskIdLow = p_config->mask_id_low;
    filter_config.CAN_FilterFIFOAssignment = p_config->fifo_assignment;
    filter_config.CAN_FilterActivation =
        p_config->is_activated ? PLAT_ENABLE : PLAT_DISABLE;

    CAN_FilterInit(&filter_config);
    return true;
}

/**
 * @brief 请求工作模式（CAN_OperatingModeRequest，代替标准库缺失的 CAN_Cmd）。
 * @param instance 实例基地址。
 * @param operating_mode 目标模式，取 PLAT_CAN_OPERATING_MODE_x。
 * @retval true 返回 CAN_ModeStatus_Success。
 * @retval false 实例不受支持或模式切换失败。
 */
static bool plat_ch32v203_can_operating_mode_request(uintptr_t instance,
                                                     uint8_t operating_mode)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    return (CAN_OperatingModeRequest((PLAT_CAN_INSTANCE_TYPE *)(instance),
                                     operating_mode) == CAN_ModeStatus_Success);
}

/**
 * @brief 使能/禁止指定的 CAN 中断源（CAN_ITConfig）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_CAN_IT_x。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_can_it_config(uintptr_t instance, uint32_t it,
                                        bool enable)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    CAN_ITConfig((PLAT_CAN_INSTANCE_TYPE *)(instance), it, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 配置并使能 CAN 中断线（RX0 或 TX）。
 * @param irq_channel 中断号，取 PLAT_CAN_IRQ_RX0 / PLAT_CAN_IRQ_TX。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_can_irq_enable(uint32_t irq_channel,
                                         uint8_t preemption_priority,
                                         uint8_t sub_priority)
{
    PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority, PLAT_ENABLE);
    return true;
}

/**
 * @brief 请求发送一帧（CAN_Transmit）。
 * @param instance 实例基地址。
 * @param p_message 待发送的平台报文。
 * @return 邮箱号 0..2；无空闲邮箱时返回 PLAT_CAN_TX_STATUS_NO_MAILBOX；
 *         实例或报文非法时返回 PLAT_CAN_MAILBOX_NONE。
 */
static uint8_t plat_ch32v203_can_transmit(
    uintptr_t instance,
    const platform_ch32v203_can_message_t *p_message)
{
    PLAT_CAN_TX_MESSAGE_TYPE tx_message;

    if (!plat_ch32v203_can_is_valid(instance)) {
        return PLAT_CAN_MAILBOX_NONE;
    }

    if (!plat_ch32v203_can_message_to_tx(p_message, &tx_message)) {
        return PLAT_CAN_MAILBOX_NONE;
    }

    return CAN_Transmit((PLAT_CAN_INSTANCE_TYPE *)(instance), &tx_message);
}

/**
 * @brief 查询邮箱发送状态（CAN_TransmitStatus）。
 * @param instance 实例基地址。
 * @param mailbox 邮箱号 0..2。
 * @return PLAT_CAN_TX_STATUS_x；实例或邮箱号非法时返回 PLAT_CAN_TX_STATUS_FAILED。
 */
static uint8_t plat_ch32v203_can_transmit_status(uintptr_t instance,
                                                 uint8_t mailbox)
{
    if ((!plat_ch32v203_can_is_valid(instance)) ||
        (mailbox >= PLAT_CAN_MAILBOX_COUNT)) {
        return PLAT_CAN_TX_STATUS_FAILED;
    }

    return CAN_TransmitStatus((PLAT_CAN_INSTANCE_TYPE *)(instance), mailbox);
}

/**
 * @brief 取消指定邮箱的发送请求（CAN_CancelTransmit）。
 * @param instance 实例基地址。
 * @param mailbox 邮箱号 0..2。
 * @retval true 操作成功。
 * @retval false 实例或邮箱号非法。
 */
static bool plat_ch32v203_can_cancel_transmit(uintptr_t instance, uint8_t mailbox)
{
    if ((!plat_ch32v203_can_is_valid(instance)) ||
        (mailbox >= PLAT_CAN_MAILBOX_COUNT)) {
        return false;
    }

    CAN_CancelTransmit((PLAT_CAN_INSTANCE_TYPE *)(instance), mailbox);
    return true;
}

/**
 * @brief 从指定 FIFO 读取一帧（CAN_Receive，读取即释放 FIFO 空间）。
 * @param instance 实例基地址。
 * @param fifo_number FIFO 编号，取 PLAT_CAN_FIFO_x。
 * @param p_message 输出：平台报文。
 * @retval true 操作成功。
 * @retval false 实例/FIFO 非法或输出指针为空。
 */
static bool plat_ch32v203_can_receive(uintptr_t instance, uint8_t fifo_number,
                                      platform_ch32v203_can_message_t *p_message)
{
    PLAT_CAN_RX_MESSAGE_TYPE rx_message;

    if ((p_message == NULL) || (!plat_ch32v203_can_is_valid(instance)) ||
        (!plat_ch32v203_can_is_fifo_valid(fifo_number))) {
        return false;
    }

    CAN_Receive((PLAT_CAN_INSTANCE_TYPE *)(instance), fifo_number, &rx_message);
    plat_ch32v203_can_message_from_rx(&rx_message, p_message);

    return true;
}

/**
 * @brief 查询指定 FIFO 中待处理的报文数（CAN_MessagePending）。
 * @param instance 实例基地址。
 * @param fifo_number FIFO 编号，取 PLAT_CAN_FIFO_x。
 * @return 待处理报文数；实例或 FIFO 非法时返回 0。
 */
static uint8_t plat_ch32v203_can_message_pending(uintptr_t instance,
                                                 uint8_t fifo_number)
{
    if ((!plat_ch32v203_can_is_valid(instance)) ||
        (!plat_ch32v203_can_is_fifo_valid(fifo_number))) {
        return 0U;
    }

    return CAN_MessagePending((PLAT_CAN_INSTANCE_TYPE *)(instance), fifo_number);
}

/**
 * @brief 释放指定 FIFO（CAN_FIFORelease）。
 * @param instance 实例基地址。
 * @param fifo_number FIFO 编号，取 PLAT_CAN_FIFO_x。
 * @retval true 操作成功。
 * @retval false 实例或 FIFO 非法。
 */
static bool plat_ch32v203_can_fifo_release(uintptr_t instance,
                                           uint8_t fifo_number)
{
    if ((!plat_ch32v203_can_is_valid(instance)) ||
        (!plat_ch32v203_can_is_fifo_valid(fifo_number))) {
        return false;
    }

    CAN_FIFORelease((PLAT_CAN_INSTANCE_TYPE *)(instance), fifo_number);
    return true;
}

/**
 * @brief 查询状态标志（CAN_GetFlagStatus）。
 * @param instance 实例基地址。
 * @param flag 标志，取 PLAT_CAN_FLAG_x。
 * @retval true 标志置位。
 * @retval false 标志复位或实例非法。
 */
static bool plat_ch32v203_can_get_flag(uintptr_t instance, uint32_t flag)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    return (CAN_GetFlagStatus((PLAT_CAN_INSTANCE_TYPE *)(instance), flag) != PLAT_RESET);
}

/**
 * @brief 清除状态标志（CAN_ClearFlag）。
 * @param instance 实例基地址。
 * @param flag 标志，取 PLAT_CAN_FLAG_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_can_clear_flag(uintptr_t instance, uint32_t flag)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    CAN_ClearFlag((PLAT_CAN_INSTANCE_TYPE *)(instance), flag);
    return true;
}

/**
 * @brief 查询中断状态（CAN_GetITStatus）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_CAN_IT_x。
 * @retval true 中断挂起。
 * @retval false 中断未挂起或实例非法。
 */
static bool plat_ch32v203_can_get_it_status(uintptr_t instance, uint32_t it)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    return (CAN_GetITStatus((PLAT_CAN_INSTANCE_TYPE *)(instance), it) != PLAT_RESET);
}

/**
 * @brief 清除中断挂起位（CAN_ClearITPendingBit）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_CAN_IT_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_can_clear_it_pending_bit(uintptr_t instance,
                                                   uint32_t it)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return false;
    }

    CAN_ClearITPendingBit((PLAT_CAN_INSTANCE_TYPE *)(instance), it);
    return true;
}

/**
 * @brief 读最近一次错误码（CAN_GetLastErrorCode）。
 * @param instance 实例基地址。
 * @return 错误码；实例非法时返回 0。
 */
static uint8_t plat_ch32v203_can_get_last_error_code(uintptr_t instance)
{
    if (!plat_ch32v203_can_is_valid(instance)) {
        return 0U;
    }

    return CAN_GetLastErrorCode((PLAT_CAN_INSTANCE_TYPE *)(instance));
}

/** @brief CH32V203 CAN 操作函数表（本文件私有）。 */
static const platform_ch32v203_can_ops_t s_can_ops = {
    .pf_clock_enable = plat_ch32v203_can_clock_enable,
    .pf_deinit = plat_ch32v203_can_deinit,
    .pf_config = plat_ch32v203_can_config,
    .pf_filter_config = plat_ch32v203_can_filter_config,
    .pf_operating_mode_request = plat_ch32v203_can_operating_mode_request,
    .pf_it_config = plat_ch32v203_can_it_config,
    .pf_irq_enable = plat_ch32v203_can_irq_enable,
    .pf_transmit = plat_ch32v203_can_transmit,
    .pf_transmit_status = plat_ch32v203_can_transmit_status,
    .pf_cancel_transmit = plat_ch32v203_can_cancel_transmit,
    .pf_receive = plat_ch32v203_can_receive,
    .pf_message_pending = plat_ch32v203_can_message_pending,
    .pf_fifo_release = plat_ch32v203_can_fifo_release,
    .pf_get_flag = plat_ch32v203_can_get_flag,
    .pf_clear_flag = plat_ch32v203_can_clear_flag,
    .pf_get_it_status = plat_ch32v203_can_get_it_status,
    .pf_clear_it_pending_bit = plat_ch32v203_can_clear_it_pending_bit,
    .pf_get_last_error_code = plat_ch32v203_can_get_last_error_code
};

/** @brief CH32V203 CAN 操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_can_ops_t *const plat_ch32v203_can_ops = &s_can_ops;
