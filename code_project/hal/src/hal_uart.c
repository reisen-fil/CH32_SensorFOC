/**
 * @file hal_uart.c
 * @brief UART 对象操作 + CH32V203 板级实例（VOFA 串口）+ IDLE 收数据 + HAL 函数表。
 * @details 硬件访问统一经平台 UART 操作函数表指针 plat_ch32v203_uart_ops 与平台
 *          通用 DMA 操作函数表指针 plat_ch32v203_dma_ops 完成，引脚组构建经 HAL
 *          GPIO 函数表指针 hal_gpio_ops 完成，本文件不含厂商库符号。
 *          板级配置依据参考实现 drv_uart.c（Drv_USART_Init / USART_DMA_Send /
 *          usart3_dma_handle 前半段）：
 *          - 外设：USART3，波特率 4000000，8 位字长、1 停止位、无校验、无流控、
 *            Tx|Rx（drv_uart.c:75-81, 109）；
 *          - 引脚：PB10(TX, AF_PP) / PB11(RX, 浮空输入)（drv_uart.c:65-72），
 *            即本工程 hal_gpio_usart_pin_table，USART3 默认映射无需重映射；
 *          - 接收 DMA：USART3 接收方向的通用 DMA 抽象通道（通道号由
 *            plat_ch32v203_dma_ops->pf_channel_get(..., false) 给出），外设→内存，
 *            字节宽度，循环模式，VeryHigh，装配后常开（drv_uart.c:37-50, 62）；
 *            目的缓冲由上层经 hal_uart_bind_rx_buffer() 提供，缓冲长度即环形队列长度；
 *          - 发送 DMA：USART3 发送方向的通用 DMA 抽象通道（同表 is_tx=true），
 *            内存→外设，字节宽度，普通模式，Medium，在 hal_uart_init() 中装配
 *            （存储器地址与传输长度置 0），逐次发送只写这两个寄存器并使能/关闭通道
 *            （drv_uart.c:52-60, 118-157）；
 *          - 传输请求：Tx 与 Rx 均使能（drv_uart.c:99-100）；
 *          - IDLE 中断：参考实现抢占优先级 1 / 子优先级 0（drv_uart.c:84-88, 90），
 *            本实现默认不使能中断线，需由应用层实现 USART3_IRQHandler 后显式打开。
 * @note 参考实现的帧格式没有帧头/长度/校验字段：帧边界由 USART 空闲（IDLE）中断
 *       界定，帧内容为 VOFA 命令 ASCII 字符串；本层据实照搬，不额外发明协议字段。
 * @note 本层只承担"空闲中断接收部分"：清 IDLE 标志 + 由接收 DMA 剩余计数算出当前
 *       写位置，再把写位置经回调上报上层；环形队列写入与整帧提取在 BSP 层通信链路
 *       模块（bsp_comm_mgr.c）中实现。
 */
#include "hal_uart.h"
#include "hal_gpio.h"
#include "platform_ch32v203_dma.h"

#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * 板级配置：通用 USART 外设（当前板级用途：VOFA 串口 USART3）
 * ========================================================================== */

/** @brief 板级通用 USART 外设配置（当前板级用途：VOFA 串口；取值来源：参考实现 drv_uart.c）。 */
static const hal_uart_config_t s_usart_config = {
    .baud_rate = 2000000U,                                  /* drv_uart.c:109 */
    .word_length = PLAT_UART_WORD_LENGTH_8B,                /* drv_uart.c:76 */
    .stop_bits = PLAT_UART_STOP_BITS_1,                     /* drv_uart.c:77 */
    .parity = PLAT_UART_PARITY_NONE,                        /* drv_uart.c:78 */
    .mode = PLAT_UART_MODE_TX | PLAT_UART_MODE_RX,          /* drv_uart.c:80 */
    .flow_control = PLAT_UART_FLOW_CONTROL_NONE,            /* drv_uart.c:79 */
    .timeout_count = HAL_UART_DMA_TIMEOUT_COUNT,
    .p_user_data = NULL
};

/** @brief IDLE 空闲中断线（PLAT_IRQ_USART3）优先级：抢占 1 / 子 0。
 *  @note 与 hal_adc.c 的注入组中断采用同一配置方式：优先级在本文件以板级常量给出，
 *        中断线在 hal_uart_init() 中配置并使能（此时中断源未打开，不会触发中断）。
 *        本工程标准库 NVIC_Init() 只区分抢占 0/1 两档，采样控制取 0，本路取 1，
 *        保证串口空闲中断不会打断电流环采样中断。 */
#define HAL_UART_IDLE_IRQ_PREEMPTION_PRIORITY      (1U)
#define HAL_UART_IDLE_IRQ_SUB_PRIORITY             (0U)

/** @brief VOFA 串口实例（USART3 + DMA 收发）。 */
hal_uart_t hal_uart_comm = {0};

/* ==========================================================================
 * 对象操作
 * ========================================================================== */

/**
 * @brief 检查UART对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（实例基地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_uart_is_object_valid(const hal_uart_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 取板级 VOFA 串口（USART3）某收发方向的通用 DMA 抽象通道号。
 * @details 通道映射是片内 DMA 请求表的事实，统一由平台 DMA 层给出，
 *          本层只知道"哪个外设、哪个方向"，不出现任何通道号常量。
 * @param is_tx true 取发送方向通道，false 取接收方向通道。
 * @return 抽象通道号 platform_ch32v203_dma_channel_x。
 */
static uint8_t hal_uart_dma_channel(bool is_tx)
{
    return plat_ch32v203_dma_ops->pf_channel_get(
        (uint8_t)platform_ch32v203_dma_peripheral_usart3, is_tx);
}

/**
 * @brief 绑定UART对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址。
 * @param p_config 模块初始化配置指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效或必需指针为空。
 */
hal_uart_status_enum hal_uart_bind(hal_uart_t *p_object, uintptr_t base_address,
                                   const hal_uart_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) || (base_address == 0U)) {
        return hal_uart_err_param;
    }

    p_object->state = hal_uart_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;
    /* 接收缓冲与回调须由 hal_uart_bind_rx_buffer() / hal_uart_bind_rx_callback()
       重新绑定，不需要接收时可不绑定 */
    p_object->p_rx_buffer = NULL;
    p_object->rx_buffer_size = 0U;
    p_object->pf_rx_callback = NULL;
    p_object->p_rx_context = NULL;

    return hal_uart_ok;
}

/**
 * @brief 绑定接收 DMA 缓冲（须在 hal_uart_init() 之前调用）。
 * @param p_object 模块对象指针。
 * @param p_rx_buffer 接收缓冲首地址。
 * @param buffer_size 缓冲字节数。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效或对象未绑定。
 */
hal_uart_status_enum hal_uart_bind_rx_buffer(hal_uart_t *p_object,
                                             uint8_t *p_rx_buffer,
                                             uint16_t buffer_size)
{
    if ((!hal_uart_is_object_valid(p_object)) || (p_rx_buffer == NULL) ||
        (buffer_size == 0U)) {
        return hal_uart_err_param;
    }

    p_object->p_rx_buffer = p_rx_buffer;
    p_object->rx_buffer_size = buffer_size;

    return hal_uart_ok;
}

/**
 * @brief 绑定接收通知回调（接收 DMA 写位置更新时通知上层）。
 * @param p_object 模块对象指针。
 * @param pf_callback 回调函数指针。
 * @param p_context 回调上下文。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效或对象未绑定。
 */
hal_uart_status_enum hal_uart_bind_rx_callback(
    hal_uart_t *p_object,
    hal_uart_rx_callback_t pf_callback,
    void *p_context)
{
    if ((!hal_uart_is_object_valid(p_object)) || (pf_callback == NULL)) {
        return hal_uart_err_param;
    }

    p_object->pf_rx_callback = pf_callback;
    p_object->p_rx_context = p_context;

    return hal_uart_ok;
}

/**
 * @brief 初始化UART：时钟 + 复位 + 帧格式 + 收发DMA装配 + 传输请求 + 使能。
 * @param p_object 模块对象指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 对象未绑定或实例不受支持。
 * @retval hal_uart_err_state 平台层装配失败。
 */
hal_uart_status_enum hal_uart_init(hal_uart_t *p_object)
{
    platform_ch32v203_uart_config_t platform_config;
    platform_ch32v203_dma_config_t rx_dma_config;
    platform_ch32v203_dma_config_t tx_dma_config;
    uintptr_t instance;
    uintptr_t data_register;
    uint8_t rx_channel;
    uint8_t tx_channel;

    if (!hal_uart_is_object_valid(p_object)) {
        return hal_uart_err_param;
    }

    instance = p_object->base_address;

    platform_config.baud_rate = p_object->config.baud_rate;
    platform_config.word_length = p_object->config.word_length;
    platform_config.stop_bits = p_object->config.stop_bits;
    platform_config.parity = p_object->config.parity;
    platform_config.flow_control = p_object->config.flow_control;
    platform_config.mode = p_object->config.mode;

    if ((!plat_ch32v203_uart_ops->pf_clock_enable(instance)) ||
        (!plat_ch32v203_dma_ops->pf_clock_enable()) ||
        (!plat_ch32v203_uart_ops->pf_deinit(instance)) ||
        (!plat_ch32v203_uart_ops->pf_config(instance, &platform_config))) {
        p_object->state = hal_uart_state_error;
        return hal_uart_err_param;
    }

    /* 收发 DMA 的数据寄存器地址（发送/接收装配都要用），此处统一取得 */
    data_register =
        plat_ch32v203_uart_ops->pf_get_data_register_address(instance);
    if (data_register == 0U) {
        p_object->state = hal_uart_state_error;
        return hal_uart_err_state;
    }

    /* 接收 DMA：循环模式，目的缓冲由上层提供；装配后常开，由 IDLE 中断上报写位置 */
    if (p_object->p_rx_buffer != NULL) {
        rx_channel = hal_uart_dma_channel(false);

        rx_dma_config.peripheral_address = data_register;
        rx_dma_config.memory_address = (uintptr_t)p_object->p_rx_buffer;
        rx_dma_config.direction = PLAT_DMA_DIR_PERIPHERAL_TO_MEMORY;
        rx_dma_config.buffer_size = p_object->rx_buffer_size;
        rx_dma_config.peripheral_increment =
            PLAT_DMA_PERIPHERAL_INCREMENT_DISABLE;
        rx_dma_config.memory_increment = PLAT_DMA_MEMORY_INCREMENT_ENABLE;
        rx_dma_config.peripheral_data_size = PLAT_DMA_PERIPHERAL_DATA_SIZE_BYTE;
        rx_dma_config.memory_data_size = PLAT_DMA_MEMORY_DATA_SIZE_BYTE;
        rx_dma_config.mode = PLAT_DMA_MODE_CIRCULAR;
        rx_dma_config.priority = PLAT_DMA_PRIORITY_VERY_HIGH;
        rx_dma_config.memory_to_memory = PLAT_DMA_MEMORY_TO_MEMORY_DISABLE;

        if ((!plat_ch32v203_dma_ops->pf_deinit(rx_channel)) ||
            (!plat_ch32v203_dma_ops->pf_config(rx_channel, &rx_dma_config)) ||
            (!plat_ch32v203_dma_ops->pf_cmd(rx_channel, true))) {
            p_object->state = hal_uart_state_error;
            return hal_uart_err_state;
        }
    }

    /* 发送 DMA：普通模式，对应参考实现 drv_USART_DMA_init() 中"先装配、发送时再写
     * 地址与长度"的做法（drv_uart.c:52-60）。除存储器地址与传输长度之外的参数在此
     * 一次配好；这两个寄存器是逐次发送的动态值，装配时置 0，由 hal_uart_send() 写入。
     * **不使能通道**：使能与关闭都在 hal_uart_send() 中完成。 */
    tx_channel = hal_uart_dma_channel(true);

    tx_dma_config.peripheral_address = data_register;
    tx_dma_config.memory_address = 0U;
    tx_dma_config.direction = PLAT_DMA_DIR_MEMORY_TO_PERIPHERAL;
    tx_dma_config.buffer_size = 0U;
    tx_dma_config.peripheral_increment = PLAT_DMA_PERIPHERAL_INCREMENT_DISABLE;
    tx_dma_config.memory_increment = PLAT_DMA_MEMORY_INCREMENT_ENABLE;
    tx_dma_config.peripheral_data_size = PLAT_DMA_PERIPHERAL_DATA_SIZE_BYTE;
    tx_dma_config.memory_data_size = PLAT_DMA_MEMORY_DATA_SIZE_BYTE;
    tx_dma_config.mode = PLAT_DMA_MODE_NORMAL;
    tx_dma_config.priority = PLAT_DMA_PRIORITY_MEDIUM;
    tx_dma_config.memory_to_memory = PLAT_DMA_MEMORY_TO_MEMORY_DISABLE;

    if ((!plat_ch32v203_dma_ops->pf_deinit(tx_channel)) ||
        (!plat_ch32v203_dma_ops->pf_config(tx_channel, &tx_dma_config))) {
        p_object->state = hal_uart_state_error;
        return hal_uart_err_state;
    }

    /* 收发 DMA 请求与 USART 使能（发送通道配置已就绪，逐次发送只写地址/长度并使能） */
    if ((!plat_ch32v203_uart_ops->pf_dma_request_cmd(
            instance, PLAT_UART_TRANSFER_REQUEST_TX, true)) ||
        (!plat_ch32v203_uart_ops->pf_dma_request_cmd(
            instance, PLAT_UART_TRANSFER_REQUEST_RX, true)) ||
        (!plat_ch32v203_uart_ops->pf_cmd(instance, true))) {
        p_object->state = hal_uart_state_error;
        return hal_uart_err_state;
    }

    /* IDLE 中断分两层处理（与 hal_adc.c 注入组一致）：
     * - 中断线（PLAT_IRQ_USART3）：构建时即按本文件板级常量配置并使能，
     *   此时中断源未打开，不会触发中断；
     * - 中断源（IDLE）：默认不使能，应用层实现 USART3_IRQHandler 之后经
     *   hal_uart_ops->pf_enable_idle_irq() 按需打开。 */
    if (!plat_ch32v203_uart_ops->pf_irq_enable(
            instance, HAL_UART_IDLE_IRQ_PREEMPTION_PRIORITY,
            HAL_UART_IDLE_IRQ_SUB_PRIORITY)) {
        p_object->state = hal_uart_state_error;
        return hal_uart_err_state;
    }

    p_object->state = hal_uart_state_enabled;
    return hal_uart_ok;
}

/**
 * @brief 关闭并复位UART外设（含收发 DMA 通道与 IDLE 中断源）。
 * @param p_object 模块对象指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 对象未绑定或实例不受支持。
 * @retval hal_uart_err_state 平台层关闭失败。
 */
hal_uart_status_enum hal_uart_deinit(hal_uart_t *p_object)
{
    uintptr_t instance;

    if (!hal_uart_is_object_valid(p_object)) {
        return hal_uart_err_param;
    }

    instance = p_object->base_address;

    (void)plat_ch32v203_uart_ops->pf_it_config(instance, PLAT_UART_IT_IDLE, false);
    (void)plat_ch32v203_dma_ops->pf_cmd(hal_uart_dma_channel(true), false);
    (void)plat_ch32v203_dma_ops->pf_cmd(hal_uart_dma_channel(false), false);

    if ((!plat_ch32v203_uart_ops->pf_cmd(instance, false)) ||
        (!plat_ch32v203_uart_ops->pf_deinit(instance))) {
        return hal_uart_err_state;
    }

    p_object->state = hal_uart_state_reset;
    return hal_uart_ok;
}

/**
 * @brief 使能UART收发。
 * @param p_object 模块对象指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 对象未绑定。
 * @retval hal_uart_err_state 平台层拒绝。
 */
hal_uart_status_enum hal_uart_enable(hal_uart_t *p_object)
{
    if (!hal_uart_is_object_valid(p_object)) {
        return hal_uart_err_param;
    }

    if (!plat_ch32v203_uart_ops->pf_cmd(p_object->base_address, true)) {
        return hal_uart_err_state;
    }

    p_object->state = hal_uart_state_enabled;
    return hal_uart_ok;
}

/**
 * @brief 禁用UART收发。
 * @param p_object 模块对象指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 对象未绑定。
 * @retval hal_uart_err_state 平台层拒绝。
 */
hal_uart_status_enum hal_uart_disable(hal_uart_t *p_object)
{
    if (!hal_uart_is_object_valid(p_object)) {
        return hal_uart_err_param;
    }

    if (!plat_ch32v203_uart_ops->pf_cmd(p_object->base_address, false)) {
        return hal_uart_err_state;
    }

    p_object->state = hal_uart_state_disabled;
    return hal_uart_ok;
}

/**
 * @brief DMA 阻塞式发送一段数据。
 * @details 思路对应参考实现 USART_DMA_Send()（drv_uart.c:118-157）：
 *          写本次传输的存储器地址与数据长度 → 清传输完成标志 → 使能通道
 *          → 等剩余计数归零 → 等传输完成标志 → 清标志并关通道 → 等 USART TC。
 *          通道的方向/宽度/模式/优先级等参数已在 hal_uart_init() 中装配好，
 *          本函数只写"存储器地址 + 数据长度"这两个逐次变化的寄存器。
 * @param p_object 模块对象指针。
 * @param p_tx_buf 待发送数据缓冲区（本次传输的存储器地址）。
 * @param data_length 待发送字节数（本次传输的数据长度）。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_state 对象未使能或平台层写入失败。
 * @retval hal_uart_err_timeout 等待 DMA 剩余计数/传输完成标志/USART TC 超时。
 * @note 参考实现为死等；本实现按 config.timeout_count 加超时保护。
 */
hal_uart_status_enum hal_uart_send(hal_uart_t *p_object, const uint8_t *p_tx_buf,
                                   uint16_t data_length)
{
    uintptr_t instance;
    uint32_t timeout;
    uint8_t tx_channel;

    if ((!hal_uart_is_object_valid(p_object)) || (p_tx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_uart_err_param;
    }
    if (p_object->state != hal_uart_state_enabled) {
        return hal_uart_err_state;
    }

    instance = p_object->base_address;
    tx_channel = hal_uart_dma_channel(true);

    /* 1. 写本次传输的两个动态寄存器：存储器地址与数据长度
     *    （对应参考实现直写 DMA1_Channel2->MADDR / CNTR，drv_uart.c:123-124；
     *      通道其余参数已在 hal_uart_init() 装配完毕） */
    if ((!plat_ch32v203_dma_ops->pf_set_memory_address(
            tx_channel, (uintptr_t)p_tx_buf)) ||
        (!plat_ch32v203_dma_ops->pf_set_data_length(tx_channel, data_length))) {
        return hal_uart_err_state;
    }

    /* 2. 清传输完成事件标志 */
    if (!plat_ch32v203_dma_ops->pf_clear_event(
            tx_channel,
            (uint8_t)platform_ch32v203_dma_event_transfer_complete)) {
        return hal_uart_err_state;
    }

    /* 3. 使能发送 DMA 通道 */
    if (!plat_ch32v203_dma_ops->pf_cmd(tx_channel, true)) {
        return hal_uart_err_state;
    }

    /* 4. 轮询等待剩余计数归零 */
    timeout = p_object->config.timeout_count;
    while (plat_ch32v203_dma_ops->pf_get_data_length(tx_channel) != 0U) {
        if (timeout-- == 0U) {
            (void)plat_ch32v203_dma_ops->pf_cmd(tx_channel, false);
            return hal_uart_err_timeout;
        }
    }

    /* 5. 轮询等待传输完成事件置位 */
    timeout = p_object->config.timeout_count;
    while (!plat_ch32v203_dma_ops->pf_get_event(
        tx_channel,
        (uint8_t)platform_ch32v203_dma_event_transfer_complete)) {
        if (timeout-- == 0U) {
            (void)plat_ch32v203_dma_ops->pf_cmd(tx_channel, false);
            return hal_uart_err_timeout;
        }
    }

    /* 6. 清事件、关通道，再等 USART 发送完成（TC） */
    (void)plat_ch32v203_dma_ops->pf_clear_event(
        tx_channel, (uint8_t)platform_ch32v203_dma_event_transfer_complete);
    (void)plat_ch32v203_dma_ops->pf_cmd(tx_channel, false);

    timeout = p_object->config.timeout_count;
    while (!plat_ch32v203_uart_ops->pf_get_flag(instance, PLAT_UART_FLAG_TC)) {
        if (timeout-- == 0U) {
            return hal_uart_err_timeout;
        }
    }

    return hal_uart_ok;
}

/**
 * @brief 轮询直读一段数据（逐字节等 RXNE 后读数据寄存器）。
 * @param p_object 模块对象指针。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_timeout 等待 RXNE 超时。
 */
hal_uart_status_enum hal_uart_read(hal_uart_t *p_object, uint8_t *p_rx_buf,
                                   uint16_t data_length)
{
    uintptr_t instance;
    uint32_t timeout;
    uint16_t index;

    if ((!hal_uart_is_object_valid(p_object)) || (p_rx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_uart_err_param;
    }

    instance = p_object->base_address;

    for (index = 0U; index < data_length; index++) {
        timeout = p_object->config.timeout_count;
        while (!plat_ch32v203_uart_ops->pf_get_flag(instance,
                                                    PLAT_UART_FLAG_RXNE)) {
            if (timeout-- == 0U) {
                return hal_uart_err_timeout;
            }
        }
        p_rx_buf[index] =
            (uint8_t)plat_ch32v203_uart_ops->pf_receive_data(instance);
    }

    return hal_uart_ok;
}

/**
 * @brief 查询发送通道是否仍在传输（忙判断）。
 * @param p_object 模块对象指针。
 * @retval true DMA 剩余计数非 0，发送尚未结束。
 * @retval false 发送空闲或对象无效。
 */
bool hal_uart_is_tx_busy(const hal_uart_t *p_object)
{
    if (!hal_uart_is_object_valid(p_object)) {
        return false;
    }

    return (plat_ch32v203_dma_ops->pf_get_data_length(
                hal_uart_dma_channel(true)) != 0U);
}

/**
 * @brief IDLE 空闲中断服务本体（空闲中断接收部分）。
 * @details 对应参考实现 usart3_dma_handle() 的前半段：判定并清 IDLE 标志（先读
 *          状态寄存器再读数据寄存器）→ 由接收 DMA 剩余计数算出当前写位置
 *          （循环模式下 已收字节 = 缓冲长度 - 剩余）→ 经绑定的回调把写位置上报
 *          上层。环形队列写入与整帧提取由 BSP 层完成，本层不持有帧元数据。
 * @param p_object 模块对象指针。
 */
void hal_uart_rx_idle_handler(hal_uart_t *p_object)
{
    uintptr_t instance;
    uint16_t remaining;
    uint16_t write_position;

    if ((!hal_uart_is_object_valid(p_object)) ||
        (p_object->p_rx_buffer == NULL) ||
        (p_object->pf_rx_callback == NULL)) {
        return;
    }

    instance = p_object->base_address;

    /* 1. 判定 IDLE 中断并清标志：先读状态寄存器再读数据寄存器 */
    if (!plat_ch32v203_uart_ops->pf_get_it_status(instance, PLAT_UART_IT_IDLE)) {
        return;
    }
    plat_ch32v203_uart_ops->pf_clear_idle(instance);

    /* 2. 由接收 DMA 剩余计数算当前写位置（循环模式下 已收字节 = 缓冲长度 - 剩余） */
    remaining = plat_ch32v203_dma_ops->pf_get_data_length(
        hal_uart_dma_channel(false));
    if (remaining >= p_object->rx_buffer_size) {
        write_position = 0U;
    } else {
        write_position = (uint16_t)(p_object->rx_buffer_size - remaining);
    }

    /* 3. 把写位置上报上层：环形队列写入与整帧提取由上层负责 */
    p_object->pf_rx_callback(p_object->p_rx_context, write_position);
}

/**
 * @brief 使能/禁止 IDLE 空闲中断源（中断线已在 hal_uart_init() 中配好）。
 * @details 与 hal_adc.c 的 hal_adc_enable_injected_irq() 同一形式：中断线（含抢占/
 *          子优先级，取本文件板级常量 HAL_UART_IDLE_IRQ_PREEMPTION_PRIORITY /
 *          HAL_UART_IDLE_IRQ_SUB_PRIORITY）在构建时配置并使能，本接口只在运行时
 *          开关 IDLE 中断源（USART_ITConfig）。
 * @param p_object 模块对象指针。
 * @param enable true 使能 IDLE 中断源，false 关闭。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 对象未绑定。
 * @retval hal_uart_err_state 平台层拒绝。
 */
hal_uart_status_enum hal_uart_enable_idle_irq(hal_uart_t *p_object, bool enable)
{
    uintptr_t instance;

    if (!hal_uart_is_object_valid(p_object)) {
        return hal_uart_err_param;
    }

    instance = p_object->base_address;

    if (!plat_ch32v203_uart_ops->pf_it_config(instance, PLAT_UART_IT_IDLE,
                                              enable)) {
        return hal_uart_err_state;
    }

    return hal_uart_ok;
}

/**
 * @brief 返回UART对象当前状态。
 * @param p_object 模块对象指针。
 * @return 当前对象状态；对象为空时返回 hal_uart_state_error。
 */
hal_uart_state_enum hal_uart_get_state(const hal_uart_t *p_object)
{
    if (p_object == NULL) {
        return hal_uart_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * 板级构建
 * ========================================================================== */

/**
 * @brief 构建板级 UART 实例（引脚组 + 配置 + DMA + 使能）。
 * @param p_object UART 对象指针。
 * @param p_rx_buffer 接收 DMA 缓冲首地址，可为 NULL（只发送）。
 * @param rx_buffer_size 接收 DMA 缓冲字节数。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_state 引脚组构建或平台层配置失败。
 */
hal_uart_status_enum hal_uart_build(hal_uart_t *p_object,
                                    uint8_t *p_rx_buffer,
                                    uint16_t rx_buffer_size)
{
    hal_uart_status_enum status;

    if (p_object == NULL) {
        return hal_uart_err_param;
    }

    /* 板级前置：PB10(TX)/PB11(RX) 引脚组（经 HAL GPIO 函数表构建） */
    if (hal_gpio_ops->pf_build(&hal_gpio_usart, hal_gpio_usart_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_usart_pin_table)) !=
        hal_gpio_ok) {
        return hal_uart_err_state;
    }

    status = hal_uart_bind(p_object, (uintptr_t)PLAT_PERIPH_USART3,
                           &s_usart_config);
    if (status != hal_uart_ok) {
        return status;
    }

    if (p_rx_buffer != NULL) {
        status = hal_uart_bind_rx_buffer(p_object, p_rx_buffer,
                                         rx_buffer_size);
        if (status != hal_uart_ok) {
            return status;
        }
    }

    /* 时钟 + 帧格式 + 收发 DMA 装配 + 传输请求 + USART 使能（默认不开中断） */
    return hal_uart_init(p_object);
}

/* ==========================================================================
 * HAL 层 UART 函数表
 * ========================================================================== */

/** @brief HAL 层 UART 函数表（本文件私有，绑定本层对外接口）。 */
static const hal_uart_ops_t s_hal_uart_ops = {
    .pf_build = hal_uart_build,
    .pf_bind_rx_buffer = hal_uart_bind_rx_buffer,
    .pf_bind_rx_callback = hal_uart_bind_rx_callback,
    .pf_init = hal_uart_init,
    .pf_deinit = hal_uart_deinit,
    .pf_enable = hal_uart_enable,
    .pf_disable = hal_uart_disable,
    .pf_send = hal_uart_send,
    .pf_read = hal_uart_read,
    .pf_is_tx_busy = hal_uart_is_tx_busy,
    .pf_rx_idle_handler = hal_uart_rx_idle_handler,
    .pf_enable_idle_irq = hal_uart_enable_idle_irq,
    .pf_get_state = hal_uart_get_state
};

/** @brief HAL 层 UART 函数表指针（对外外露，声明于 hal_uart.h）。 */
const hal_uart_ops_t *const hal_uart_ops = &s_hal_uart_ops;
