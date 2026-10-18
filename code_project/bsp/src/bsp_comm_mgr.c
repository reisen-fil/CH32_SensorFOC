/**
 * @file bsp_comm_mgr.c
 * @brief 通信链路 BSP 实现（JustFloat 组帧发送 + 接收环形队列与整帧提取）。
 * @details 硬件访问统一经 HAL UART 操作函数表指针 hal_uart_ops 完成，本文件不含
 *          厂商库符号与平台细节；板级配置、实例与函数表在本文件给出。
 *          发送时序：bsp_comm_mgr_send_justfloat()
 *          → 通道数据按小端搬入组帧缓冲 + 末尾补 4 字节帧尾
 *          → hal_uart_ops->pf_send() 把整帧阻塞发出。
 *          接收时序（帧边界由 IDLE 中断界定，环形队列在本层）：
 *          应用层 USART3_IRQHandler → hal_uart_ops->pf_rx_idle_handler()
 *          （清 IDLE + 由接收 DMA 剩余计数算写位置）
 *          → 本层回调 bsp_comm_mgr_rx_notify() → bsp_comm_mgr_rx_idle_handler()
 *          （写入帧元数据环）→ bsp_comm_mgr_get_frame() 拷出整帧。
 */

/* 
    待构建项：
    1、CAN收发BSP级函数封装（后续进行新增）
    2、接收帧解析：把取出的整帧转成上层可用的命令/参数（后续进行新增）

    待测项：
    1、CAN收发BSP层测试（后续测试）
    2、USART收发BSP层测试（JustFloat 组帧 + 发送、IDLE 定帧 + 取帧，已经上机核对完成）
*/

#include "bsp_comm_mgr.h"

#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * JustFloat 协议常量
 * ========================================================================== */

/**
 * @brief JustFloat 帧尾：0x00 0x00 0x80 0x7F。
 * @note 该 4 字节按小端解析即 0x7F800000（+Inf），上位机以它判定"一帧数据结束"。
 */
static const uint8_t s_justfloat_tail[BSP_COMM_MGR_JUSTFLOAT_TAIL_SIZE] = {
    0x00U, 0x00U, 0x80U, 0x7FU
};

/** @brief 编译期约束：协议要求每通道 4 字节单精度浮点（size 不符时此处报错）。 */
typedef char bsp_comm_mgr_float_size_check[
    (sizeof(float) == BSP_COMM_MGR_JUSTFLOAT_CHANNEL_SIZE) ? 1 : -1];

/* ==========================================================================
 * 板级配置与实例
 * ========================================================================== */

/** @brief 板级调试通信链路配置（承载介质：USART3 板级实例 hal_uart_comm）。 */
static const bsp_comm_mgr_config_t s_comm_mgr_config = {
    .role = bsp_comm_mgr_role_debug,
    .transport = bsp_comm_mgr_transport_uart,
    .maximum_payload_length = BSP_COMM_MGR_JUSTFLOAT_FRAME_SIZE,
    .timeout_count = 0U,        /* 保留：发送超时由 HAL UART 自身配置负责 */
    .p_user_data = NULL
};

/** @brief 调试通信链路实例（USART3 + JustFloat）。 */
bsp_comm_mgr_t bsp_comm_mgr_debug = {0};

/* ==========================================================================
 * 接收环形队列内部工具
 * ========================================================================== */

/**
 * @brief 复位接收环形队列（清帧计数、读写索引、游标与标志）。
 * @param p_rx 接收句柄指针。
 */
static void bsp_comm_mgr_rx_handle_reset(bsp_comm_mgr_rx_handle_t *p_rx)
{
    p_rx->dma_buffer_size = BSP_COMM_MGR_RX_DMA_BUFFER_SIZE;
    p_rx->dma_cursor = 0U;
    p_rx->write_index = 0U;
    p_rx->read_index = 0U;
    p_rx->frame_count = 0U;
    p_rx->frame_length = 0U;
    p_rx->is_frame_ready = false;
}

/**
 * @brief 接收通知回调适配：HAL 回调 → 本层环形队列写入（中断上下文）。
 * @param p_context 通信对象指针（绑定时给出）。
 * @param write_position DMA 当前写位置。
 */
static void bsp_comm_mgr_rx_notify(void *p_context, uint16_t write_position)
{
    bsp_comm_mgr_rx_idle_handler((bsp_comm_mgr_t *)p_context, write_position);
}

/* ==========================================================================
 * 对象操作
 * ========================================================================== */

/**
 * @brief 初始化通信链路：构建底层 UART 板级实例并把对象置为可用。
 * @details 顺序：复位接收环形队列（DMA 随后从缓冲起点开始写）→ 经 HAL UART
 *          函数表构建板级实例（引脚组/UART 配置/收发 DMA/使能，接收 DMA 的目的
 *          缓冲即本层 rx.dma_buffer）→ 绑定接收通知回调 → 下发板级配置并清空组帧
 *          缓冲。本接口不使能 IDLE 中断。
 * @param p_object 通信对象指针。
 * @retval bsp_comm_mgr_ok 操作成功。
 * @retval bsp_comm_mgr_err_param 参数无效。
 * @retval bsp_comm_mgr_err_state 底层 UART 构建或回调绑定失败，对象置为 error。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_init(bsp_comm_mgr_t *p_object)
{
    if (p_object == NULL) {
        return bsp_comm_mgr_err_param;
    }

    /* 接收环形队列先复位：DMA 随即从缓冲起点开始写，游标必须与之一致 */
    bsp_comm_mgr_rx_handle_reset(&p_object->rx);

    /* 底层 UART 板级实例（USART3 + DMA 收发）：pf_build() 内部完成
       引脚组构建 → 绑定实例与配置 → 绑定本层接收缓冲 → UART/DMA 装配与使能 */
    if (hal_uart_ops->pf_build(&hal_uart_comm, p_object->rx.dma_buffer,
                               BSP_COMM_MGR_RX_DMA_BUFFER_SIZE) != hal_uart_ok) {
        p_object->state = bsp_comm_mgr_state_error;
        return bsp_comm_mgr_err_state;
    }

    /* 绑定接收通知回调：IDLE 中断把 DMA 写位置交给本层环形队列 */
    if (hal_uart_ops->pf_bind_rx_callback(&hal_uart_comm,
                                          bsp_comm_mgr_rx_notify,
                                          p_object) != hal_uart_ok) {
        p_object->state = bsp_comm_mgr_state_error;
        return bsp_comm_mgr_err_state;
    }

    p_object->config = s_comm_mgr_config;
    (void)memset(p_object->tx_frame, 0, sizeof(p_object->tx_frame));
    p_object->tx_frame_length = 0U;
    p_object->tx_channel_count = 0U;
    p_object->state = bsp_comm_mgr_state_enabled;

    return bsp_comm_mgr_ok;
}

/**
 * @brief 按 JustFloat 协议发送一帧浮点数据。
 * @details 组帧：N 个通道按小端 IEEE-754 单精度依次搬入组帧缓冲，末尾补 4 字节帧尾
 *          0x00 0x00 0x80 0x7F；整帧一次经 hal_uart_ops->pf_send() 阻塞发出。
 *          目标 MCU 为小端，float 按内存原样搬运即为协议要求的字节序。
 * @param p_object 通信对象指针。
 * @param p_data 待发送浮点数组首地址（元素个数不少于 channel_count）。
 * @param channel_count 本帧通道数（1 … BSP_COMM_MGR_JUSTFLOAT_MAX_CHANNELS）。
 * @retval bsp_comm_mgr_ok 操作成功。
 * @retval bsp_comm_mgr_err_param 参数无效、通道数越界或帧长超过配置上限。
 * @retval bsp_comm_mgr_err_state 对象未初始化或底层发送失败。
 * @retval bsp_comm_mgr_err_timeout 底层发送超时。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_send_justfloat(
    bsp_comm_mgr_t *p_object,
    const float *p_data,
    uint8_t channel_count)
{
    hal_uart_status_enum hal_status;
    uint16_t frame_length;
    uint8_t index;

    if ((p_object == NULL) || (p_data == NULL) || (channel_count == 0U) ||
        (channel_count > BSP_COMM_MGR_JUSTFLOAT_MAX_CHANNELS)) {
        return bsp_comm_mgr_err_param;
    }
    if (p_object->state != bsp_comm_mgr_state_enabled) {
        return bsp_comm_mgr_err_state;
    }

    /* 1. 组帧：通道数据 + 帧尾（帧长 = 通道数 × 4 + 4） */
    for (index = 0U; index < channel_count; index++) {
        (void)memcpy(
            &p_object->tx_frame[index * BSP_COMM_MGR_JUSTFLOAT_CHANNEL_SIZE],
            &p_data[index], BSP_COMM_MGR_JUSTFLOAT_CHANNEL_SIZE);
    }

    frame_length = (uint16_t)(
        (channel_count * BSP_COMM_MGR_JUSTFLOAT_CHANNEL_SIZE) +
        BSP_COMM_MGR_JUSTFLOAT_TAIL_SIZE);
    if (frame_length > p_object->config.maximum_payload_length) {
        return bsp_comm_mgr_err_param;
    }

    (void)memcpy(
        &p_object->tx_frame[frame_length - BSP_COMM_MGR_JUSTFLOAT_TAIL_SIZE],
        s_justfloat_tail, BSP_COMM_MGR_JUSTFLOAT_TAIL_SIZE);

    /* 2. 整帧发送：地址与长度交给 HAL UART，由其完成串口 + DMA 阻塞发送 */
    hal_status = hal_uart_ops->pf_send(&hal_uart_comm, p_object->tx_frame,
                                       frame_length);
    if (hal_status != hal_uart_ok) {
        return (hal_status == hal_uart_err_timeout) ? bsp_comm_mgr_err_timeout
                                                    : bsp_comm_mgr_err_state;
    }

    p_object->tx_frame_length = frame_length;
    p_object->tx_channel_count = channel_count;

    return bsp_comm_mgr_ok;
}

/**
 * @brief 接收环形队列写入（IDLE 中断上下文）。
 * @details 写位置未变化时视为无新数据直接返回；否则与上次游标比较得到本帧长度
 *          （越过环形缓冲尾部时按回绕计算），把 (start_offset, length) 写入帧元数据
 *          环（环满时覆盖旧帧且不再增计数），最后更新游标。
 * @param p_object 通信对象指针。
 * @param write_position DMA 在 rx.dma_buffer 中的当前写位置（0 … dma_buffer_size）。
 */
void bsp_comm_mgr_rx_idle_handler(bsp_comm_mgr_t *p_object,
                                  uint16_t write_position)
{
    bsp_comm_mgr_rx_handle_t *p_rx;
    uint16_t length;

    if (p_object == NULL) {
        return;
    }

    p_rx = &p_object->rx;

    /* 异常写位置直接丢弃，避免元数据越界 */
    if (write_position > p_rx->dma_buffer_size) {
        return;
    }

    /* 写位置未变化：本次 IDLE 没有新数据 */
    if (write_position == p_rx->dma_cursor) {
        return;
    }

    /* 1. 与上次游标比较得到本帧长度（越过缓冲尾部时按环形回绕计算） */
    if (write_position > p_rx->dma_cursor) {
        length = (uint16_t)(write_position - p_rx->dma_cursor);
    } else {
        length = (uint16_t)(p_rx->dma_buffer_size - p_rx->dma_cursor +
                            write_position);
    }

    /* 2. 存入帧元数据环；环满时覆盖旧帧且不再增计数 */
    p_rx->frame_info[p_rx->write_index].start_offset = p_rx->dma_cursor;
    p_rx->frame_info[p_rx->write_index].length = length;
    p_rx->write_index =
        (uint8_t)((p_rx->write_index + 1U) % BSP_COMM_MGR_RX_FRAME_COUNT);
    if (p_rx->frame_count < BSP_COMM_MGR_RX_FRAME_COUNT) {
        p_rx->frame_count++;
    }

    /* 3. 更新游标 */
    p_rx->dma_cursor = write_position;
}

/**
 * @brief 从帧元数据环取出最早一帧（含环形回绕的两段拷贝）。
 * @details frame_count 为 0 时直接返回；读索引前移、计数减一（无论拷贝成功与否该帧
 *          都出环）；start + length 未越界时一次拷贝，越界时分两段拷贝；长度限幅到
 *          整帧缓冲容量；取出的整帧与长度写入 rx.frame_buffer / rx.frame_length。
 * @param p_object 通信对象指针。
 * @retval bsp_comm_mgr_ok 成功取出一帧。
 * @retval bsp_comm_mgr_err_param 参数无效。
 * @retval bsp_comm_mgr_err_state 当前无待取帧，或帧元数据异常（该帧被丢弃）。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_get_frame(bsp_comm_mgr_t *p_object)
{
    bsp_comm_mgr_rx_handle_t *p_rx;
    bsp_comm_mgr_frame_info_t *p_frame;
    uint16_t start;
    uint16_t length;
    uint16_t part1;

    if (p_object == NULL) {
        return bsp_comm_mgr_err_param;
    }

    p_rx = &p_object->rx;

    if (p_rx->frame_count == 0U) {
        p_rx->is_frame_ready = false;
        return bsp_comm_mgr_err_state;
    }

    p_frame = &p_rx->frame_info[p_rx->read_index];
    start = p_frame->start_offset;
    length = p_frame->length;

    /* 读索引前移、计数减一：无论拷贝成功与否该帧都出环 */
    p_rx->read_index =
        (uint8_t)((p_rx->read_index + 1U) % BSP_COMM_MGR_RX_FRAME_COUNT);
    p_rx->frame_count--;
    p_rx->is_frame_ready = false;

    /* 异常元数据直接丢弃，避免越界读 */
    if ((start >= p_rx->dma_buffer_size) || (length == 0U)) {
        return bsp_comm_mgr_err_state;
    }

    /* 长度限幅到整帧缓冲容量（参考实现中同样有 MAX_FRAME_SIZE 限幅） */
    if (length > BSP_COMM_MGR_RX_FRAME_BUFFER_SIZE) {
        length = BSP_COMM_MGR_RX_FRAME_BUFFER_SIZE;
    }

    /* 环形回绕处理：未越界一次拷贝，越界分两段 */
    if ((uint16_t)(start + length) <= p_rx->dma_buffer_size) {
        (void)memcpy(p_rx->frame_buffer, &p_rx->dma_buffer[start], length);
    } else {
        part1 = (uint16_t)(p_rx->dma_buffer_size - start);
        (void)memcpy(p_rx->frame_buffer, &p_rx->dma_buffer[start], part1);
        (void)memcpy(&p_rx->frame_buffer[part1], p_rx->dma_buffer,
                     (size_t)(length - part1));
    }

    p_rx->frame_length = length;
    p_rx->is_frame_ready = true;

    return bsp_comm_mgr_ok;
}

/**
 * @brief 查询帧元数据环中是否有待取完整帧。
 * @param p_object 通信对象指针。
 * @retval true 有帧待取。
 * @retval false 无帧或对象无效。
 */
bool bsp_comm_mgr_is_data_available(const bsp_comm_mgr_t *p_object)
{
    if (p_object == NULL) {
        return false;
    }

    return (p_object->rx.frame_count > 0U);
}

/**
 * @brief 使能/禁止接收 IDLE 空闲中断源（中断线已在 bsp_comm_mgr_init() 内配好）。
 * @details 经 HAL 层 hal_uart_ops->pf_enable_idle_irq() 在运行时开关 IDLE 中断源；
 *          抢占/子优先级由 HAL 层板级常量给出（抢占 1 / 子 0）。
 * @param p_object 通信对象指针。
 * @param enable true 使能 IDLE 中断源，false 关闭。
 * @retval bsp_comm_mgr_ok 操作成功。
 * @retval bsp_comm_mgr_err_param 参数无效。
 * @retval bsp_comm_mgr_err_state 使能时对象未初始化，或平台层拒绝。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_enable_rx_irq(bsp_comm_mgr_t *p_object,
                                                    bool enable)
{
    if (p_object == NULL) {
        return bsp_comm_mgr_err_param;
    }
    /* 关闭中断任何时候都允许；使能则要求对象已初始化 */
    if ((enable) && (p_object->state != bsp_comm_mgr_state_enabled)) {
        return bsp_comm_mgr_err_state;
    }

    if (hal_uart_ops->pf_enable_idle_irq(&hal_uart_comm, enable) != hal_uart_ok) {
        return bsp_comm_mgr_err_state;
    }

    return bsp_comm_mgr_ok;
}

/**
 * @brief 查询通信对象当前状态。
 * @param p_object 通信对象指针。
 * @return 对象状态；p_object 为 NULL 时返回 bsp_comm_mgr_state_error。
 */
bsp_comm_mgr_state_enum bsp_comm_mgr_get_state(const bsp_comm_mgr_t *p_object)
{
    if (p_object == NULL) {
        return bsp_comm_mgr_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * 通信链路操作函数表
 * ========================================================================== */

/** @brief 通信链路操作函数表（本文件私有，绑定本层对外接口）。 */
static const bsp_comm_mgr_ops_t s_bsp_comm_mgr_ops = {
    .pf_init = bsp_comm_mgr_init,
    .pf_send_justfloat = bsp_comm_mgr_send_justfloat,
    .pf_rx_idle_handler = bsp_comm_mgr_rx_idle_handler,
    .pf_get_frame = bsp_comm_mgr_get_frame,
    .pf_is_data_available = bsp_comm_mgr_is_data_available,
    .pf_enable_rx_irq = bsp_comm_mgr_enable_rx_irq,
    .pf_get_state = bsp_comm_mgr_get_state
};

/** @brief 通信链路操作函数表指针（对外外露，声明于 bsp_comm_mgr.h）。 */
const bsp_comm_mgr_ops_t *const bsp_comm_mgr_ops = &s_bsp_comm_mgr_ops;
