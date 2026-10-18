/**
 * @file hal_can.c
 * @brief CAN 对象操作 + CH32V203 板级实例（CAN1 主机）+ HAL 函数表。
 * @details 硬件访问统一经平台 CAN 操作函数表指针 plat_ch32v203_can_ops 完成，
 *          引脚组构建经 HAL GPIO 函数表指针 hal_gpio_ops 完成，本文件不含厂商库符号。
 *          板级配置依据参考实现 drv_can.c：
 *          - 引脚：PA12 = CAN 发送（复用推挽 50MHz）、PA11 = CAN 接收（上拉输入），
 *            由 hal_gpio_can_pin_table 提供，CAN1 默认映射无需重映射；
 *          - 位时序：SJW = 1tq、BS1 = 2tq、BS2 = 3tq、预分频 = 12、正常模式
 *            （Bps = Fpclk1 / ((1 + BS1 + BS2) * 预分频) = 72MHz / (6 * 12) = 1Mbps）；
 *          - 总线开关：TTCM/ABOM/AWUM/RFLM/TXFP 关闭，禁止自动重传（NART 位使能）；
 *          - 过滤器组 0：掩码模式、32 位尺度、ID = 0x200<<4（即标准帧 0x100）、
 *            掩码 = 0xE00<<4（即 0x700）→ 接受标准帧 0x100~0x1FF 并路由到 FIFO0；
 *          - 自测帧：标准 ID 0x4、长度 4、数据 {0x01,0x02,0x03,0x04}；
 *          - 中断：FIFO0 消息挂起中断 + RX0 中断线，抢占/子优先级均为 1。
 * @note 本工程 startup 中 CAN1 RX0 中断向量是 .weak 的死循环默认处理函数，
 *       故板级构建默认不打开中断：应用层实现 CAN1 RX0 中断服务函数（调用
 *       hal_can_rx_handler）后，再调 hal_can_enable_rx_irq 打开中断。
 */
#include "hal_can.h"
#include "hal_gpio.h"
#include <stddef.h>

/*
    待测项：
    1、CAN数据帧发送验证（目前无法测试，后续测试）
    2、基于过滤器使用中断进行数据接收（目前无法测试，后续测试）
*/


/* ==========================================================================
 * 板级配置：位时序、工作模式、过滤器与自测帧（来源 drv_can.c / drv_can.h）
 * ========================================================================== */

/**
 * @brief CAN1 位时序与工作模式（来源 drv_can.c 的顶层初始化函数及其内部配置函数）。
 * @note Fpclk1 = 72MHz（SYSCLK 144MHz、APB1 = HCLK/2），
 *       Bps = 72MHz / ((1 + 2 + 3) * 12) = 1Mbps，与参考实现注释一致。
 */
static const hal_can_config_t s_can_host_config = {
    .mode = PLAT_CAN_MODE_NORMAL,                     /* 正常模式 */
    .synchronization_jump_width = PLAT_CAN_SJW_1TQ,   /* SJW = 1tq */
    .bit_time_segment1 = PLAT_CAN_BS1_2TQ,            /* BS1 = 2tq */
    .bit_time_segment2 = PLAT_CAN_BS2_3TQ,            /* BS2 = 3tq */
    .prescaler = 12U,                                 /* 预分频 = 12 */
    .is_time_trigger_enabled = false,                 /* 时间触发模式：关闭 */
    .is_auto_bus_off_enabled = false,                 /* 自动离线管理：关闭 */
    .is_auto_wakeup_enabled = false,                  /* 自动唤醒：关闭 */
    .is_auto_retransmission_enabled = false,          /* 禁止自动重传（NART 置位） */
    .is_receive_fifo_locked_enabled = false,          /* 接收 FIFO 锁定：关闭 */
    .is_transmit_fifo_priority_enabled = false,       /* 发送 FIFO 优先级：关闭 */
    .timeout_count = 100000U,
    .p_user_data = NULL
};

/**
 * @brief CAN1 过滤器组 0（来源 drv_can.c 内部初始化函数的过滤器配置部分）。
 * @note IdHigh = 0x200 << 4，等价于标准帧 ID 0x100（标准 ID 位于高 16 位 bit15:5）；
 *       MaskIdHigh = 0xE00 << 4，等价于掩码 0x700，故接受标准帧 ID 0x100~0x1FF。
 *       fifo_assignment 同时被 hal_can_receive()/hal_can_rx_handler() 用作接收 FIFO 号。
 */
static const hal_can_filter_config_t s_can_host_filter = {
    .filter_number = 0U,                              /* 过滤器组 0 */
    .mode = PLAT_CAN_FILTER_MODE_ID_MASK,             /* 掩码模式 */
    .scale = PLAT_CAN_FILTER_SCALE_32BIT,             /* 32 位尺度 */
    .id_high = (uint16_t)(0x200U << 4),               /* 标准帧 ID 0x100 */
    .id_low = 0U,                                     /* 标准帧、数据帧 */
    .mask_id_high = (uint16_t)(0xE00U << 4),          /* 掩码 0x700 → 0x100~0x1FF */
    .mask_id_low = 0U,
    .fifo_assignment = PLAT_CAN_FILTER_FIFO_0,        /* 路由到 FIFO0 */
    .is_activated = true                              /* 激活该过滤器组 */
};

/** @brief 板级自测数据（来源 drv_can.c 的发送缓存数组 {0x01,0x02,0x03,0x04}）。 */
const uint8_t hal_can_test_data[HAL_CAN_TEST_DATA_LENGTH] = {
    0x01U, 0x02U, 0x03U, 0x04U
};

/** @brief CAN1 板级主机实例。 */
hal_can_t hal_can_host = {0};

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
 * @brief 检查CAN对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（实例基地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_can_is_object_valid(const hal_can_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 把 HAL 报文转换为平台报文。
 * @param p_frame 输入：HAL 报文。
 * @param p_message 输出：平台报文。
 */
static void hal_can_frame_to_platform(
    const hal_can_frame_t *p_frame,
    platform_ch32v203_can_message_t *p_message)
{
    uint8_t index;

    p_message->identifier = p_frame->identifier;
    p_message->data_length = p_frame->data_length;
    p_message->is_extended = p_frame->is_extended;
    p_message->is_remote = p_frame->is_remote;
    p_message->filter_index = 0U;

    for (index = 0U; index < HAL_CAN_DATA_LENGTH_MAX; index++) {
        p_message->data[index] = p_frame->data[index];
    }
}

/**
 * @brief 把平台报文转换为 HAL 报文。
 * @param p_message 输入：平台报文。
 * @param p_frame 输出：HAL 报文。
 */
static void hal_can_frame_from_platform(
    const platform_ch32v203_can_message_t *p_message,
    hal_can_frame_t *p_frame)
{
    uint8_t index;

    p_frame->identifier = p_message->identifier;
    p_frame->data_length = p_message->data_length;
    p_frame->is_extended = p_message->is_extended;
    p_frame->is_remote = p_message->is_remote;
    p_frame->filter_index = p_message->filter_index;

    for (index = 0U; index < HAL_CAN_DATA_LENGTH_MAX; index++) {
        p_frame->data[index] = p_message->data[index];
    }
}

/**
 * @brief 取对象过滤器关联的接收 FIFO 号。
 * @param p_object 模块对象指针。
 * @return FIFO 号（PLAT_CAN_FIFO_x）；过滤器未配置时默认 FIFO0。
 */
static uint8_t hal_can_get_receive_fifo(const hal_can_t *p_object)
{
    return (uint8_t)(p_object->filter.fifo_assignment);
}

/* ==========================================================================
 * 对象操作
 * ========================================================================== */

/**
 * @brief 绑定CAN对象：保存实例基地址、总线配置与过滤器配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址。
 * @param p_config 位时序与工作模式配置。
 * @param p_filter 过滤器组配置。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效或必需指针为空。
 */
hal_can_status_enum hal_can_bind(hal_can_t *p_object,
                                 uintptr_t base_address,
                                 const hal_can_config_t *p_config,
                                 const hal_can_filter_config_t *p_filter)
{
    if ((p_object == NULL) || (p_config == NULL) || (p_filter == NULL) ||
        (base_address == 0U)) {
        return hal_can_err_param;
    }

    p_object->state = hal_can_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;
    p_object->filter = *p_filter;
    p_object->rx_frame.identifier = 0U;
    p_object->rx_frame.data_length = 0U;
    p_object->rx_finish_flag = false;

    return hal_can_ok;
}

/**
 * @brief 初始化CAN：时钟 + 复位 + 位时序/模式配置 + 过滤器配置。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 * @retval hal_can_err_timeout 位时序配置未能退出初始化模式。
 * @retval hal_can_err_state 时钟/复位或过滤器配置失败。
 */
hal_can_status_enum hal_can_init(hal_can_t *p_object)
{
    platform_ch32v203_can_config_t platform_config;
    platform_ch32v203_can_filter_config_t platform_filter;
    uintptr_t instance;

    if (!hal_can_is_object_valid(p_object)) {
        return hal_can_err_param;
    }

    instance = p_object->base_address;

    platform_config.prescaler = p_object->config.prescaler;
    platform_config.mode = (uint8_t)p_object->config.mode;
    platform_config.sync_jump_width = p_object->config.synchronization_jump_width;
    platform_config.bit_segment1 = p_object->config.bit_time_segment1;
    platform_config.bit_segment2 = p_object->config.bit_time_segment2;
    platform_config.is_time_trigger_enabled = p_object->config.is_time_trigger_enabled;
    platform_config.is_auto_bus_off_enabled = p_object->config.is_auto_bus_off_enabled;
    platform_config.is_auto_wakeup_enabled = p_object->config.is_auto_wakeup_enabled;
    /* HAL 用正向语义（允许自动重传），平台层对应标准库 NART 位（禁止自动重传） */
    platform_config.is_no_auto_retransmission_enabled =
        (!p_object->config.is_auto_retransmission_enabled);
    platform_config.is_receive_fifo_locked_enabled =
        p_object->config.is_receive_fifo_locked_enabled;
    platform_config.is_transmit_fifo_priority_enabled =
        p_object->config.is_transmit_fifo_priority_enabled;

    platform_filter.filter_number = p_object->filter.filter_number;
    platform_filter.mode = p_object->filter.mode;
    platform_filter.scale = p_object->filter.scale;
    platform_filter.id_high = p_object->filter.id_high;
    platform_filter.id_low = p_object->filter.id_low;
    platform_filter.mask_id_high = p_object->filter.mask_id_high;
    platform_filter.mask_id_low = p_object->filter.mask_id_low;
    platform_filter.fifo_assignment = p_object->filter.fifo_assignment;
    platform_filter.is_activated = p_object->filter.is_activated;

    if ((!plat_ch32v203_can_ops->pf_clock_enable(instance)) ||
        (!plat_ch32v203_can_ops->pf_deinit(instance))) {
        p_object->state = hal_can_state_error;
        return hal_can_err_state;
    }

    if (!plat_ch32v203_can_ops->pf_config(instance, &platform_config)) {
        p_object->state = hal_can_state_error;
        return hal_can_err_timeout;
    }

    if (!plat_ch32v203_can_ops->pf_filter_config(instance, &platform_filter)) {
        p_object->state = hal_can_state_error;
        return hal_can_err_state;
    }

    p_object->rx_finish_flag = false;
    p_object->state = hal_can_state_ready;
    return hal_can_ok;
}

/**
 * @brief 停止总线（请求睡眠模式）并复位CAN外设。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 */
hal_can_status_enum hal_can_deinit(hal_can_t *p_object)
{
    if (!hal_can_is_object_valid(p_object)) {
        return hal_can_err_param;
    }

    /* 先请求睡眠模式停止总线，结果不影响复位动作 */
    (void)plat_ch32v203_can_ops->pf_operating_mode_request(
        p_object->base_address, PLAT_CAN_OPERATING_MODE_SLEEP);

    if (!plat_ch32v203_can_ops->pf_deinit(p_object->base_address)) {
        return hal_can_err_param;
    }

    p_object->rx_finish_flag = false;
    p_object->state = hal_can_state_reset;
    return hal_can_ok;
}

/**
 * @brief 启动CAN总线（请求正常模式）。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定。
 * @retval hal_can_err_state 平台层拒绝（模式切换失败）。
 */
hal_can_status_enum hal_can_enable(hal_can_t *p_object)
{
    if (!hal_can_is_object_valid(p_object)) {
        return hal_can_err_param;
    }

    if (!plat_ch32v203_can_ops->pf_operating_mode_request(
            p_object->base_address, PLAT_CAN_OPERATING_MODE_NORMAL)) {
        return hal_can_err_state;
    }

    p_object->state = hal_can_state_enabled;
    return hal_can_ok;
}

/**
 * @brief 停止CAN总线（请求睡眠模式）。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定。
 * @retval hal_can_err_state 平台层拒绝（模式切换失败）。
 */
hal_can_status_enum hal_can_disable(hal_can_t *p_object)
{
    if (!hal_can_is_object_valid(p_object)) {
        return hal_can_err_param;
    }

    if (!plat_ch32v203_can_ops->pf_operating_mode_request(
            p_object->base_address, PLAT_CAN_OPERATING_MODE_SLEEP)) {
        return hal_can_err_state;
    }

    p_object->state = hal_can_state_disabled;
    return hal_can_ok;
}

/**
 * @brief 重新配置过滤器组。
 * @param p_object 模块对象指针。
 * @param p_filter 过滤器配置。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效。
 * @retval hal_can_err_state 平台层拒绝。
 */
hal_can_status_enum hal_can_config_filter(hal_can_t *p_object,
                                          const hal_can_filter_config_t *p_filter)
{
    platform_ch32v203_can_filter_config_t platform_filter;

    if ((!hal_can_is_object_valid(p_object)) || (p_filter == NULL)) {
        return hal_can_err_param;
    }

    platform_filter.filter_number = p_filter->filter_number;
    platform_filter.mode = p_filter->mode;
    platform_filter.scale = p_filter->scale;
    platform_filter.id_high = p_filter->id_high;
    platform_filter.id_low = p_filter->id_low;
    platform_filter.mask_id_high = p_filter->mask_id_high;
    platform_filter.mask_id_low = p_filter->mask_id_low;
    platform_filter.fifo_assignment = p_filter->fifo_assignment;
    platform_filter.is_activated = p_filter->is_activated;

    if (!plat_ch32v203_can_ops->pf_filter_config(p_object->base_address,
                                                 &platform_filter)) {
        return hal_can_err_state;
    }

    p_object->filter = *p_filter;
    return hal_can_ok;
}

/**
 * @brief 使能 FIFO 消息挂起中断并配置 CAN1 RX0 中断线。
 * @param p_object 模块对象指针。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 * @retval hal_can_err_state 平台层中断配置失败。
 */
hal_can_status_enum hal_can_enable_rx_irq(hal_can_t *p_object,
                                          uint8_t preemption_priority,
                                          uint8_t sub_priority)
{
    if (!hal_can_is_object_valid(p_object)) {
        return hal_can_err_param;
    }

    if ((!plat_ch32v203_can_ops->pf_it_config(p_object->base_address,
                                             PLAT_CAN_IT_FMP0, true)) ||
        (!plat_ch32v203_can_ops->pf_irq_enable(PLAT_CAN_IRQ_RX0,
                                              preemption_priority,
                                              sub_priority))) {
        return hal_can_err_state;
    }

    return hal_can_ok;
}

/**
 * @brief 关闭 FIFO 消息挂起中断。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 */
hal_can_status_enum hal_can_disable_rx_irq(hal_can_t *p_object)
{
    if (!hal_can_is_object_valid(p_object)) {
        return hal_can_err_param;
    }

    if (!plat_ch32v203_can_ops->pf_it_config(p_object->base_address,
                                             PLAT_CAN_IT_FMP0, false)) {
        return hal_can_err_param;
    }

    return hal_can_ok;
}

/**
 * @brief CAN1 RX0 中断服务函数本体（应用层 ISR 中调用）。
 * @param p_object 模块对象指针。
 */
void hal_can_rx_handler(hal_can_t *p_object)
{
    platform_ch32v203_can_message_t platform_message;
    uint8_t fifo_number;

    if (!hal_can_is_object_valid(p_object)) {
        return;
    }

    fifo_number = hal_can_get_receive_fifo(p_object);

    /* 消息挂起中断有效且上一帧已被取走时才接收新帧（参考实现 rx_finish_flag 语义） */
    if ((plat_ch32v203_can_ops->pf_get_it_status(p_object->base_address,
                                                 PLAT_CAN_IT_FMP0)) &&
        (!p_object->rx_finish_flag)) {
        if (plat_ch32v203_can_ops->pf_receive(p_object->base_address,
                                             fifo_number,
                                             &platform_message)) {
            hal_can_frame_from_platform(&platform_message, &p_object->rx_frame);
            p_object->rx_finish_flag = true;
        }
    }

    /* 清除中断挂起位 */
    (void)plat_ch32v203_can_ops->pf_clear_it_pending_bit(p_object->base_address,
                                                         PLAT_CAN_IT_FMP0);
}

/**
 * @brief 发送一个CAN报文（请求挂入发送邮箱，成功即返回）。
 * @param p_object 模块对象指针。
 * @param p_frame 待发送报文。
 * @retval hal_can_ok 报文已挂入发送邮箱。
 * @retval hal_can_err_param 参数无效或标识符超出帧类型范围。
 * @retval hal_can_err_no_mailbox 三个发送邮箱全忙，报文未挂入。
 */
hal_can_status_enum hal_can_send(hal_can_t *p_object,
                                 const hal_can_frame_t *p_frame)
{
    platform_ch32v203_can_message_t platform_message;
    uint8_t mailbox;

    if ((!hal_can_is_object_valid(p_object)) || (p_frame == NULL) ||
        (p_frame->data_length > HAL_CAN_DATA_LENGTH_MAX)) {
        return hal_can_err_param;
    }

    if ((!p_frame->is_extended) && (p_frame->identifier > HAL_CAN_STD_ID_MAX)) {
        return hal_can_err_param;
    }

    hal_can_frame_to_platform(p_frame, &platform_message);

    mailbox = plat_ch32v203_can_ops->pf_transmit(p_object->base_address,
                                                 &platform_message);
    if (mailbox == PLAT_CAN_MAILBOX_NONE) {
        return hal_can_err_param;
    }
    if (mailbox == PLAT_CAN_TX_STATUS_NO_MAILBOX) {
        return hal_can_err_no_mailbox;
    }

    /* 报文已挂入邮箱即视为成功（与参考实现标准帧发送函数返回 1 的语义一致） */
    return hal_can_ok;
}

/**
 * @brief 发送一个标准数据帧（对应参考实现的标准帧发送函数）。
 * @param p_object 模块对象指针。
 * @param std_id 标准帧 ID。
 * @param p_data 数据缓冲区。
 * @param length 数据长度（超过 8 时按参考实现截断为 8）。
 * @retval hal_can_ok 报文已挂入发送邮箱。
 * @retval hal_can_err_param 参数无效或 ID 越界。
 * @retval hal_can_err_no_mailbox 三个发送邮箱全忙，报文未挂入。
 */
hal_can_status_enum hal_can_send_std_data(hal_can_t *p_object,
                                          uint32_t std_id,
                                          const uint8_t *p_data,
                                          uint8_t length)
{
    hal_can_frame_t frame = {0};
    uint8_t index;

    if ((!hal_can_is_object_valid(p_object)) || (p_data == NULL) ||
        (std_id > HAL_CAN_STD_ID_MAX)) {
        return hal_can_err_param;
    }

    /* 参考实现把超长长度截断为 8，此处保持一致 */
    if (length > HAL_CAN_DATA_LENGTH_MAX) {
        length = HAL_CAN_DATA_LENGTH_MAX;
    }

    frame.identifier = std_id;
    frame.data_length = length;
    frame.is_extended = false;
    frame.is_remote = false;
    for (index = 0U; index < length; index++) {
        frame.data[index] = p_data[index];
    }

    return hal_can_send(p_object, &frame);
}

/**
 * @brief 发送板级自测帧（对应参考实现的自测发送函数）。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 报文已挂入发送邮箱。
 * @retval hal_can_err_param 对象未绑定。
 * @retval hal_can_err_no_mailbox 三个发送邮箱全忙，报文未挂入。
 */
hal_can_status_enum hal_can_send_test_frame(hal_can_t *p_object)
{
    return hal_can_send_std_data(p_object, HAL_CAN_TEST_STD_ID,
                                 hal_can_test_data, HAL_CAN_TEST_DATA_LENGTH);
}

/**
 * @brief 查询指定发送邮箱的状态。
 * @param p_object 模块对象指针。
 * @param mailbox 邮箱号 0..PLAT_CAN_MAILBOX_COUNT-1。
 * @param p_status 输出：PLAT_CAN_TX_STATUS_x。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效或邮箱号越界。
 */
hal_can_status_enum hal_can_get_transmit_status(const hal_can_t *p_object,
                                                uint8_t mailbox,
                                                uint8_t *p_status)
{
    if ((!hal_can_is_object_valid(p_object)) || (p_status == NULL)) {
        return hal_can_err_param;
    }

    *p_status = plat_ch32v203_can_ops->pf_transmit_status(p_object->base_address,
                                                          mailbox);
    return hal_can_ok;
}

/**
 * @brief 接收一个CAN报文。
 * @param p_object 模块对象指针。
 * @param p_frame 输出报文。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效。
 * @retval hal_can_err_state 当前无待处理报文。
 */
hal_can_status_enum hal_can_receive(hal_can_t *p_object,
                                    hal_can_frame_t *p_frame)
{
    platform_ch32v203_can_message_t platform_message;
    uint8_t fifo_number;

    if ((!hal_can_is_object_valid(p_object)) || (p_frame == NULL)) {
        return hal_can_err_param;
    }

    /* 1) 中断路径已缓存一帧：直接取用并清标志 */
    if (p_object->rx_finish_flag) {
        *p_frame = p_object->rx_frame;
        p_object->rx_finish_flag = false;
        return hal_can_ok;
    }

    /* 2) 轮询路径：过滤器关联 FIFO 中无待处理报文则返回 */
    fifo_number = hal_can_get_receive_fifo(p_object);
    if (plat_ch32v203_can_ops->pf_message_pending(p_object->base_address,
                                                  fifo_number) == 0U) {
        return hal_can_err_state;
    }

    if (!plat_ch32v203_can_ops->pf_receive(p_object->base_address, fifo_number,
                                           &platform_message)) {
        return hal_can_err_state;
    }

    hal_can_frame_from_platform(&platform_message, p_frame);
    return hal_can_ok;
}

/**
 * @brief 查询待处理报文数（含中断缓存中的 1 帧）。
 * @param p_object 模块对象指针。
 * @return 待处理报文数；对象为空或未绑定时返回 0。
 */
uint8_t hal_can_is_message_pending(const hal_can_t *p_object)
{
    uint8_t pending;

    if (!hal_can_is_object_valid(p_object)) {
        return 0U;
    }

    pending = plat_ch32v203_can_ops->pf_message_pending(
        p_object->base_address, hal_can_get_receive_fifo(p_object));

    if (p_object->rx_finish_flag) {
        pending++;
    }

    return pending;
}

/**
 * @brief 返回CAN对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_can_state_enum hal_can_get_state(const hal_can_t *p_object)
{
    if (p_object == NULL) {
        return hal_can_state_error;
    }
    return p_object->state;
}

/* ==========================================================================
 * 板级构建
 * ========================================================================== */

/**
 * @brief 构建板级 CAN 实例（引脚组 + 时钟 + 位时序/模式 + 过滤器）。
 * @param p_object CAN 对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效。
 * @retval hal_can_err_state 引脚组构建或平台层时钟/复位/过滤器配置失败。
 * @retval hal_can_err_timeout 位时序配置未能退出初始化模式。
 */
hal_can_status_enum hal_can_build(hal_can_t *p_object)
{
    hal_can_status_enum status;

    if (p_object == NULL) {
        return hal_can_err_param;
    }

    /* 板级前置：PA12（CAN 发送）/PA11（CAN 接收）引脚组，默认映射无需重映射 */
    if (hal_gpio_ops->pf_build(&hal_gpio_can,
                               hal_gpio_can_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_can_pin_table))
        != hal_gpio_ok) {
        return hal_can_err_state;
    }

    status = hal_can_bind(p_object, (uintptr_t)PLAT_PERIPH_CAN1,
                          &s_can_host_config, &s_can_host_filter);
    if (status != hal_can_ok) {
        return status;
    }

    /* 默认不打开 RX0 中断（startup 中该向量为 .weak 死循环），build 后即可轮询收发 */
    return hal_can_init(p_object);
}

/* ==========================================================================
 * HAL 层 CAN 函数表
 * ========================================================================== */

/** @brief HAL 层 CAN 函数表（本文件私有，绑定本层对外接口）。 */
static const hal_can_ops_t s_hal_can_ops = {
    .pf_build = hal_can_build,
    .pf_init = hal_can_init,
    .pf_deinit = hal_can_deinit,
    .pf_enable = hal_can_enable,
    .pf_disable = hal_can_disable,
    .pf_config_filter = hal_can_config_filter,
    .pf_enable_rx_irq = hal_can_enable_rx_irq,
    .pf_disable_rx_irq = hal_can_disable_rx_irq,
    .pf_rx_handler = hal_can_rx_handler,
    .pf_send = hal_can_send,
    .pf_send_std_data = hal_can_send_std_data,
    .pf_send_test_frame = hal_can_send_test_frame,
    .pf_receive = hal_can_receive,
    .pf_is_message_pending = hal_can_is_message_pending,
    .pf_get_state = hal_can_get_state
};

/** @brief HAL 层 CAN 函数表指针（对外外露，声明于 hal_can.h）。 */
const hal_can_ops_t *const hal_can_ops = &s_hal_can_ops;
