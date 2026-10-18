/**
 * @file platform_ch32v203_i2c.c
 * @brief CH32V203 I2C 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 I2C 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_i2c_ops 函数表对外绑定；hal_i2c.c 通过该表访问 I2C 硬件：
 *          - 配置类封装：时钟使能 / 复位 / 配置 / 使能 / 软件复位 / 应答使能；
 *          - 传输原语封装：START / STOP / 7 位地址 / 写字节 / 读字节（一次调用只做
 *            一个硬件动作，事件轮询与超时不在本层）；
 *          - 状态类封装：事件判定 / 最近事件 / 标志查询 / AF 查询 / 清标志 / 中断
 *            使能与中断线配置；
 *          - 可选 DMA 路径封装：本层只保留 I2C 侧的 DMA 请求控制（DMA 请求使能与
 *            "最后一次传输"，均为 I2C 寄存器位）与数据寄存器地址查询
 *            （供上层拼 platform_ch32v203_dma_config_t 的外设地址）；
 *            DMA 通道装配 / 开关 / 存储器地址与数据长度 / 事件标志查询与清除已全部
 *            移入 platform_ch32v203_dma.c（plat_ch32v203_dma_ops），通道号由该层
 *            pf_channel_get 统一给出（I2C1 TX=CH6 / RX=CH7）。
 *            （对应参考实现 drv_i2c.c 的 Hardware_I2C_Mode==2 分支；本工程板级链路
 *            为轮询方式，等价 Hardware_I2C_Mode==1，不使用 DMA）。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；仅时钟使能沿用平台时钟宏
 *       PLAT_RCC_APB1_CLOCK_ENABLE，中断线沿用 PLAT_PFIC_INIT。
 * @note 硬件 I2C 与软件模拟 I2C 备选：参考实现 drv_i2c.c 同时给出硬件 I2C 与软件
 *       模拟 I2C（drv_i2c.h 注明 400kHz 硬件读一次角度约 157µs、软件模拟约 148µs，
 *       最终选用软件模拟）。本工程按硬件 I2C1 编写；若改用软件模拟，可在 HAL 层用
 *       HAL GPIO 位操作（hal_gpio_ops->pf_write()/pf_read()）实现，本层不再参与。
 *       参考实现中另有"9 个时钟脉冲 + STOP"的总线解锁恢复路径，它依赖 µs 级延时与
 *       SCL/SDA 模式切换；本工程无延时设施，故该路径未在本层实现，卡总线恢复只做
 *       "软件复位 + 重新配置 + 重新使能"（见 hal_i2c.c 的 hal_i2c_recover_bus()）。
 */
#include "platform_ch32v203_i2c.h"
#include <stddef.h>

/**
 * @brief 判断实例是否为受支持的 I2C 外设。
 * @param instance 实例基地址，取 PLAT_PERIPH_I2C1 / PLAT_PERIPH_I2C2。
 * @retval true 受支持。
 * @retval false 其它外设。
 */
static bool plat_ch32v203_i2c_is_valid(uintptr_t instance)
{
    PLAT_I2C_INSTANCE_TYPE *p_i2c = (PLAT_I2C_INSTANCE_TYPE *)(instance);

    return (p_i2c == PLAT_PERIPH_I2C1) || (p_i2c == PLAT_PERIPH_I2C2);
}

/**
 * @brief 使能 I2C 实例所在总线（APB1）的外设时钟。
 * @note 时钟使能是本文件通过平台宏完成的动作之一。I2C1/I2C2 同在 APB1。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_clock_enable(uintptr_t instance)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    if ((PLAT_I2C_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_I2C1) {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_I2C1);
    } else {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_I2C2);
    }

    return true;
}

/**
 * @brief 复位 I2C 外设（I2C_DeInit）。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_deinit(uintptr_t instance)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_DeInit((PLAT_I2C_INSTANCE_TYPE *)(instance));
    return true;
}

/**
 * @brief 配置 I2C（I2C_Init）。
 * @param instance 实例基地址。
 * @param p_config 平台 I2C 配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持或配置为空。
 */
static bool plat_ch32v203_i2c_config(
    uintptr_t instance,
    const platform_ch32v203_i2c_config_t *p_config)
{
    PLAT_I2C_INIT_TYPE init_config;

    if ((p_config == NULL) || (!plat_ch32v203_i2c_is_valid(instance))) {
        return false;
    }

    init_config.I2C_ClockSpeed = p_config->clock_speed_hz;
    init_config.I2C_Mode = p_config->mode;
    init_config.I2C_DutyCycle = p_config->duty_cycle;
    init_config.I2C_OwnAddress1 = p_config->own_address;
    init_config.I2C_Ack = p_config->ack;
    init_config.I2C_AcknowledgedAddress = p_config->acknowledged_address;

    I2C_Init((PLAT_I2C_INSTANCE_TYPE *)(instance), &init_config);
    return true;
}

/**
 * @brief 使能/禁止 I2C 外设（I2C_Cmd）。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_Cmd((PLAT_I2C_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 软件复位 I2C 状态机（I2C_SoftwareResetCmd）。
 * @note 对应参考实现 drv_i2c.c 的 I2C_BusUnlock_Recovery 第 1 步：先 I2C_Cmd(I2Cx, DISABLE)
 *       再 I2C_SoftwareResetCmd(I2Cx, ENABLE) 清空残余标志与状态机（此处按参考实现原文引用，
 *       本文件代码内统一使用 PLAT_ENABLE / PLAT_DISABLE）。
 * @param instance 实例基地址。
 * @param enable true 置位 SWRST，false 释放 SWRST。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_software_reset(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_SoftwareResetCmd((PLAT_I2C_INSTANCE_TYPE *)(instance),
                         enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止应答位（I2C_AcknowledgeConfig）。
 * @param instance 实例基地址。
 * @param enable true 使能 ACK，false 改为 NACK（读末字节用）。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_ack_config(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_AcknowledgeConfig((PLAT_I2C_INSTANCE_TYPE *)(instance),
                          enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 产生或撤销 START 条件（I2C_GenerateSTART）。
 * @note 写寄存器地址后再次调用即为重复 START（参考实现 drv_i2c.c 读时序第 4 步）。
 * @param instance 实例基地址。
 * @param enable true 产生 START，false 撤销。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_start(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_GenerateSTART((PLAT_I2C_INSTANCE_TYPE *)(instance),
                      enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 产生或撤销 STOP 条件（I2C_GenerateSTOP）。
 * @param instance 实例基地址。
 * @param enable true 产生 STOP，false 撤销。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_stop(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_GenerateSTOP((PLAT_I2C_INSTANCE_TYPE *)(instance),
                     enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 发送 7 位器件地址（I2C_Send7bitAddress）。
 * @details 标准库只对实参补/清读写方向位，不做左移；参考实现 drv_i2c.c 是在调用处
 *          自行完成 `devAddr << 1` 的，故这里把左移收进平台封装：调用方直接传
 *          原始 7 位地址即可，方向位由 is_read 补齐。
 * @param instance 实例基地址。
 * @param address 原始 7 位器件地址（0x00~0x7F）。
 * @param is_read true 读（方向位置 1），false 写。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_send_address(uintptr_t instance, uint8_t address,
                                           bool is_read)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_Send7bitAddress((PLAT_I2C_INSTANCE_TYPE *)(instance), (uint8_t)(address << 1U),
                        is_read ? PLAT_I2C_DIRECTION_RECEIVER :
                                  PLAT_I2C_DIRECTION_TRANSMITTER);
    return true;
}

/**
 * @brief 写一个字节（I2C_SendData）。
 * @param instance 实例基地址。
 * @param data 待发送字节。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_send_data(uintptr_t instance, uint8_t data)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_SendData((PLAT_I2C_INSTANCE_TYPE *)(instance), data);
    return true;
}

/**
 * @brief 读一个字节（I2C_ReceiveData）。
 * @param instance 实例基地址。
 * @return 接收到的字节；实例非法时返回 0。
 */
static uint8_t plat_ch32v203_i2c_receive_data(uintptr_t instance)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return 0U;
    }

    return I2C_ReceiveData((PLAT_I2C_INSTANCE_TYPE *)(instance));
}

/**
 * @brief 判断事件是否成立（I2C_CheckEvent）。
 * @param instance 实例基地址。
 * @param event 事件，取 PLAT_I2C_EVENT_x。
 * @retval true 事件成立。
 * @retval false 事件未成立或实例非法。
 */
static bool plat_ch32v203_i2c_check_event(uintptr_t instance, uint32_t event)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    return (I2C_CheckEvent((PLAT_I2C_INSTANCE_TYPE *)(instance), event) == PLAT_READY);
}

/**
 * @brief 读最近一次事件组合（I2C_GetLastEvent，诊断用）。
 * @param instance 实例基地址。
 * @return STAR1|STAR2<<16 的事件值；实例非法时返回 0。
 */
static uint32_t plat_ch32v203_i2c_get_last_event(uintptr_t instance)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return 0U;
    }

    return I2C_GetLastEvent((PLAT_I2C_INSTANCE_TYPE *)(instance));
}

/**
 * @brief 查询状态标志（I2C_GetFlagStatus）。
 * @param instance 实例基地址。
 * @param flag 标志，取 PLAT_I2C_FLAG_x。
 * @return PLAT_I2C_STATUS_SET 置位 / PLAT_I2C_STATUS_RESET 复位。
 */
static uint8_t plat_ch32v203_i2c_get_flag(uintptr_t instance, uint32_t flag)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return PLAT_I2C_STATUS_RESET;
    }

    return (I2C_GetFlagStatus((PLAT_I2C_INSTANCE_TYPE *)(instance), flag) != PLAT_RESET) ?
           PLAT_I2C_STATUS_SET : PLAT_I2C_STATUS_RESET;
}

/**
 * @brief 查询应答失败标志 AF（从机未应答）。
 * @param instance 实例基地址。
 * @retval true 从机未应答（AF 置位）。
 * @retval false AF 未置位或实例非法。
 */
static bool plat_ch32v203_i2c_get_ack_failed(uintptr_t instance)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    return (I2C_GetFlagStatus((PLAT_I2C_INSTANCE_TYPE *)(instance),
                              PLAT_I2C_FLAG_AF) != PLAT_RESET);
}

/**
 * @brief 清除标志（I2C_ClearFlag）。
 * @param instance 实例基地址。
 * @param flag 待清除标志，取 PLAT_I2C_FLAG_x。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_clear_flag(uintptr_t instance, uint32_t flag)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_ClearFlag((PLAT_I2C_INSTANCE_TYPE *)(instance), flag);
    return true;
}

/**
 * @brief 使能/禁止指定的 I2C 中断源（I2C_ITConfig）。
 * @param instance 实例基地址。
 * @param it 中断源，取 PLAT_I2C_IT_EVT/ERR/BUF。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_it_config(uintptr_t instance, uint16_t it,
                                        bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_ITConfig((PLAT_I2C_INSTANCE_TYPE *)(instance), it,
                 enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 配置并使能 I2C 中断线。
 * @note 本工程板级链路为轮询方式，不使用 I2C 中断；此接口保留给中断化改造。
 * @param irq_channel 中断号，取 PLAT_I2C_IRQ_I2C1_EVENT / PLAT_I2C_IRQ_I2C1_ERROR。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_i2c_irq_enable(uint32_t irq_channel,
                                         uint8_t preemption_priority,
                                         uint8_t sub_priority)
{
    PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority, PLAT_ENABLE);
    return true;
}

/**
 * @brief 使能/禁止 I2C 的 DMA 请求（I2C_DMACmd）。
 * @note 该位属于 I2C 外设寄存器（CTLR2 的 DMAEN），不是 DMA 控制器状态，故保留在本层。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_dma_request_cmd(uintptr_t instance, bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_DMACmd((PLAT_I2C_INSTANCE_TYPE *)(instance), enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 使能/禁止"最后一次 DMA 传输"（I2C_DMALastTransferCmd）。
 * @note 参考实现 drv_i2c.c 在 DMA 接收 2 字节前使能、传输完成后禁止，用于在最后一个
 *       字节自动产生 NACK。
 * @param instance 实例基地址。
 * @param enable true 使能，false 禁止。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_i2c_dma_last_transfer_cmd(uintptr_t instance,
                                                    bool enable)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return false;
    }

    I2C_DMALastTransferCmd((PLAT_I2C_INSTANCE_TYPE *)(instance),
                           enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/**
 * @brief 取 I2C 数据寄存器（DATAR）地址。
 * @details 通道装配（DMA_Init / 方向 / 宽度 / 模式 / 优先级）已移入
 *          platform_ch32v203_dma.c，本层只提供"外设侧地址"这一条 I2C 事实，
 *          供上层填 platform_ch32v203_dma_config_t::peripheral_address。
 * @param instance 实例基地址。
 * @return 数据寄存器地址；实例不受支持时返回 0。
 */
static uintptr_t plat_ch32v203_i2c_get_data_register_address(uintptr_t instance)
{
    if (!plat_ch32v203_i2c_is_valid(instance)) {
        return 0U;
    }

    return (uintptr_t)(&(((PLAT_I2C_INSTANCE_TYPE *)(instance))->DATAR));
}

/** @brief CH32V203 I2C 操作函数表（本文件私有）。 */
static const platform_ch32v203_i2c_ops_t s_i2c_ops = {
    .pf_clock_enable = plat_ch32v203_i2c_clock_enable,
    .pf_deinit = plat_ch32v203_i2c_deinit,
    .pf_config = plat_ch32v203_i2c_config,
    .pf_cmd = plat_ch32v203_i2c_cmd,
    .pf_software_reset = plat_ch32v203_i2c_software_reset,
    .pf_ack_config = plat_ch32v203_i2c_ack_config,
    .pf_start = plat_ch32v203_i2c_start,
    .pf_stop = plat_ch32v203_i2c_stop,
    .pf_send_address = plat_ch32v203_i2c_send_address,
    .pf_send_data = plat_ch32v203_i2c_send_data,
    .pf_receive_data = plat_ch32v203_i2c_receive_data,
    .pf_check_event = plat_ch32v203_i2c_check_event,
    .pf_get_last_event = plat_ch32v203_i2c_get_last_event,
    .pf_get_flag = plat_ch32v203_i2c_get_flag,
    .pf_get_ack_failed = plat_ch32v203_i2c_get_ack_failed,
    .pf_clear_flag = plat_ch32v203_i2c_clear_flag,
    .pf_it_config = plat_ch32v203_i2c_it_config,
    .pf_irq_enable = plat_ch32v203_i2c_irq_enable,
    .pf_dma_request_cmd = plat_ch32v203_i2c_dma_request_cmd,
    .pf_dma_last_transfer_cmd = plat_ch32v203_i2c_dma_last_transfer_cmd,
    .pf_get_data_register_address =
        plat_ch32v203_i2c_get_data_register_address
};

/** @brief CH32V203 I2C 操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_i2c_ops_t *const plat_ch32v203_i2c_ops = &s_i2c_ops;
