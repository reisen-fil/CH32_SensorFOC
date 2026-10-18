/**
 * @file hal_i2c.c
 * @brief I2C 对象操作 + CH32V203 板级实例（I2C1 @400kHz 角度传感器）+ HAL 函数表。
 * @details 硬件访问统一经平台 I2C 操作函数表指针 plat_ch32v203_i2c_ops 完成，
 *          I2C1 重映射经平台 GPIO 操作函数表（plat_ch32v203_gpio_ops）完成，
 *          引脚组构建经 HAL GPIO 函数表指针（hal_gpio_ops）完成，
 *          本文件不含任何厂商库符号。
 *          事务时序（本例以 2 字节寄存器读为主）依据参考实现 drv_i2c.c 的硬件 I2C
 *          轮询路径（Hardware_I2C_Mode==1，drv_SoftI2C_ReadTwoBytes 的 #ifdef 分支）：
 *          等 !BUSY → START → EVT5 → 地址(写) → EVT6 → 写寄存器地址 → EVT8_2 →
 *          重复 START → EVT5 → 地址(读) → EVT6 → 逐字节等 RXNE（末字节前关应答并发
 *          STOP）→ 恢复应答；每阶段轮询超时 0x1000，BUSY 检测计数 0x10。
 *          板级配置依据 drv_i2c.c / drv_i2c.h：400kHz 快速模式、占空比 2、自身地址
 *          0x00、应答使能、7 位应答地址；目标器件为 AS5600 类角度传感器（0x36）、
 *          角度寄存器 0x0C、读 2 字节；可选 DMA 路径（参考实现的 DMA 模式）未使用
 *          （通用 DMA 通道配置在平台层 platform_ch32v203_dma.c）。
 * @note 软件模拟 I2C 备选：参考实现最终选用软件模拟（drv_i2c.h 注明 400kHz 硬件读
 *       一次角度约 157µs、软件模拟约 148µs）。本文件按硬件 I2C1 实现；改用软件模拟
 *       时可用 hal_gpio_ops->pf_write()/pf_read() 直接翻转 SCL/SDA（引脚枚举
 *       hal_gpio_i2c_pin_scl / hal_gpio_i2c_pin_sda），本层对外接口保持不变。
 *       参考实现的"9 个时钟脉冲 + STOP"总线解锁依赖 µs 级延时与引脚模式切换，
 *       本工程无延时设施，故卡总线恢复只做"软件复位 + 重新配置 + 重新使能"，并在
 *       hal_i2c_recover_bus() 处注明。
 */
#include "hal_i2c.h"
#include "hal_gpio.h"
#include <stddef.h>


// 当前功能层待配合实际I2C接口进行验证

/* ==========================================================================
 * 板级配置：I2C1 参数与角度传感器寄存器（来源：drv_i2c.c / drv_i2c.h）
 * ========================================================================== */

/**
 * @brief 板级编码器 I2C 配置（来源 drv_i2c.c 的 I2C 初始化结构体与轮询超时）。
 * @details 400kHz 快速模式、占空比 Tlow/Thigh = 2、主机自身地址 0x00、应答使能、
 *          7 位应答地址；单阶段轮询超时取参考实现的 0x1000。
 */
static const hal_i2c_config_t s_encoder_i2c_config = {
    .clock_speed_hz = 400000U,
    .mode = PLAT_I2C_MODE_I2C,
    .duty_cycle = PLAT_I2C_DUTY_CYCLE_2,
    .own_address = 0x00U,
    .ack = PLAT_I2C_ACK_ENABLE,
    .acknowledged_address = PLAT_I2C_ACKNOWLEDGED_ADDRESS_7BIT,
    .timeout_count = PLAT_I2C_TIMEOUT_COUNT,
    .p_user_data = NULL
};

/** @brief 板级编码器 I2C 实例（I2C1 @400kHz，PB8/PB9 重映射）。 */
hal_i2c_t hal_i2c_encoder = {0};

/* ==========================================================================
 * 对象操作
 * ========================================================================== */

/**
 * @brief 检查I2C对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（实例基地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_i2c_is_object_valid(const hal_i2c_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 检查I2C对象是否处于可收发状态。
 * @param p_object 模块对象指针。
 * @retval true 已绑定且状态为 ready/enabled。
 * @retval false 未绑定、未初始化或已出错。
 */
static bool hal_i2c_is_transferable(const hal_i2c_t *p_object)
{
    if (!hal_i2c_is_object_valid(p_object)) {
        return false;
    }

    return (p_object->state == hal_i2c_state_ready) ||
           (p_object->state == hal_i2c_state_enabled);
}

/**
 * @brief 绑定I2C对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址。
 * @param p_config 模块初始化配置指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或必需指针为空。
 */
hal_i2c_status_enum hal_i2c_bind(hal_i2c_t *p_object,
                                 uintptr_t base_address,
                                 const hal_i2c_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) || (base_address == 0U)) {
        return hal_i2c_err_param;
    }

    p_object->state = hal_i2c_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;

    return hal_i2c_ok;
}

/**
 * @brief 把对象配置写入 I2C 外设并使能（不含时钟与 GPIO，供 init/总线恢复复用）。
 * @details 对应参考实现 drv_i2c.c 的"配置 + 使能"两步，并补一次应答使能
 *          （参考实现的卡总线检测末尾动作）。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_state 平台层拒绝（实例不受支持/配置非法）。
 */
static hal_i2c_status_enum hal_i2c_apply_config(hal_i2c_t *p_object)
{
    platform_ch32v203_i2c_config_t platform_config;
    uintptr_t instance = p_object->base_address;

    platform_config.clock_speed_hz = p_object->config.clock_speed_hz;
    platform_config.mode = p_object->config.mode;
    platform_config.duty_cycle = p_object->config.duty_cycle;
    platform_config.own_address = p_object->config.own_address;
    platform_config.ack = p_object->config.ack;
    platform_config.acknowledged_address = p_object->config.acknowledged_address;

    if ((!plat_ch32v203_i2c_ops->pf_config(instance, &platform_config)) ||
        (!plat_ch32v203_i2c_ops->pf_ack_config(instance, true)) ||
        (!plat_ch32v203_i2c_ops->pf_cmd(instance, true))) {
        return hal_i2c_err_state;
    }

    return hal_i2c_ok;
}

/**
 * @brief 初始化I2C：时钟使能 + 复位 + 配置 + 应答使能 + 外设使能。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 对象未绑定。
 * @retval hal_i2c_err_state 实例不受支持或配置失败。
 */
hal_i2c_status_enum hal_i2c_init(hal_i2c_t *p_object)
{
    if (!hal_i2c_is_object_valid(p_object)) {
        return hal_i2c_err_param;
    }

    if ((!plat_ch32v203_i2c_ops->pf_clock_enable(p_object->base_address)) ||
        (!plat_ch32v203_i2c_ops->pf_deinit(p_object->base_address))) {
        p_object->state = hal_i2c_state_error;
        return hal_i2c_err_state;
    }

    if (hal_i2c_apply_config(p_object) != hal_i2c_ok) {
        p_object->state = hal_i2c_state_error;
        return hal_i2c_err_state;
    }

    p_object->state = hal_i2c_state_ready;
    return hal_i2c_ok;
}

/**
 * @brief 关闭并复位I2C外设。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 对象未绑定。
 * @retval hal_i2c_err_state 实例不受支持。
 */
hal_i2c_status_enum hal_i2c_deinit(hal_i2c_t *p_object)
{
    if (!hal_i2c_is_object_valid(p_object)) {
        return hal_i2c_err_param;
    }

    if ((!plat_ch32v203_i2c_ops->pf_cmd(p_object->base_address, false)) ||
        (!plat_ch32v203_i2c_ops->pf_deinit(p_object->base_address))) {
        return hal_i2c_err_state;
    }

    p_object->state = hal_i2c_state_reset;
    return hal_i2c_ok;
}

/**
 * @brief 使能I2C外设。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 对象未绑定。
 * @retval hal_i2c_err_state 实例不受支持。
 */
hal_i2c_status_enum hal_i2c_enable(hal_i2c_t *p_object)
{
    if (!hal_i2c_is_object_valid(p_object)) {
        return hal_i2c_err_param;
    }

    if ((!plat_ch32v203_i2c_ops->pf_ack_config(p_object->base_address, true)) ||
        (!plat_ch32v203_i2c_ops->pf_cmd(p_object->base_address, true))) {
        return hal_i2c_err_state;
    }

    p_object->state = hal_i2c_state_enabled;
    return hal_i2c_ok;
}

/**
 * @brief 禁止I2C外设。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 对象未绑定。
 * @retval hal_i2c_err_state 实例不受支持。
 */
hal_i2c_status_enum hal_i2c_disable(hal_i2c_t *p_object)
{
    if (!hal_i2c_is_object_valid(p_object)) {
        return hal_i2c_err_param;
    }

    if (!plat_ch32v203_i2c_ops->pf_cmd(p_object->base_address, false)) {
        return hal_i2c_err_state;
    }

    p_object->state = hal_i2c_state_disabled;
    return hal_i2c_ok;
}

/* ==========================================================================
 * 事务时序（本层私有）：忙等待、恢复、事件轮询、收发阶段
 * ========================================================================== */

/**
 * @brief 等总线空闲（!BUSY），超时计数取参考实现的 BUSY 检测计数 0x10。
 * @param p_object 模块对象指针。
 * @retval true 总线空闲。
 * @retval false 等待超时（总线被占用/卡死）。
 */
static bool hal_i2c_wait_bus_idle(const hal_i2c_t *p_object)
{
    uint32_t timeout = PLAT_I2C_BUS_LOCK_TIMEOUT;

    while (plat_ch32v203_i2c_ops->pf_get_flag(p_object->base_address,
                                              PLAT_I2C_FLAG_BUSY) ==
           PLAT_I2C_STATUS_SET) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 卡总线恢复：软件复位 + 重新配置 + 重新使能。
 * @details 对应参考实现 drv_i2c.c 的总线解锁恢复函数前半段（禁止外设 →
 *          软件复位置位 → 释放软件复位 → 重新配置并使能）。参考实现随后还有
 *          "SCL/SDA 切推挽输出 + 9 个时钟脉冲 + STOP + 读回确认"的引脚级解锁，需要
 *          µs 级延时与引脚模式切换，而本工程无延时设施（无 Tick_Delay/Delay 实现），
 *          故该路径未实现；软件复位后重新配置 I2C 已足以清空状态机与残余标志。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 恢复成功且总线已空闲。
 * @retval hal_i2c_err_state 重新配置失败。
 * @retval hal_i2c_err_timeout 复位后总线仍为忙。
 */
static hal_i2c_status_enum hal_i2c_recover_bus(hal_i2c_t *p_object)
{
    uintptr_t instance = p_object->base_address;

    (void)plat_ch32v203_i2c_ops->pf_cmd(instance, false);
    (void)plat_ch32v203_i2c_ops->pf_software_reset(instance, true);   // I2C死锁解锁执行部分在后续有待验证与更改
    (void)plat_ch32v203_i2c_ops->pf_software_reset(instance, false);

    if (hal_i2c_apply_config(p_object) != hal_i2c_ok) {
        return hal_i2c_err_state;
    }

    if (!hal_i2c_wait_bus_idle(p_object)) {
        return hal_i2c_err_timeout;
    }

    return hal_i2c_ok;
}

/**
 * @brief 事务前置：等总线空闲，必要时做一次卡总线恢复。
 * @param p_object 模块对象指针。
 * @retval hal_i2c_ok 总线可用。
 * @retval hal_i2c_err_state 恢复时重新配置失败。
 * @retval hal_i2c_err_timeout 总线忙且恢复无效。
 */
static hal_i2c_status_enum hal_i2c_prepare_bus(hal_i2c_t *p_object)
{
    if (hal_i2c_wait_bus_idle(p_object)) {
        return hal_i2c_ok;
    }

    return hal_i2c_recover_bus(p_object);
}

/**
 * @brief 等事件成立（pf_check_event 轮询，超时计数取 config.timeout_count）。
 * @param p_object 模块对象指针。
 * @param event 事件，取 PLAT_I2C_EVENT_x。
 * @retval true 事件成立。
 * @retval false 等待超时。
 */
static bool hal_i2c_wait_event(const hal_i2c_t *p_object, uint32_t event)
{
    uint32_t timeout = p_object->config.timeout_count;

    while (!plat_ch32v203_i2c_ops->pf_check_event(p_object->base_address,
                                                  event)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 等标志到达期望状态（pf_get_flag 轮询，超时计数取 config.timeout_count）。
 * @param p_object 模块对象指针。
 * @param flag 标志，取 PLAT_I2C_FLAG_x。
 * @param status 期望状态，取 PLAT_I2C_STATUS_SET / RESET。
 * @retval true 标志已到达期望状态。
 * @retval false 等待超时。
 */
static bool hal_i2c_wait_flag(const hal_i2c_t *p_object, uint32_t flag,
                              uint8_t status)
{
    uint32_t timeout = p_object->config.timeout_count;

    while (plat_ch32v203_i2c_ops->pf_get_flag(p_object->base_address, flag) !=
           status) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 释放总线：产生 STOP、清除应答失败标志并恢复应答位。
 * @details 任一阶段超时/从机未应答（AF）时调用，避免总线被本机占住
 *          （参考实现在各失败分支均产生 STOP 释放总线）。
 * @param p_object 模块对象指针。
 */
static void hal_i2c_abort(hal_i2c_t *p_object)
{
    uintptr_t instance = p_object->base_address;

    (void)plat_ch32v203_i2c_ops->pf_stop(instance, true);

    if (plat_ch32v203_i2c_ops->pf_get_ack_failed(instance)) {
        (void)plat_ch32v203_i2c_ops->pf_clear_flag(instance, PLAT_I2C_FLAG_AF);
    }

    (void)plat_ch32v203_i2c_ops->pf_ack_config(instance, true);
}

/**
 * @brief START 阶段：产生 START、等 EVT5、发地址、等地址被应答事件。
 * @details 写寄存器地址后再次调用即为重复 START（参考实现读时序第 4 步）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param is_read true 读模式地址（EVT6 为接收模式），false 写模式地址。
 * @retval true 地址阶段完成。
 * @retval false 事件等待超时或从机未应答。
 */
static bool hal_i2c_start_with_address(const hal_i2c_t *p_object,
                                       uint8_t device_address, bool is_read)
{
    uintptr_t instance = p_object->base_address;

    if (!plat_ch32v203_i2c_ops->pf_start(instance, true)) {
        return false;
    }
    if (!hal_i2c_wait_event(p_object, PLAT_I2C_EVENT_MASTER_MODE_SELECT)) {
        return false;
    }
    if (!plat_ch32v203_i2c_ops->pf_send_address(instance, device_address,
                                               is_read)) {
        return false;
    }

    return hal_i2c_wait_event(p_object,
                              is_read ?
                              PLAT_I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED :
                              PLAT_I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
}

/**
 * @brief 数据发送阶段：逐字节等 TXE 后写入，末尾等整字节发送完成（EVT8_2）。
 * @param p_object 模块对象指针。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param data_length 待发送字节数。
 * @retval true 全部字节发送完成。
 * @retval false 标志/事件等待超时。
 */
static bool hal_i2c_send_bytes(const hal_i2c_t *p_object,
                               const uint8_t *p_tx_buf, uint16_t data_length)
{
    uintptr_t instance = p_object->base_address;
    uint16_t index;

    for (index = 0U; index < data_length; index++) {
        if (!hal_i2c_wait_flag(p_object, PLAT_I2C_FLAG_TXE,
                              PLAT_I2C_STATUS_SET)) {
            return false;
        }
        if (!plat_ch32v203_i2c_ops->pf_send_data(instance, p_tx_buf[index])) {
            return false;
        }
    }

    return hal_i2c_wait_event(p_object,
                              PLAT_I2C_EVENT_MASTER_BYTE_TRANSMITTED);
}

/**
 * @brief 数据接收阶段：逐字节等 RXNE 后读出，末字节前关应答并发 STOP。
 * @details 边界处理与参考实现 drv_i2c.c 一致：每读一个字节前等 RXNE；对**最后一个
 *          字节**先关闭应答（NACK）并产生 STOP，再等 RXNE 并读出，全部读完后恢复
 *          应答使能。2 字节读的时序即"读第 1 字节 → 关应答 + STOP → 读第 2 字节 →
 *          开应答"。
 * @param p_object 模块对象指针。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数。
 * @retval true 全部字节接收完成。
 * @retval false 标志/事件等待超时。
 */
static bool hal_i2c_receive_bytes(const hal_i2c_t *p_object, uint8_t *p_rx_buf,
                                  uint16_t data_length)
{
    uintptr_t instance = p_object->base_address;
    uint16_t index;

    for (index = 0U; index < data_length; index++) {
        if (index == (uint16_t)(data_length - 1U)) {
            /* 末字节：先关应答（NACK）并产生 STOP，再等 RXNE 读出 */
            if (!plat_ch32v203_i2c_ops->pf_ack_config(instance, false)) {
                return false;
            }
            if (!plat_ch32v203_i2c_ops->pf_stop(instance, true)) {
                return false;
            }
        }

        if (!hal_i2c_wait_flag(p_object, PLAT_I2C_FLAG_RXNE,
                              PLAT_I2C_STATUS_SET)) {
            return false;
        }
        p_rx_buf[index] = plat_ch32v203_i2c_ops->pf_receive_data(instance);
    }

    return plat_ch32v203_i2c_ops->pf_ack_config(instance, true);
}

/* ==========================================================================
 * 收发接口
 * ========================================================================== */

/**
 * @brief 向指定I2C设备写 N 字节（START + 地址(写) + 数据 + STOP）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param data_length 待发送字节数。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_state 平台层拒绝（停止动作失败）。
 * @retval hal_i2c_err_timeout 等待超时或从机未应答（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_write(hal_i2c_t *p_object,
                                  uint8_t device_address,
                                  const uint8_t *p_tx_buf,
                                  uint16_t data_length)
{
    hal_i2c_status_enum status;

    if ((!hal_i2c_is_transferable(p_object)) || (p_tx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_i2c_err_param;
    }

    status = hal_i2c_prepare_bus(p_object);
    if (status != hal_i2c_ok) {
        return status;
    }

    if ((!hal_i2c_start_with_address(p_object, device_address, false)) ||
        (!hal_i2c_send_bytes(p_object, p_tx_buf, data_length))) {
        hal_i2c_abort(p_object);
        return hal_i2c_err_timeout;
    }

    if (!plat_ch32v203_i2c_ops->pf_stop(p_object->base_address, true)) {
        return hal_i2c_err_state;
    }

    return hal_i2c_ok;
}

/**
 * @brief 从指定I2C设备读 N 字节（START + 地址(读) + N 字节 + NACK/STOP）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 等待超时或从机未应答（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_read(hal_i2c_t *p_object,
                                 uint8_t device_address,
                                 uint8_t *p_rx_buf,
                                 uint16_t data_length)
{
    hal_i2c_status_enum status;

    if ((!hal_i2c_is_transferable(p_object)) || (p_rx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_i2c_err_param;
    }

    status = hal_i2c_prepare_bus(p_object);
    if (status != hal_i2c_ok) {
        return status;
    }

    if (!hal_i2c_start_with_address(p_object, device_address, true)) {
        hal_i2c_abort(p_object);
        return hal_i2c_err_timeout;
    }

    if (!hal_i2c_receive_bytes(p_object, p_rx_buf, data_length)) {
        hal_i2c_abort(p_object);
        return hal_i2c_err_timeout;
    }

    return hal_i2c_ok;
}

/**
 * @brief 从 I2C 设备指定寄存器读 N 字节（写寄存器地址 + 重复 START + 读）。
 * @details 与参考实现 drv_i2c.c 的硬件 I2C 轮询读路径逐阶段对应；寄存器地址为 8 位
 *          （AS5600 等该类器件的寄存器地址宽度）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param register_address 起始寄存器地址（8 位）。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 等待超时或从机未应答（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_mem_read(hal_i2c_t *p_object,
                                     uint8_t device_address,
                                     uint8_t register_address,
                                     uint8_t *p_rx_buf,
                                     uint16_t data_length)
{
    hal_i2c_status_enum status;

    if ((!hal_i2c_is_transferable(p_object)) || (p_rx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_i2c_err_param;
    }

    status = hal_i2c_prepare_bus(p_object);
    if (status != hal_i2c_ok) {
        return status;
    }

    /* 阶段一：写寄存器地址（不发 STOP，为随后的重复 START 保持总线） */
    if ((!hal_i2c_start_with_address(p_object, device_address, false)) ||
        (!hal_i2c_send_bytes(p_object, &register_address, 1U))) {
        hal_i2c_abort(p_object);
        return hal_i2c_err_timeout;
    }

    /* 阶段二：重复 START + 地址(读) + N 字节（含末字节 NACK+STOP） */
    if ((!hal_i2c_start_with_address(p_object, device_address, true)) ||
        (!hal_i2c_receive_bytes(p_object, p_rx_buf, data_length))) {
        hal_i2c_abort(p_object);
        return hal_i2c_err_timeout;
    }

    return hal_i2c_ok;
}

/**
 * @brief 向 I2C 设备指定寄存器写 N 字节。
 * @details 寄存器地址与数据在同一写事务内连续发送（寄存器地址在前），末尾发 STOP。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param register_address 起始寄存器地址（8 位）。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param data_length 待发送字节数。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_state 平台层拒绝（停止动作失败）。
 * @retval hal_i2c_err_timeout 等待超时或从机未应答（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_mem_write(hal_i2c_t *p_object,
                                      uint8_t device_address,
                                      uint8_t register_address,
                                      const uint8_t *p_tx_buf,
                                      uint16_t data_length)
{
    hal_i2c_status_enum status;

    if ((!hal_i2c_is_transferable(p_object)) || (p_tx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_i2c_err_param;
    }

    status = hal_i2c_prepare_bus(p_object);
    if (status != hal_i2c_ok) {
        return status;
    }

    if ((!hal_i2c_start_with_address(p_object, device_address, false)) ||
        (!hal_i2c_send_bytes(p_object, &register_address, 1U)) ||
        (!hal_i2c_send_bytes(p_object, p_tx_buf, data_length))) {
        hal_i2c_abort(p_object);
        return hal_i2c_err_timeout;
    }

    if (!plat_ch32v203_i2c_ops->pf_stop(p_object->base_address, true)) {
        return hal_i2c_err_state;
    }

    return hal_i2c_ok;
}

/**
 * @brief 从指定寄存器连续读 2 字节（对应参考实现 drv_SoftI2C_ReadTwoBytes）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param register_address 起始寄存器地址。
 * @param p_data 输出缓冲区（至少 2 字节，p_data[0] 为高字节）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 等待超时或从机未应答（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_read_two_bytes(hal_i2c_t *p_object,
                                           uint8_t device_address,
                                           uint8_t register_address,
                                           uint8_t *p_data)
{
    return hal_i2c_mem_read(p_object, device_address, register_address, p_data,
                            HAL_I2C_TWO_BYTES_LENGTH);
}

/**
 * @brief 读取板级角度传感器的原始角度值（板级核心用途）。
 * @details 设备地址 0x36、寄存器 0x0C、读 2 字节，高字节在前（drv_i2c.c 的
 *          pdata[0] 为高字节），取低 12 位原始角度。
 * @param p_object 模块对象指针。
 * @param p_raw_angle 输出原始角度（12 位）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 等待超时或从机未应答（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_encoder_read_angle(hal_i2c_t *p_object,
                                               uint16_t *p_raw_angle)
{
    uint8_t data[HAL_I2C_ENCODER_ANGLE_LENGTH];
    hal_i2c_status_enum status;

    if (p_raw_angle == NULL) {
        return hal_i2c_err_param;
    }

    status = hal_i2c_mem_read(p_object, HAL_I2C_ENCODER_DEVICE_ADDRESS,
                              HAL_I2C_ENCODER_ANGLE_REGISTER, data,
                              HAL_I2C_ENCODER_ANGLE_LENGTH);
    if (status != hal_i2c_ok) {
        return status;
    }

    *p_raw_angle = (uint16_t)((((uint16_t)data[0] << 8U) |
                               (uint16_t)data[1]) &
                              HAL_I2C_ENCODER_ANGLE_MASK);
    return hal_i2c_ok;
}

/**
 * @brief 返回I2C对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_i2c_state_enum hal_i2c_get_state(const hal_i2c_t *p_object)
{
    if (p_object == NULL) {
        return hal_i2c_state_error;
    }
    return p_object->state;
}

/* ==========================================================================
 * 板级构建
 * ========================================================================== */

/**
 * @brief 构建板级 I2C 实例（I2C1 重映射 + 引脚组 + 配置 + 使能）。
 * @details 板级前置①：PB8/PB9 属 I2C1 重映射引脚，必须先经平台 GPIO 操作函数表
 *          使能 AFIO 时钟与 I2C1 重映射（参考实现 drv_i2c.c 用默认脚 PB6/PB7，
 *          本工程板级表为 PB8/PB9，故重映射不可省）。
 *          板级前置②：经 HAL GPIO 函数表构建 hal_gpio_i2c_hardware 引脚组
 *          （PB8=SCL、PB9=SDA，复用开漏、空闲高）。
 *          随后绑定板级配置（I2C1 @400kHz）并初始化（时钟 + 复位 + 配置 + 使能）。
 * @param p_object I2C 对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效。
 * @retval hal_i2c_err_state 引脚组构建失败或平台层配置失败。
 */
hal_i2c_status_enum hal_i2c_build(hal_i2c_t *p_object)
{
    hal_i2c_status_enum status;

    if (p_object == NULL) {
        return hal_i2c_err_param;
    }

    /* 板级前置①：I2C1 重映射（PB8/PB9）— 必须先于引脚组构建 */
    plat_ch32v203_gpio_ops->pf_remap_enable(PLAT_GPIO_REMAP_AFIO_RCC,
                                            PLAT_GPIO_REMAP_I2C1);

    /* 板级前置②：PB8(SCL)/PB9(SDA) 复用开漏引脚组（经 HAL GPIO 函数表构建） */
    if (hal_gpio_ops->pf_build(
            &hal_gpio_i2c_hardware, hal_gpio_i2c_hardware_pin_table,
            HAL_GPIO_TABLE_COUNT(hal_gpio_i2c_hardware_pin_table)) !=
        hal_gpio_ok) {
        return hal_i2c_err_state;
    }

    status = hal_i2c_bind(p_object, (uintptr_t)PLAT_PERIPH_I2C1,
                          &s_encoder_i2c_config);
    if (status != hal_i2c_ok) {
        return status;
    }

    return hal_i2c_init(p_object);
}

/* ==========================================================================
 * HAL 层 I2C 函数表
 * ========================================================================== */

/** @brief HAL 层 I2C 函数表（本文件私有，绑定本层对外接口）。 */
static const hal_i2c_ops_t s_hal_i2c_ops = {
    .pf_build = hal_i2c_build,
    // .pf_init = hal_i2c_init,
    // .pf_write = hal_i2c_write,
    // .pf_read = hal_i2c_read,
    .pf_mem_read = hal_i2c_mem_read,
    .pf_mem_write = hal_i2c_mem_write,
    .pf_read_two_bytes = hal_i2c_read_two_bytes,
    /* 备用接口：与同名 HAL 函数一一对应，供外部按需接入 */
    .pf_get_state = hal_i2c_get_state
};

/** @brief HAL 层 I2C 函数表指针（对外外露，声明于 hal_i2c.h）。 */
const hal_i2c_ops_t *const hal_i2c_ops = &s_hal_i2c_ops;
