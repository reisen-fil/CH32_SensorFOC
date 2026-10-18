/**
 * @file platform_ch32v203_i2c.h
 * @brief CH32V203 I2C 的 MCU 抽象接口（取值宏 + 操作函数表）。
 * @details 与 GPIO/SPI/ADC 平台层同构，本层提供两类内容：
 *          - 取值宏：工作模式 / 快速模式占空比 / 自身地址应答 / 传输方向 /
 *            事件 / 标志 / 中断源 / 中断线 / 标志状态 /
 *            超时计数，供 HAL 层配置表字段与流程判断填写；
 *          - 操作函数表 platform_ch32v203_i2c_ops_t：把标准库 I2C 接口
 *            （时钟使能、I2C_DeInit、I2C_Init、I2C_Cmd、I2C_SoftwareResetCmd、
 *            I2C_AcknowledgeConfig、I2C_GenerateSTART/STOP、I2C_Send7bitAddress、
 *            I2C_SendData/I2C_ReceiveData、I2C_CheckEvent/I2C_GetLastEvent、
 *            I2C_GetFlagStatus、I2C_ClearFlag、I2C_ITConfig）以及 I2C 侧的 DMA
 *            请求控制与数据寄存器地址查询逐个做"简单函数封装"后绑定，并通过
 *            plat_ch32v203_i2c_ops 指针对外外露；通用 DMA 通道装配 / 开关 /
 *            存储器地址与长度 / 事件标志一律归 platform_ch32v203_dma.c
 *            （plat_ch32v203_dma_ops），通道号由该层的 pf_channel_get 统一给出。
 *          I2C 的对象模型、板级实例（hal_i2c_encoder，I2C1 @400kHz）与"设备地址 +
 *          寄存器地址 + 读 N 字节"的事务时序（START/重复 START、事件轮询、超时、
 *          末字节 NACK+STOP）实现于 hal_i2c.c/h；本层不做任何轮询与延时。
 * @note 本层不提供任何业务语义（角度、寄存器含义），只做 MCU 抽象。
 * @note 硬件 I2C 与软件模拟 I2C 备选：参考实现 drv_i2c.c 同时给出硬件 I2C 与
 *       软件模拟 I2C，其头文件 drv_i2c.h 注明"400kHz 硬件 I2C 读一次角度整帧约
 *       157µs，软件模拟（汇编 NOP 延时）约 148µs，故最终选用软件模拟"。本工程按
 *       **硬件 I2C1**（PB8/PB9 重映射，400kHz 快速模式，Tlow/Thigh = 2）编写链路；
 *       软件模拟 I2C 作为备选方案，可用 HAL GPIO 位操作实现
 *       （hal_gpio_ops->pf_write() / pf_read() 直接翻转 SCL/SDA，本层不参与）。
 * @note 平台头文件中不得出现任何厂商类型（I2C_InitTypeDef / DMA_InitTypeDef 等）：
 *       本头可能在 ch32v20x_conf.h 之前被解析，故只使用 <stdint.h>/<stdbool.h>
 *       的纯 C 类型；宏体内引用 I2C_/DMA_ 常量是安全的（仅在使用处展开）。
 */
#ifndef __PLATFORM_CH32V203_I2C_H__
#define __PLATFORM_CH32V203_I2C_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/** @brief I2C 工作模式（本工程为主机模式，模式取值仍按标准库给出）。 */
#define PLAT_I2C_MODE_I2C                      I2C_Mode_I2C

/** @brief I2C 快速模式占空比（Tlow/Thigh = 2，参考实现 drv_i2c.c 取值）。 */
#define PLAT_I2C_DUTY_CYCLE_2                  I2C_DutyCycle_2
#define PLAT_I2C_DUTY_CYCLE_16_9               I2C_DutyCycle_16_9

/** @brief I2C 自身地址应答使能（主机模式下自身地址为 0x00）。 */
#define PLAT_I2C_ACK_ENABLE                    I2C_Ack_Enable
#define PLAT_I2C_ACK_DISABLE                   I2C_Ack_Disable

/** @brief I2C 应答地址宽度。 */
#define PLAT_I2C_ACKNOWLEDGED_ADDRESS_7BIT     I2C_AcknowledgedAddress_7bit
#define PLAT_I2C_ACKNOWLEDGED_ADDRESS_10BIT    I2C_AcknowledgedAddress_10bit

/** @brief I2C 传输方向（由 pf_send_address 的 is_read 形参内部选用）。 */
#define PLAT_I2C_DIRECTION_TRANSMITTER         I2C_Direction_Transmitter
#define PLAT_I2C_DIRECTION_RECEIVER            I2C_Direction_Receiver

/** @brief I2C 主机事件（供 pf_check_event 使用）。 */
#define PLAT_I2C_EVENT_MASTER_MODE_SELECT \
    I2C_EVENT_MASTER_MODE_SELECT
#define PLAT_I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED \
    I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED
#define PLAT_I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED \
    I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED
#define PLAT_I2C_EVENT_MASTER_BYTE_TRANSMITTING \
    I2C_EVENT_MASTER_BYTE_TRANSMITTING
#define PLAT_I2C_EVENT_MASTER_BYTE_TRANSMITTED \
    I2C_EVENT_MASTER_BYTE_TRANSMITTED
#define PLAT_I2C_EVENT_MASTER_BYTE_RECEIVED \
    I2C_EVENT_MASTER_BYTE_RECEIVED
#define PLAT_I2C_EVENT_MASTER_MODE_ADDRESS10 \
    I2C_EVENT_MASTER_MODE_ADDRESS10

/** @brief I2C 状态标志（供 pf_get_flag / pf_clear_flag 使用）。 */
#define PLAT_I2C_FLAG_SB                       I2C_FLAG_SB
#define PLAT_I2C_FLAG_ADDR                     I2C_FLAG_ADDR
#define PLAT_I2C_FLAG_BTF                      I2C_FLAG_BTF
#define PLAT_I2C_FLAG_TXE                      I2C_FLAG_TXE
#define PLAT_I2C_FLAG_RXNE                     I2C_FLAG_RXNE
#define PLAT_I2C_FLAG_BUSY                     I2C_FLAG_BUSY
#define PLAT_I2C_FLAG_AF                       I2C_FLAG_AF
#define PLAT_I2C_FLAG_ARLO                     I2C_FLAG_ARLO
#define PLAT_I2C_FLAG_BERR                     I2C_FLAG_BERR
#define PLAT_I2C_FLAG_OVR                      I2C_FLAG_OVR

/**
 * @brief I2C 中断源（供 pf_it_config 使用）。
 * @note 本 IP 的 I2C_ITConfig() 实参是 CTLR2 的 16 位中断使能位，故只外露
 *       EVT/ERR/BUF 三项；TXE/RXNE/BTF 等具体事件由 EVT（+BUF）组合在中断中判别
 *       （见标准库 I2C_ITConfig 形参为 uint16_t）。
 */
#define PLAT_I2C_IT_EVT                        I2C_IT_EVT
#define PLAT_I2C_IT_ERR                        I2C_IT_ERR
#define PLAT_I2C_IT_BUF                        I2C_IT_BUF

/** @brief I2C1 事件/错误中断线（供 pf_irq_enable 使用；板级轮询链路未使用）。 */
#define PLAT_I2C_IRQ_I2C1_EVENT                PLAT_IRQ_I2C1_EVENT
#define PLAT_I2C_IRQ_I2C1_ERROR                PLAT_IRQ_I2C1_ERROR

/** @brief I2C 标志状态值（pf_get_flag 的返回值）。 */
#define PLAT_I2C_STATUS_SET                    PLAT_SET
#define PLAT_I2C_STATUS_RESET                  PLAT_RESET

/**
 * @brief 参考实现 drv_i2c.c 的时序等待计数（每个阶段的轮询超时，Hardware_I2C_Mode==1）。
 */
#define PLAT_I2C_TIMEOUT_COUNT                 (0x1000U)

/** @brief 参考实现 drv_i2c.c 的 BUSY 卡总线检测计数（I2C_Lock_Check）。 */
#define PLAT_I2C_BUS_LOCK_TIMEOUT              (0x10U)

/**
 * @brief I2C 可选 DMA 路径（参考实现 drv_i2c.c 的 Hardware_I2C_Mode==2）。
 * @note 通道映射由 platform_ch32v203_dma.h 的 pf_channel_get 统一给出，
 *       I2C1 TX=CH6 / RX=CH7（platform_ch32v203_dma_channel_6 / _7）；
 *       参考实现的"内存→外设方向、外设地址为 I2C 数据寄存器、字节宽度、普通模式、
 *       高优先级"由 platform_ch32v203_dma.c 的 pf_config 按
 *       platform_ch32v203_dma_config_t 装配（外设地址由本层
 *       pf_get_data_register_address 提供）。
 *       本工程板级链路为轮询方式（等价 Hardware_I2C_Mode==1），不使用 DMA。
 */

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_i2c.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_I2C_INSTANCE_TYPE                 I2C_TypeDef
#define PLAT_I2C_INIT_TYPE                     I2C_InitTypeDef

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C 平台配置（对应标准库 I2C_InitTypeDef 的可移植子集）。
 * @details 字段取值来自上面的 PLAT_I2C_* 取值宏；主板级取值（400kHz、占空比 2、
 *          自身地址 0x00、应答使能、7 位应答地址）由 hal_i2c.c 的板级静态配置给出。
 */
typedef struct {
    uint32_t clock_speed_hz;       /**< 时钟速率，本工程 400000（400kHz 快速模式） */
    uint16_t mode;                 /**< PLAT_I2C_MODE_I2C */
    uint16_t duty_cycle;           /**< PLAT_I2C_DUTY_CYCLE_2 / 16_9 */
    uint16_t own_address;          /**< 自身地址（主机模式为 0x00） */
    uint16_t ack;                  /**< PLAT_I2C_ACK_ENABLE / DISABLE */
    uint16_t acknowledged_address; /**< PLAT_I2C_ACKNOWLEDGED_ADDRESS_7BIT / 10BIT */
} platform_ch32v203_i2c_config_t;

/**
 * @brief CH32V203 I2C 操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 全部函数以"实例基地址 + 值参数"形式描述，不含厂商类型与 HAL 类型；
 *          返回 bool 的成员 true 表示成功，取值类成员直接返回寄存器/状态值。
 *          "事件轮询、超时计数、START/重复 START 时序、末字节 NACK+STOP"一律不在
 *          本层实现，已上移到 hal_i2c.c（与 SPI 把收发细节放 HAL 的做法一致）。
 */
typedef struct {
    /** @brief 使能 I2C 实例所在总线（APB1）的外设时钟。 */
    bool (*pf_clock_enable)(uintptr_t instance);
    /** @brief 复位 I2C 外设（I2C_DeInit）。 */
    bool (*pf_deinit)(uintptr_t instance);
    /** @brief 配置 I2C（I2C_Init，实参由本结构体转成标准库初始化结构）。 */
    bool (*pf_config)(uintptr_t instance,
                      const platform_ch32v203_i2c_config_t *p_config);
    /** @brief 使能/禁止 I2C 外设（I2C_Cmd）。 */
    bool (*pf_cmd)(uintptr_t instance, bool enable);
    /** @brief 软件复位 I2C 状态机（I2C_SoftwareResetCmd，卡总线恢复用）。 */
    bool (*pf_software_reset)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止应答位（I2C_AcknowledgeConfig）。 */
    bool (*pf_ack_config)(uintptr_t instance, bool enable);
    /** @brief 产生/撤销 START 条件（I2C_GenerateSTART；重复 START 亦用它）。 */
    bool (*pf_start)(uintptr_t instance, bool enable);
    /** @brief 产生/撤销 STOP 条件（I2C_GenerateSTOP）。 */
    bool (*pf_stop)(uintptr_t instance, bool enable);
    /**
     * @brief 发送 7 位器件地址（I2C_Send7bitAddress 的简单函数封装）。
     * @param instance 实例基地址。
     * @param address 原始 7 位器件地址（如 AS5600 的 0x36）；标准库的
     *                I2C_Send7bitAddress() 只补/清方向位、不做左移，故由本封装内部
     *                完成左移（参考实现 drv_i2c.c 是在调用处自行左移的），
     *                调用方只需传真值 7 位地址。
     * @param is_read true 读（方向位置 1），false 写。
     * @retval true 操作成功。
     * @retval false 实例不受支持。
     */
    bool (*pf_send_address)(uintptr_t instance, uint8_t address, bool is_read);
    /** @brief 写一个字节（I2C_SendData）。 */
    bool (*pf_send_data)(uintptr_t instance, uint8_t data);
    /** @brief 读一个字节（I2C_ReceiveData）。 */
    uint8_t (*pf_receive_data)(uintptr_t instance);
    /** @brief 判断事件是否成立（I2C_CheckEvent，event 取 PLAT_I2C_EVENT_x）。 */
    bool (*pf_check_event)(uintptr_t instance, uint32_t event);
    /** @brief 读最近一次事件组合（I2C_GetLastEvent，诊断用）。 */
    uint32_t (*pf_get_last_event)(uintptr_t instance);
    /**
     * @brief 查询状态标志（I2C_GetFlagStatus）。
     * @return PLAT_I2C_STATUS_SET 表示置位，PLAT_I2C_STATUS_RESET 表示复位。
     */
    uint8_t (*pf_get_flag)(uintptr_t instance, uint32_t flag);
    /** @brief 查询应答失败标志 AF（从机未应答，读时序用）。 */
    bool (*pf_get_ack_failed)(uintptr_t instance);
    /** @brief 清除标志（I2C_ClearFlag，flag 取 PLAT_I2C_FLAG_x）。 */
    bool (*pf_clear_flag)(uintptr_t instance, uint32_t flag);
    /** @brief 使能/禁止指定的 I2C 中断源（I2C_ITConfig，it 取 PLAT_I2C_IT_x）。 */
    bool (*pf_it_config)(uintptr_t instance, uint16_t it, bool enable);
    /** @brief 配置并使能 I2C 中断线（板级轮询链路未使用，保留接口）。 */
    bool (*pf_irq_enable)(uint32_t irq_channel, uint8_t preemption_priority,
                          uint8_t sub_priority);
    /** @brief 使能/禁止 I2C 的 DMA 请求（I2C_DMACmd，I2C 侧寄存器位）。 */
    bool (*pf_dma_request_cmd)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止"最后一次 DMA 传输"（I2C_DMALastTransferCmd）。 */
    bool (*pf_dma_last_transfer_cmd)(uintptr_t instance, bool enable);
    /**
     * @brief 取 I2C 数据寄存器地址（供装配 DMA 通道时填外设地址）。
     * @details 通用 DMA 通道装配已由 platform_ch32v203_dma.c 承担，本项只提供
     *          "外设侧地址"这一条 I2C 事实，避免上层直接访问寄存器。
     * @param instance 实例基地址，取 PLAT_PERIPH_I2C1 / PLAT_PERIPH_I2C2。
     * @return 数据寄存器地址；实例不受支持时返回 0。
     */
    uintptr_t (*pf_get_data_register_address)(uintptr_t instance);
} platform_ch32v203_i2c_ops_t;

/** @brief CH32V203 I2C 操作函数表指针（在 platform_ch32v203_i2c.c 中组装）。 */
extern const platform_ch32v203_i2c_ops_t *const plat_ch32v203_i2c_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_I2C_H__ */
