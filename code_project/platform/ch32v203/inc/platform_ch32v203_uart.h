/**
 * @file platform_ch32v203_uart.h
 * @brief CH32V203 UART/USART 的 MCU 抽象接口（取值宏 + 操作函数表）。
 * @details 与 GPIO/SPI/ADC 平台层同构：
 *          - 取值宏：字长 / 停止位 / 校验 / 收发模式 / 硬件流控 / 标志 / 中断源 /
 *            传输请求（DMA 请求位是 USART 侧寄存器位，不属于通用 DMA 通道抽象）；
 *          - 操作函数表 platform_ch32v203_uart_ops_t：把标准库 USART 接口
 *            （时钟使能、USART_DeInit、USART_Init、USART_Cmd、USART_ITConfig、
 *            USART_DMACmd、USART_SendData/ReceiveData、标志与中断状态查询/清除、
 *            USART 数据寄存器地址查询、PLAT_PFIC_INIT 中断线配置）逐个做简单函数
 *            封装后绑定，并通过 plat_ch32v203_uart_ops 指针对外外露（表本体私有）。
 *          UART 的对象模型与板级实例（hal_uart_comm = USART3 + DMA 收发）实现于
 *          hal_uart.c / hal_uart.h；IDLE 定帧后的环形队列与整帧提取在 BSP 层。
 * @note 本层不提供任何业务语义（VOFA 协议、命令帧解析等），只做 MCU 抽象。
 * @note 本头会被 hal_uart.h 包含，而 hal.h 可能在 ch32v20x_conf.h 之前解析到本头，
 *       因此本头内**不得出现任何厂商类型**（USART_InitTypeDef、DMA_InitTypeDef 等）；
 *       结构体一律使用 <stdint.h>/<stdbool.h> 的纯 C 类型，厂商符号只允许出现在
 *       宏体中（延迟展开，见下方「类型接入宏」区块）。
 * @note 本层只保留"外设侧"的 DMA 相关操作：收发 DMA 传输请求的使能/禁止
 *       （pf_dma_request_cmd，封装 USART_DMACmd）以及 USART 数据寄存器地址查询
 *       （pf_get_data_register_address，供上层拼通用 DMA 配置）；中断线映射仍按
 *       实例在 platform_ch32v203_uart.c 内部确定，故 pf_irq_enable 的入参只给实例
 *       基地址，HAL 层无需厂商中断号。
 * @note DMA 通道映射由 platform_ch32v203_dma.h 的 pf_channel_get 统一给出
 *       （USARTx 的 TX/RX 各对应哪条抽象通道是该层唯一事实）；通道装配、使能、
 *       长度与事件标志等"通用 DMA 通道配置/状态"也一律归 plat_ch32v203_dma_ops，
 *       通用 DMA 通道优先级取该层的 PLAT_DMA_PRIORITY_x，本层不再定义通道号、
 *       传输完成标志与 DMA 优先级取值宏。
 */
#ifndef __PLATFORM_CH32V203_UART_H__
#define __PLATFORM_CH32V203_UART_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/** @brief UART 字长。 */
#define PLAT_UART_WORD_LENGTH_8B               USART_WordLength_8b
#define PLAT_UART_WORD_LENGTH_9B               USART_WordLength_9b

/** @brief UART 停止位。 */
#define PLAT_UART_STOP_BITS_1                  USART_StopBits_1
#define PLAT_UART_STOP_BITS_0_5                USART_StopBits_0_5
#define PLAT_UART_STOP_BITS_2                  USART_StopBits_2
#define PLAT_UART_STOP_BITS_1_5                USART_StopBits_1_5

/** @brief UART 校验方式。 */
#define PLAT_UART_PARITY_NONE                  USART_Parity_No
#define PLAT_UART_PARITY_EVEN                  USART_Parity_Even
#define PLAT_UART_PARITY_ODD                   USART_Parity_Odd

/** @brief UART 收发模式（可按位或组合）。 */
#define PLAT_UART_MODE_RX                      USART_Mode_Rx
#define PLAT_UART_MODE_TX                      USART_Mode_Tx

/** @brief UART 硬件流控。 */
#define PLAT_UART_FLOW_CONTROL_NONE            USART_HardwareFlowControl_None
#define PLAT_UART_FLOW_CONTROL_RTS             USART_HardwareFlowControl_RTS
#define PLAT_UART_FLOW_CONTROL_CTS             USART_HardwareFlowControl_CTS
#define PLAT_UART_FLOW_CONTROL_RTS_CTS         USART_HardwareFlowControl_RTS_CTS

/** @brief UART 状态标志（供 pf_get_flag / pf_clear_flag 使用）。 */
#define PLAT_UART_FLAG_CTS                     USART_FLAG_CTS
#define PLAT_UART_FLAG_LBD                     USART_FLAG_LBD
#define PLAT_UART_FLAG_TXE                     USART_FLAG_TXE
#define PLAT_UART_FLAG_TC                      USART_FLAG_TC
#define PLAT_UART_FLAG_RXNE                    USART_FLAG_RXNE
#define PLAT_UART_FLAG_IDLE                    USART_FLAG_IDLE
#define PLAT_UART_FLAG_ORE                     USART_FLAG_ORE
#define PLAT_UART_FLAG_NE                      USART_FLAG_NE
#define PLAT_UART_FLAG_FE                      USART_FLAG_FE
#define PLAT_UART_FLAG_PE                      USART_FLAG_PE

/** @brief UART 中断源（供 pf_it_config / pf_get_it_status 使用）。 */
#define PLAT_UART_IT_PE                        USART_IT_PE
#define PLAT_UART_IT_TXE                       USART_IT_TXE
#define PLAT_UART_IT_TC                        USART_IT_TC
#define PLAT_UART_IT_RXNE                      USART_IT_RXNE
#define PLAT_UART_IT_ORE_RX                    USART_IT_ORE_RX
#define PLAT_UART_IT_IDLE                      USART_IT_IDLE
#define PLAT_UART_IT_LBD                       USART_IT_LBD
#define PLAT_UART_IT_CTS                       USART_IT_CTS
#define PLAT_UART_IT_ERR                       USART_IT_ERR
#define PLAT_UART_IT_ORE_ER                    USART_IT_ORE_ER
#define PLAT_UART_IT_NE                        USART_IT_NE
#define PLAT_UART_IT_FE                        USART_IT_FE
#define PLAT_UART_IT_ORE                       USART_IT_ORE

/** @brief UART 的 DMA 传输请求（供 pf_dma_request_cmd 使用，属外设侧寄存器位）。 */
#define PLAT_UART_TRANSFER_REQUEST_TX          USART_DMAReq_Tx
#define PLAT_UART_TRANSFER_REQUEST_RX          USART_DMAReq_Rx

/* --------------------------------------------------------------------------
 * 收发 DMA 通道映射不在本层定义：CH32V203 的 USART↔DMA 通道是固定映射，
 * 但该映射是片内 DMA 请求表的事实，已统一收归 platform_ch32v203_dma.h /
 * platform_ch32v203_dma.c —— 由 plat_ch32v203_dma_ops->pf_channel_get(
 * platform_ch32v203_dma_peripheral_usartN, is_tx) 给出抽象通道号。
 * 通道的装配/使能/长度/传输完成事件标志同样由 plat_ch32v203_dma_ops 提供，
 * 通道优先级取 PLAT_DMA_PRIORITY_x；本层只保留外设侧的 USART_DMACmd
 * （pf_dma_request_cmd）与数据寄存器地址查询（pf_get_data_register_address）。
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_uart.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_UART_INSTANCE_TYPE                USART_TypeDef
#define PLAT_UART_INIT_TYPE                    USART_InitTypeDef

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UART 平台配置（对应标准库 USART_InitTypeDef 的可移植子集）。 */
typedef struct {
    uint32_t baud_rate;     /**< 波特率（bps） */
    uint16_t word_length;   /**< PLAT_UART_WORD_LENGTH_x */
    uint16_t stop_bits;     /**< PLAT_UART_STOP_BITS_x */
    uint16_t parity;        /**< PLAT_UART_PARITY_x */
    uint16_t flow_control;  /**< PLAT_UART_FLOW_CONTROL_x */
    uint16_t mode;          /**< PLAT_UART_MODE_TX | PLAT_UART_MODE_RX */
} platform_ch32v203_uart_config_t;

/**
 * @brief CH32V203 UART 操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 返回 bool，true 表示成功；取值类函数直接返回寄存器值。
 *          表中 is_tx 方向参数：true = 存储器→外设（发送通道），
 *          false = 外设→存储器（接收通道）；对应的通用 DMA 抽象通道号由
 *          platform_ch32v203_dma.h 的 pf_channel_get() 给出，不在本表内。
 * @note 本表不含任何"通用 DMA 通道"成员（装配/使能/长度/事件标志/优先级）：
 *       那部分统一由 platform_ch32v203_dma.h 的 plat_ch32v203_dma_ops 提供，
 *       本表只保留外设侧的 USART_DMACmd 与数据寄存器地址查询。
 */
typedef struct {
    /** @brief 使能 UART 实例所在总线的外设时钟。 */
    bool (*pf_clock_enable)(uintptr_t instance);
    /** @brief 复位 UART 外设（USART_DeInit）。 */
    bool (*pf_deinit)(uintptr_t instance);
    /** @brief 配置 UART 帧格式与波特率（USART_Init）。 */
    bool (*pf_config)(uintptr_t instance,
                      const platform_ch32v203_uart_config_t *p_config);
    /** @brief 使能/禁止 UART（USART_Cmd）。 */
    bool (*pf_cmd)(uintptr_t instance, bool enable);
    /** @brief 使能/禁止指定的 UART 中断源（USART_ITConfig）。 */
    bool (*pf_it_config)(uintptr_t instance, uint16_t it, bool enable);
    /**
     * @brief 配置并使能 UART 的中断线（PLAT_PFIC_INIT）。
     * @note 中断线由本层按实例映射（USART1/2/3 → USART1/2/3_IRQn），
     *       故入参为实例基地址，HAL 层不需要厂商中断号。
     */
    bool (*pf_irq_enable)(uintptr_t instance, uint8_t preemption_priority,
                          uint8_t sub_priority);
    /** @brief 使能/禁止 UART 的 DMA 传输请求（USART_DMACmd）。 */
    bool (*pf_dma_request_cmd)(uintptr_t instance, uint16_t request,
                               bool enable);
    /**
     * @brief 取 UART 数据寄存器（DATAR）地址，供上层拼通用 DMA 配置使用。
     * @param instance 实例基地址。
     * @return 数据寄存器地址（uintptr_t）；实例不受支持时返回 0。
     */
    uintptr_t (*pf_get_data_register_address)(uintptr_t instance);
    /** @brief 查询 UART 状态标志（PLAT_UART_FLAG_x）。 */
    bool (*pf_get_flag)(uintptr_t instance, uint16_t flag);
    /** @brief 清除 UART 状态标志（USART_ClearFlag）。 */
    bool (*pf_clear_flag)(uintptr_t instance, uint16_t flag);
    /** @brief 查询 UART 中断状态（USART_GetITStatus）。 */
    bool (*pf_get_it_status)(uintptr_t instance, uint16_t it);
    /**
     * @brief 清除 IDLE 空闲中断标志。
     * @details 顺序固定为"先读状态寄存器（USART_GetFlagStatus）再读数据寄存器
     *          （USART_ReceiveData）"，与参考实现一致，否则 IDLE 标志不会清除。
     */
    void (*pf_clear_idle)(uintptr_t instance);
    /** @brief 发送一个帧（USART_SendData）。 */
    bool (*pf_send_data)(uintptr_t instance, uint16_t data);
    /** @brief 接收一个帧（USART_ReceiveData）。 */
    uint16_t (*pf_receive_data)(uintptr_t instance);
} platform_ch32v203_uart_ops_t;

/** @brief CH32V203 UART 操作函数表指针（在 platform_ch32v203_uart.c 中组装）。 */
extern const platform_ch32v203_uart_ops_t *const plat_ch32v203_uart_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_UART_H__ */
