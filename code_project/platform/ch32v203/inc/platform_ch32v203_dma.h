/**
 * @file platform_ch32v203_dma.h
 * @brief CH32V203 DMA 的 MCU 抽象接口（取值宏 + 类型接入宏 + 通道映射 + 操作函数表）。
 * @details 与 GPIO/SPI/ADC/OPAMP/TIM/CAN/UART/I2C 平台层同构：
 *          - 取值宏：数据方向 / 外设与存储器地址自增 / 数据宽度 / 循环与普通模式 /
 *            软件优先级 / 存储器到存储器；
 *          - 类型接入宏：PLAT_DMA_CHANNEL_INSTANCE_TYPE / PLAT_DMA_INIT_TYPE，把厂商
 *            类型名收敛到宏体中（延迟展开），使 platform_ch32v203_dma.c 内只出现
 *            PLAT_* 名字；
 *          - 抽象通道号 platform_ch32v203_dma_channel_enum（CH1..CH7）与
 *            事件类别 platform_ch32v203_dma_event_enum（GL/TC/HT/TE），
 *            厂商的 DMA1_Channelx 指针、DMA1_FLAG_xxxn / DMA1_IT_xxxn、
 *            DMA1_Channelx_IRQn 一律不出本层；
 *          - 外设→通道映射由 pf_channel_get() 统一给出（片上 DMA 请求映射唯一的
 *            CH32V203 事实：ADC1 规则=CH1，USART1 TX/RX=CH4/CH5，USART2 TX/RX=CH7/CH6，
 *            USART3 TX/RX=CH2/CH3，I2C1 TX/RX=CH6/CH7）；
 *          - 操作函数表 platform_ch32v203_dma_ops_t：把标准库 DMA 接口
 *            （DMA_DeInit / DMA_Init / DMA_Cmd / DMA_SetCurrDataCounter /
 *            DMA_GetCurrDataCounter / DMA_GetFlagStatus / DMA_ClearFlag /
 *            DMA_ITConfig / DMA_GetITStatus / DMA_ClearITPendingBit /
 *            DMA1 时钟使能 / PLAT_PFIC_INIT 中断线配置）逐个做简单函数封装后绑定，
 *            并通过 plat_ch32v203_dma_ops 指针对外外露（表本体私有）。
 * @note 本层不提供任何业务语义（ADC 采样序列、UART 收帧、I2C 时序等），只做 MCU 抽象；
 *       各外设平台层只保留"外设侧"的 DMA 相关操作（如 USART_DMACmd / I2C_DMACmd /
 *       ADC_DMACmd）与"外设数据寄存器地址"查询，通道配置与通道状态全部归本层。
 * @note 本头会被各 HAL 头间接包含，而 hal.h 可能在 ch32v20x_conf.h 之前解析到本头，
 *       因此本头内**不得出现任何厂商类型**（DMA_InitTypeDef、DMA_Channel_TypeDef 等）；
 *       结构体一律使用 <stdint.h>/<stdbool.h> 的纯 C 类型，厂商符号只允许出现在
 *       宏体中（延迟展开）以及 platform_ch32v203_dma.c 内。
 */
#ifndef __PLATFORM_CH32V203_DMA_H__
#define __PLATFORM_CH32V203_DMA_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/* --------------------------------------------------------------------------
 * 1) 取值宏：标准库 DMA 初始化参数的可移植映射
 * -------------------------------------------------------------------------- */

/** @brief DMA 数据方向（数据从哪搬到哪）。 */
#define PLAT_DMA_DIR_PERIPHERAL_TO_MEMORY      DMA_DIR_PeripheralSRC
#define PLAT_DMA_DIR_MEMORY_TO_PERIPHERAL      DMA_DIR_PeripheralDST

/** @brief DMA 外设地址是否自增。 */
#define PLAT_DMA_PERIPHERAL_INCREMENT_DISABLE  DMA_PeripheralInc_Disable
#define PLAT_DMA_PERIPHERAL_INCREMENT_ENABLE   DMA_PeripheralInc_Enable

/** @brief DMA 存储器地址是否自增。 */
#define PLAT_DMA_MEMORY_INCREMENT_DISABLE      DMA_MemoryInc_Disable
#define PLAT_DMA_MEMORY_INCREMENT_ENABLE       DMA_MemoryInc_Enable

/** @brief DMA 外设侧数据宽度。 */
#define PLAT_DMA_PERIPHERAL_DATA_SIZE_BYTE     DMA_PeripheralDataSize_Byte
#define PLAT_DMA_PERIPHERAL_DATA_SIZE_HALFWORD DMA_PeripheralDataSize_HalfWord
#define PLAT_DMA_PERIPHERAL_DATA_SIZE_WORD     DMA_PeripheralDataSize_Word

/** @brief DMA 存储器侧数据宽度。 */
#define PLAT_DMA_MEMORY_DATA_SIZE_BYTE         DMA_MemoryDataSize_Byte
#define PLAT_DMA_MEMORY_DATA_SIZE_HALFWORD     DMA_MemoryDataSize_HalfWord
#define PLAT_DMA_MEMORY_DATA_SIZE_WORD         DMA_MemoryDataSize_Word

/** @brief DMA 传输模式。 */
#define PLAT_DMA_MODE_NORMAL                   DMA_Mode_Normal
#define PLAT_DMA_MODE_CIRCULAR                 DMA_Mode_Circular

/** @brief DMA 通道软件优先级。 */
#define PLAT_DMA_PRIORITY_LOW                  DMA_Priority_Low
#define PLAT_DMA_PRIORITY_MEDIUM               DMA_Priority_Medium
#define PLAT_DMA_PRIORITY_HIGH                 DMA_Priority_High
#define PLAT_DMA_PRIORITY_VERY_HIGH            DMA_Priority_VeryHigh

/** @brief 存储器到存储器传输开关。 */
#define PLAT_DMA_MEMORY_TO_MEMORY_DISABLE      DMA_M2M_Disable
#define PLAT_DMA_MEMORY_TO_MEMORY_ENABLE       DMA_M2M_Enable

/* --------------------------------------------------------------------------
 * 2) 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_dma.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_DMA_CHANNEL_INSTANCE_TYPE     DMA_Channel_TypeDef
#define PLAT_DMA_INIT_TYPE                 DMA_InitTypeDef

/* --------------------------------------------------------------------------
 * 3) 抽象通道号与事件类别
 * -------------------------------------------------------------------------- */

/**
 * @brief DMA 抽象通道号。
 * @note CH32V203 只有 DMA1（CH1..CH7）；保留 CH8 位序但不提供，便于将来移植。
 *       0 号保留为"无/非法通道"，便于 pf_channel_get() 用 0 表达"无映射"。
 */
typedef enum {
    platform_ch32v203_dma_channel_none = 0,
    platform_ch32v203_dma_channel_1 = 1,
    platform_ch32v203_dma_channel_2 = 2,
    platform_ch32v203_dma_channel_3 = 3,
    platform_ch32v203_dma_channel_4 = 4,
    platform_ch32v203_dma_channel_5 = 5,
    platform_ch32v203_dma_channel_6 = 6,
    platform_ch32v203_dma_channel_7 = 7
} platform_ch32v203_dma_channel_enum;

/**
 * @brief DMA 事件（标志 / 中断源）类别。
 * @details 取值即"每通道 4 位分组内的位序号"：
 *          GL = bit0、TC = bit1、HT = bit2、TE = bit3；
 *          平台实现按 (通道号, 类别) 组合出 DMA1_FLAG_xxxn / DMA1_IT_xxxn。
 */
typedef enum {
    platform_ch32v203_dma_event_global = 0,             /**< 全局标志 GL */
    platform_ch32v203_dma_event_transfer_complete = 1,  /**< 传输完成 TC */
    platform_ch32v203_dma_event_half_transfer = 2,      /**< 半传输 HT */
    platform_ch32v203_dma_event_transfer_error = 3      /**< 传输错误 TE */
} platform_ch32v203_dma_event_enum;

/**
 * @brief 外设→DMA 通道映射的请求方标识（CH32V203 的 DMA 请求映射是固定的）。
 * @note is_tx 的含义按请求方解释：USART/I2C 表示发送/接收方向，
 *       ADC 规则通道只有一条通道，is_tx 忽略。
 */
typedef enum {
    platform_ch32v203_dma_peripheral_adc1 = 0,   /**< ADC1 规则通道（CH1） */
    platform_ch32v203_dma_peripheral_usart1,     /**< USART1：TX=CH4 / RX=CH5 */
    platform_ch32v203_dma_peripheral_usart2,     /**< USART2：TX=CH7 / RX=CH6 */
    platform_ch32v203_dma_peripheral_usart3,     /**< USART3：TX=CH2 / RX=CH3 */
    platform_ch32v203_dma_peripheral_i2c1        /**< I2C1：TX=CH6 / RX=CH7 */
} platform_ch32v203_dma_peripheral_enum;

/* --------------------------------------------------------------------------
 * 4) 配置结构体与操作函数表
 * -------------------------------------------------------------------------- */

/** @brief DMA 通道配置（对应标准库 DMA_InitTypeDef 的可移植子集）。 */
typedef struct {
    uintptr_t peripheral_address;   /**< 外设数据寄存器地址 */
    uintptr_t memory_address;       /**< 存储器缓冲区地址 */
    uint32_t direction;             /**< PLAT_DMA_DIR_x */
    uint16_t buffer_size;           /**< 待传输数据项个数（1..65535） */
    uint32_t peripheral_increment;  /**< PLAT_DMA_PERIPHERAL_INCREMENT_x */
    uint32_t memory_increment;      /**< PLAT_DMA_MEMORY_INCREMENT_x */
    uint32_t peripheral_data_size;  /**< PLAT_DMA_PERIPHERAL_DATA_SIZE_x */
    uint32_t memory_data_size;      /**< PLAT_DMA_MEMORY_DATA_SIZE_x */
    uint32_t mode;                  /**< PLAT_DMA_MODE_x */
    uint32_t priority;              /**< PLAT_DMA_PRIORITY_x */
    uint32_t memory_to_memory;      /**< PLAT_DMA_MEMORY_TO_MEMORY_x */
} platform_ch32v203_dma_config_t;

/**
 * @brief CH32V203 DMA 操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 返回 bool 的成员一律 true 表示成功、false 表示参数非法或通道不受支持；
 *          取值类成员直接返回寄存器的值。
 */
typedef struct {
    /** @brief 使能 DMA1 时钟（AHB）。 */
    bool (*pf_clock_enable)(void);

    /**
     * @brief 取某外设某方向的 DMA 抽象通道号。
     * @param peripheral 请求方，取 platform_ch32v203_dma_peripheral_x。
     * @param is_tx true 取发送方向通道，false 取接收方向通道（ADC 忽略）。
     * @return 通道号 platform_ch32v203_dma_channel_x；无映射时返回 channel_none。
     */
    uint8_t (*pf_channel_get)(uint8_t peripheral, bool is_tx);

    /** @brief 复位 DMA 通道（DMA_DeInit）。 */
    bool (*pf_deinit)(uint8_t channel);

    /** @brief 装配 DMA 通道（DMA_Init）。 */
    bool (*pf_config)(uint8_t channel,
                      const platform_ch32v203_dma_config_t *p_config);

    /** @brief 使能/禁止 DMA 通道（DMA_Cmd）。 */
    bool (*pf_cmd)(uint8_t channel, bool enable);

    /** @brief 设置 DMA 存储器地址（重复传输前改缓冲区用）。 */
    bool (*pf_set_memory_address)(uint8_t channel, uintptr_t memory_address);

    /** @brief 设置待传输数据项个数（DMA_SetCurrDataCounter）。 */
    bool (*pf_set_data_length)(uint8_t channel, uint16_t data_length);

    /** @brief 读剩余待传输数据项个数（DMA_GetCurrDataCounter）。 */
    uint16_t (*pf_get_data_length)(uint8_t channel);

    /** @brief 查询 DMA 事件标志（DMA_GetFlagStatus），event 取 PLAT_DMA_EVENT_x。 */
    bool (*pf_get_event)(uint8_t channel, uint8_t event);

    /** @brief 清除 DMA 事件标志（DMA_ClearFlag）。 */
    bool (*pf_clear_event)(uint8_t channel, uint8_t event);

    /** @brief 使能/禁止 DMA 事件中断（DMA_ITConfig）。 */
    bool (*pf_it_config)(uint8_t channel, uint8_t event, bool enable);

    /** @brief 查询 DMA 事件中断状态（DMA_GetITStatus）。 */
    bool (*pf_get_it_status)(uint8_t channel, uint8_t event);

    /** @brief 清除 DMA 事件中断挂起位（DMA_ClearITPendingBit）。 */
    bool (*pf_clear_it_pending)(uint8_t channel, uint8_t event);

    /** @brief 配置 DMA 通道中断线（PLAT_PFIC_INIT），通道→中断线号由本层映射。 */
    bool (*pf_irq_enable)(uint8_t channel, uint8_t preemption_priority,
                          uint8_t sub_priority);
} platform_ch32v203_dma_ops_t;

/** @brief 当前模块操作表（表本体私有，仅此指针外露）。 */
extern const platform_ch32v203_dma_ops_t *const plat_ch32v203_dma_ops;

#endif /* __PLATFORM_CH32V203_DMA_H__ */
