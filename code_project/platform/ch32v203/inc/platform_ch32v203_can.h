/**
 * @file platform_ch32v203_can.h
 * @brief CH32V203 CAN 的 MCU 抽象接口（取值宏 + 类型接入宏 + 操作函数表）。
 * @details 与 GPIO/ADC/SPI 平台层同构：
 *          - 取值宏：工作模式 / 工作模式请求 / 位时序（SJW、BS1、BS2、预分频）/
 *            过滤器模式与尺度 / 过滤器 FIFO / 帧标识与帧类型 / 发送邮箱状态 /
 *            接收 FIFO / 标志 / 中断源；
 *          - 操作函数表 platform_ch32v203_can_ops_t：把标准库 CAN 接口
 *            （时钟使能、CAN_DeInit、CAN_Init、CAN_FilterInit、
 *            CAN_OperatingModeRequest、CAN_Transmit、CAN_TransmitStatus、
 *            CAN_CancelTransmit、CAN_Receive、CAN_MessagePending、
 *            CAN_FIFORelease、CAN_ITConfig、CAN_GetFlagStatus、CAN_ClearFlag、
 *            CAN_GetITStatus、CAN_ClearITPendingBit、CAN_GetLastErrorCode）
 *            逐个做简单函数封装后绑定，并通过 plat_ch32v203_can_ops 指针对外外露。
 *          CAN 的对象模型、板级实例（CAN1）与收发流程实现于 hal_can.c/h。
 * @note 本层不提供任何业务语义（报文含义、周期发送、协议解析等），只做 MCU 抽象。
 * @note 本头文件只使用纯 C 类型、取值宏与类型接入宏：厂商类型名只允许出现在宏体中
 *       （宏在使用处才展开，故宏体引用 CAN_xxx 是安全的），严禁以类型名形式直接出现
 *       （CAN_TypeDef / CAN_InitTypeDef / CAN_FilterInitTypeDef / CanTxMsg / CanRxMsg）：
 *       本头可能经 hal.h 在 ch32v20x_conf.h 之前被解析，出现厂商类型会报 unknown type name。
 *       厂商类型只经下方"类型接入宏"在 platform_ch32v203_can.c 内以 PLAT_* 名字使用。
 */
#ifndef __PLATFORM_CH32V203_CAN_H__
#define __PLATFORM_CH32V203_CAN_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/** @brief CAN 工作模式（CAN_Mode_x）。 */
#define PLAT_CAN_MODE_NORMAL                   CAN_Mode_Normal
#define PLAT_CAN_MODE_LOOPBACK                 CAN_Mode_LoopBack
#define PLAT_CAN_MODE_SILENT                   CAN_Mode_Silent
#define PLAT_CAN_MODE_SILENT_LOOPBACK          CAN_Mode_Silent_LoopBack

/**
 * @brief CAN 工作模式请求（CAN_OperatingMode_x）。
 * @note CH32V20x 标准库没有 CAN_Cmd：外设的"启动/停止"由本接口表达，
 *       NORMAL = 启动（退出初始化与睡眠）、SLEEP = 停止（进入睡眠）、
 *       INITIALIZATION = 进入初始化模式。
 */
#define PLAT_CAN_OPERATING_MODE_INITIALIZATION CAN_OperatingMode_Initialization
#define PLAT_CAN_OPERATING_MODE_NORMAL         CAN_OperatingMode_Normal
#define PLAT_CAN_OPERATING_MODE_SLEEP          CAN_OperatingMode_Sleep

/** @brief CAN 初始化返回状态（CAN_Init 的返回值）。 */
#define PLAT_CAN_INIT_STATUS_FAILED            CAN_InitStatus_Failed
#define PLAT_CAN_INIT_STATUS_SUCCESS           CAN_InitStatus_Success

/** @brief CAN 重新同步跳变宽度（CAN_SJW_x）。 */
#define PLAT_CAN_SJW_1TQ                       CAN_SJW_1tq
#define PLAT_CAN_SJW_2TQ                       CAN_SJW_2tq
#define PLAT_CAN_SJW_3TQ                       CAN_SJW_3tq
#define PLAT_CAN_SJW_4TQ                       CAN_SJW_4tq

/** @brief CAN 位段 1（CAN_BS1_x）。 */
#define PLAT_CAN_BS1_1TQ                       CAN_BS1_1tq
#define PLAT_CAN_BS1_2TQ                       CAN_BS1_2tq
#define PLAT_CAN_BS1_3TQ                       CAN_BS1_3tq
#define PLAT_CAN_BS1_4TQ                       CAN_BS1_4tq
#define PLAT_CAN_BS1_5TQ                       CAN_BS1_5tq
#define PLAT_CAN_BS1_6TQ                       CAN_BS1_6tq
#define PLAT_CAN_BS1_7TQ                       CAN_BS1_7tq
#define PLAT_CAN_BS1_8TQ                       CAN_BS1_8tq
#define PLAT_CAN_BS1_9TQ                       CAN_BS1_9tq
#define PLAT_CAN_BS1_10TQ                      CAN_BS1_10tq
#define PLAT_CAN_BS1_11TQ                      CAN_BS1_11tq
#define PLAT_CAN_BS1_12TQ                      CAN_BS1_12tq
#define PLAT_CAN_BS1_13TQ                      CAN_BS1_13tq
#define PLAT_CAN_BS1_14TQ                      CAN_BS1_14tq
#define PLAT_CAN_BS1_15TQ                      CAN_BS1_15tq
#define PLAT_CAN_BS1_16TQ                      CAN_BS1_16tq

/** @brief CAN 位段 2（CAN_BS2_x）。 */
#define PLAT_CAN_BS2_1TQ                       CAN_BS2_1tq
#define PLAT_CAN_BS2_2TQ                       CAN_BS2_2tq
#define PLAT_CAN_BS2_3TQ                       CAN_BS2_3tq
#define PLAT_CAN_BS2_4TQ                       CAN_BS2_4tq
#define PLAT_CAN_BS2_5TQ                       CAN_BS2_5tq
#define PLAT_CAN_BS2_6TQ                       CAN_BS2_6tq
#define PLAT_CAN_BS2_7TQ                       CAN_BS2_7tq
#define PLAT_CAN_BS2_8TQ                       CAN_BS2_8tq

/** @brief CAN 过滤器模式（CAN_FilterMode_x）。 */
#define PLAT_CAN_FILTER_MODE_ID_MASK           CAN_FilterMode_IdMask
#define PLAT_CAN_FILTER_MODE_ID_LIST           CAN_FilterMode_IdList

/** @brief CAN 过滤器尺度（CAN_FilterScale_x）。 */
#define PLAT_CAN_FILTER_SCALE_16BIT            CAN_FilterScale_16bit
#define PLAT_CAN_FILTER_SCALE_32BIT            CAN_FilterScale_32bit

/** @brief CAN 过滤器关联的接收 FIFO（CAN_Filter_FIFO_x）。 */
#define PLAT_CAN_FILTER_FIFO_0                 CAN_Filter_FIFO0
#define PLAT_CAN_FILTER_FIFO_1                 CAN_Filter_FIFO1

/** @brief CAN 接收 FIFO 编号（CAN_FIFO0 与 CAN_Filter_FIFO0 同值，可直接互用）。 */
#define PLAT_CAN_FIFO_0                        CAN_FIFO0
#define PLAT_CAN_FIFO_1                        CAN_FIFO1

/** @brief CAN 标识符类型（CAN_Id_x）。 */
#define PLAT_CAN_ID_STANDARD                   CAN_Id_Standard
#define PLAT_CAN_ID_EXTENDED                   CAN_Id_Extended

/** @brief CAN 帧类型（CAN_RTR_x）。 */
#define PLAT_CAN_RTR_DATA                      CAN_RTR_Data
#define PLAT_CAN_RTR_REMOTE                    CAN_RTR_Remote

/** @brief CAN 发送邮箱状态（CAN_TransmitStatus 的返回值）。 */
#define PLAT_CAN_TX_STATUS_FAILED              CAN_TxStatus_Failed
#define PLAT_CAN_TX_STATUS_OK                  CAN_TxStatus_Ok
#define PLAT_CAN_TX_STATUS_PENDING             CAN_TxStatus_Pending
#define PLAT_CAN_TX_STATUS_NO_MAILBOX          CAN_TxStatus_NoMailBox

/** @brief CAN 状态标志（CAN_FLAG_x，供 pf_get_flag/pf_clear_flag 使用）。 */
#define PLAT_CAN_FLAG_RQCP0                    CAN_FLAG_RQCP0
#define PLAT_CAN_FLAG_RQCP1                    CAN_FLAG_RQCP1
#define PLAT_CAN_FLAG_RQCP2                    CAN_FLAG_RQCP2
#define PLAT_CAN_FLAG_FMP0                     CAN_FLAG_FMP0
#define PLAT_CAN_FLAG_FF0                      CAN_FLAG_FF0
#define PLAT_CAN_FLAG_FOV0                     CAN_FLAG_FOV0
#define PLAT_CAN_FLAG_FMP1                     CAN_FLAG_FMP1
#define PLAT_CAN_FLAG_FF1                      CAN_FLAG_FF1
#define PLAT_CAN_FLAG_FOV1                     CAN_FLAG_FOV1
#define PLAT_CAN_FLAG_WKU                      CAN_FLAG_WKU
#define PLAT_CAN_FLAG_SLAK                     CAN_FLAG_SLAK
#define PLAT_CAN_FLAG_EWG                      CAN_FLAG_EWG
#define PLAT_CAN_FLAG_EPV                      CAN_FLAG_EPV
#define PLAT_CAN_FLAG_BOF                      CAN_FLAG_BOF
#define PLAT_CAN_FLAG_LEC                      CAN_FLAG_LEC

/** @brief CAN 中断源（CAN_IT_x，供 pf_it_config/pf_get_it_status 使用）。 */
#define PLAT_CAN_IT_TME                        CAN_IT_TME
#define PLAT_CAN_IT_FMP0                       CAN_IT_FMP0
#define PLAT_CAN_IT_FF0                        CAN_IT_FF0
#define PLAT_CAN_IT_FOV0                       CAN_IT_FOV0
#define PLAT_CAN_IT_FMP1                       CAN_IT_FMP1
#define PLAT_CAN_IT_FF1                        CAN_IT_FF1
#define PLAT_CAN_IT_FOV1                       CAN_IT_FOV1
#define PLAT_CAN_IT_WKU                        CAN_IT_WKU
#define PLAT_CAN_IT_SLK                        CAN_IT_SLK
#define PLAT_CAN_IT_EWG                        CAN_IT_EWG
#define PLAT_CAN_IT_EPV                        CAN_IT_EPV
#define PLAT_CAN_IT_BOF                        CAN_IT_BOF
#define PLAT_CAN_IT_LEC                        CAN_IT_LEC
#define PLAT_CAN_IT_ERR                        CAN_IT_ERR

/** @brief CAN 中断线（供 pf_irq_enable 使用）。 */
#define PLAT_CAN_IRQ_RX0                       PLAT_IRQ_CAN1_RX0
#define PLAT_CAN_IRQ_TX                        PLAT_IRQ_CAN1_TX

/** @brief CAN 报文最大数据长度（字节）。 */
#define PLAT_CAN_DATA_LENGTH_MAX               (8U)

/** @brief 标准帧标识符上限（11 位）。 */
#define PLAT_CAN_STD_ID_MAX                    (0x7FFUL)

/** @brief 扩展帧标识符上限（29 位）。 */
#define PLAT_CAN_EXT_ID_MAX                    (0x1FFFFFFFUL)

/** @brief 发送邮箱数量。 */
#define PLAT_CAN_MAILBOX_COUNT                 (3U)

/** @brief 无效/失败的发送邮箱号（平台层内部约定，区别于标准库的 CAN_TxStatus_NoMailBox）。 */
#define PLAT_CAN_MAILBOX_NONE                  (0xFFU)

/** @brief 过滤器组编号上限（CH32V20x 共 14 组，0~13）。 */
#define PLAT_CAN_FILTER_NUMBER_MAX             (13U)

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_can.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_CAN_INSTANCE_TYPE             CAN_TypeDef
#define PLAT_CAN_INIT_TYPE                 CAN_InitTypeDef
#define PLAT_CAN_FILTER_INIT_TYPE          CAN_FilterInitTypeDef
#define PLAT_CAN_TX_MESSAGE_TYPE           CanTxMsg
#define PLAT_CAN_RX_MESSAGE_TYPE           CanRxMsg

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN 位时序与工作模式配置（对应标准库 CAN_InitTypeDef 的可移植子集）。 */
typedef struct {
    uint16_t prescaler;                     /**< 波特率预分频，1..1024 */
    uint8_t mode;                           /**< PLAT_CAN_MODE_x */
    uint8_t sync_jump_width;                /**< PLAT_CAN_SJW_x */
    uint8_t bit_segment1;                   /**< PLAT_CAN_BS1_x */
    uint8_t bit_segment2;                   /**< PLAT_CAN_BS2_x */
    bool is_time_trigger_enabled;           /**< 时间触发通信模式（TTCM） */
    bool is_auto_bus_off_enabled;           /**< 自动离线管理（ABOM） */
    bool is_auto_wakeup_enabled;            /**< 自动唤醒（AWUM） */
    bool is_no_auto_retransmission_enabled; /**< 禁止自动重传（NART） */
    bool is_receive_fifo_locked_enabled;    /**< 接收 FIFO 锁定（RFLM） */
    bool is_transmit_fifo_priority_enabled; /**< 发送 FIFO 优先级（TXFP） */
} platform_ch32v203_can_config_t;

/** @brief CAN 过滤器配置（对应标准库 CAN_FilterInitTypeDef 的可移植子集）。 */
typedef struct {
    uint16_t filter_number;        /**< 过滤器组编号，0..PLAT_CAN_FILTER_NUMBER_MAX */
    uint8_t mode;                  /**< PLAT_CAN_FILTER_MODE_x */
    uint8_t scale;                 /**< PLAT_CAN_FILTER_SCALE_x */
    uint16_t id_high;              /**< 标识符高 16 位（32 位尺度时为 STDID/EXID 高位） */
    uint16_t id_low;               /**< 标识符低 16 位 */
    uint16_t mask_id_high;         /**< 掩码/标识符列表高 16 位 */
    uint16_t mask_id_low;          /**< 掩码/标识符列表低 16 位 */
    uint16_t fifo_assignment;      /**< PLAT_CAN_FILTER_FIFO_x */
    bool is_activated;             /**< true 激活该过滤器组 */
} platform_ch32v203_can_filter_config_t;

/** @brief CAN 报文（对应标准库 CanTxMsg/CanRxMsg 的可移植子集）。 */
typedef struct {
    uint32_t identifier;           /**< 标准帧 STDID（0..0x7FF）或扩展帧 EXTID */
    uint8_t data_length;           /**< 数据长度，0..PLAT_CAN_DATA_LENGTH_MAX */
    uint8_t data[PLAT_CAN_DATA_LENGTH_MAX]; /**< 数据字节 */
    bool is_extended;              /**< true 扩展帧，false 标准帧 */
    bool is_remote;                /**< true 远程帧，false 数据帧 */
    uint8_t filter_index;          /**< 接收时有效：命中过滤器组编号（FMI） */
} platform_ch32v203_can_message_t;

/**
 * @brief CH32V203 CAN 操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 返回 bool，true 表示成功；取值类函数直接返回标准库的取值
 *          （邮箱号、邮箱状态、待处理报文数、错误码）。
 * @note 过滤器为 CAN1/CAN2 共享资源，故 pf_filter_config 的 instance 仅用于
 *       实例合法性校验，真正写入的是共享过滤器组。
 */
typedef struct {
    /** @brief 使能 CAN 实例时钟（APB1）。 */
    bool (*pf_clock_enable)(uintptr_t instance);
    /** @brief 复位 CAN 外设（CAN_DeInit）。 */
    bool (*pf_deinit)(uintptr_t instance);
    /** @brief 配置位时序与工作模式（CAN_Init，返回 CAN_InitStatus_Success 视为成功）。 */
    bool (*pf_config)(uintptr_t instance,
                      const platform_ch32v203_can_config_t *p_config);
    /** @brief 配置过滤器组（CAN_FilterInit）。 */
    bool (*pf_filter_config)(uintptr_t instance,
                             const platform_ch32v203_can_filter_config_t *p_config);
    /** @brief 请求工作模式（CAN_OperatingModeRequest，启动/停止/进入初始化）。 */
    bool (*pf_operating_mode_request)(uintptr_t instance, uint8_t operating_mode);
    /** @brief 使能/禁止指定的 CAN 中断源（CAN_ITConfig）。 */
    bool (*pf_it_config)(uintptr_t instance, uint32_t it, bool enable);
    /** @brief 配置并使能 CAN 中断线（RX0 或 TX）。 */
    bool (*pf_irq_enable)(uint32_t irq_channel, uint8_t preemption_priority,
                          uint8_t sub_priority);
    /**
     * @brief 请求发送一帧（CAN_Transmit）。
     * @return 邮箱号 0..2；无空闲邮箱时返回 PLAT_CAN_TX_STATUS_NO_MAILBOX；
     *         实例或报文非法时返回 PLAT_CAN_MAILBOX_NONE。
     */
    uint8_t (*pf_transmit)(uintptr_t instance,
                           const platform_ch32v203_can_message_t *p_message);
    /** @brief 查询邮箱发送状态（CAN_TransmitStatus），返回 PLAT_CAN_TX_STATUS_x。 */
    uint8_t (*pf_transmit_status)(uintptr_t instance, uint8_t mailbox);
    /** @brief 取消指定邮箱的发送请求（CAN_CancelTransmit）。 */
    bool (*pf_cancel_transmit)(uintptr_t instance, uint8_t mailbox);
    /** @brief 从指定 FIFO 读取一帧（CAN_Receive，读取即释放 FIFO 空间）。 */
    bool (*pf_receive)(uintptr_t instance, uint8_t fifo_number,
                       platform_ch32v203_can_message_t *p_message);
    /** @brief 查询指定 FIFO 中待处理的报文数（CAN_MessagePending）。 */
    uint8_t (*pf_message_pending)(uintptr_t instance, uint8_t fifo_number);
    /** @brief 释放指定 FIFO（CAN_FIFORelease）。 */
    bool (*pf_fifo_release)(uintptr_t instance, uint8_t fifo_number);
    /** @brief 查询状态标志（PLAT_CAN_FLAG_x）。 */
    bool (*pf_get_flag)(uintptr_t instance, uint32_t flag);
    /** @brief 清除状态标志（PLAT_CAN_FLAG_x）。 */
    bool (*pf_clear_flag)(uintptr_t instance, uint32_t flag);
    /** @brief 查询中断状态（PLAT_CAN_IT_x），true 表示挂起。 */
    bool (*pf_get_it_status)(uintptr_t instance, uint32_t it);
    /** @brief 清除中断挂起位（PLAT_CAN_IT_x）。 */
    bool (*pf_clear_it_pending_bit)(uintptr_t instance, uint32_t it);
    /** @brief 读最近一次错误码（CAN_GetLastErrorCode）。 */
    uint8_t (*pf_get_last_error_code)(uintptr_t instance);
} platform_ch32v203_can_ops_t;

/** @brief CH32V203 CAN 操作函数表指针（在 platform_ch32v203_can.c 中组装）。 */
extern const platform_ch32v203_can_ops_t *const plat_ch32v203_can_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_CAN_H__ */
