/**
 * @file bsp_comm_mgr.h
 * @brief 通信链路 BSP 接口（VOFA+ JustFloat 发送 + 接收环形队列与整帧提取）。
 * @details 层级依赖：本层只经 HAL 层接口（hal_uart_ops 函数表）访问硬件，不出现任何
 *          厂商库符号与平台细节；对象持有状态、配置副本、发送组帧缓冲与接收环形队列，
 *          板级常量与实例由本层给出。
 *          发送（JustFloat 帧格式，VOFA+ 协议引擎，对应插件页 justfloat）：
 *          - 帧 = N 个 小端 IEEE-754 单精度 float（每通道 4 字节）
 *                + 4 字节帧尾 0x00 0x00 0x80 0x7F；
 *          - 帧尾按小端解析即 0x7F800000（+Inf），上位机以该值作为"一帧结束"的哨兵；
 *          - 通道数 N 由上位机 JustFloat 控件设置，须与调用时给出的通道数一致；
 *          - 一次 bsp_comm_mgr_send_justfloat() 调用发送完整一帧。
 *          接收（参考实现 drv_uart.c 的"元数据环 + 整帧拷贝"模型，由 HAL 层迁入）：
 *          - 接收 DMA 直接写入本层环形缓冲 dma_buffer，帧边界由 USART 空闲（IDLE）
 *            中断界定：HAL 层的 hal_uart_rx_idle_handler() 清标志并算出 DMA 写位置，
 *            经回调进入 bsp_comm_mgr_rx_idle_handler() 写入帧元数据环；
 *          - bsp_comm_mgr_get_frame() 把最早一帧拷进整帧缓冲并给出长度。
 * @note 目标 MCU（CH32V203/RV32）为小端，float 按内存原样搬运即为协议要求的字节序，
 *       组帧不做字节序转换（见 bsp_comm_mgr.c 中的编译期尺寸断言）。
 */
#ifndef __BSP_COMM_MGR_H__
#define __BSP_COMM_MGR_H__

#include <stdint.h>

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * JustFloat 协议常量
 * ========================================================================== */

/** @brief JustFloat 单通道字节数（小端 IEEE-754 单精度浮点）。 */
#define BSP_COMM_MGR_JUSTFLOAT_CHANNEL_SIZE   (4U)

/** @brief JustFloat 帧尾字节数。 */
#define BSP_COMM_MGR_JUSTFLOAT_TAIL_SIZE      (4U)

/** @brief JustFloat 支持的最大通道数（决定组帧缓冲大小，按需调整）。 */
#define BSP_COMM_MGR_JUSTFLOAT_MAX_CHANNELS   (8U)

/** @brief JustFloat 组帧缓冲字节数（N 个通道 + 帧尾）。 */
#define BSP_COMM_MGR_JUSTFLOAT_FRAME_SIZE                    \
    ((BSP_COMM_MGR_JUSTFLOAT_MAX_CHANNELS *                  \
      BSP_COMM_MGR_JUSTFLOAT_CHANNEL_SIZE) +                 \
     BSP_COMM_MGR_JUSTFLOAT_TAIL_SIZE)

/* ==========================================================================
 * 接收环形队列常量
 * ==========================================================================
 * 取值依据：参考实现的 USART3_DMA_BUFFER_SIZE / USART3_RING_BUFFER_COUNT 定义在
 * drv_uart.c 之外的工程头里，未随参考实现给出，故取值由本工程确定；VOFA 命令均为
 * 短 ASCII 字符串，64 字节足以容纳一帧。
 */

/** @brief 接收 DMA 环形缓冲字节数（DMA 直接写入，长度即环形队列长度）。 */
#define BSP_COMM_MGR_RX_DMA_BUFFER_SIZE    (64U)

/** @brief 帧元数据环深度（可暂存的待取完整帧数，参考实现 USART3_RING_BUFFER_COUNT）。 */
#define BSP_COMM_MGR_RX_FRAME_COUNT        (8U)

/** @brief 整帧提取缓冲字节数。 */
#define BSP_COMM_MGR_RX_FRAME_BUFFER_SIZE  (BSP_COMM_MGR_RX_DMA_BUFFER_SIZE)

/* ==========================================================================
 * 类型定义
 * ========================================================================== */

/** @brief 通信对象的生命周期状态。 */
typedef enum {
    bsp_comm_mgr_state_reset = 0,
    bsp_comm_mgr_state_ready,
    bsp_comm_mgr_state_enabled,
    bsp_comm_mgr_state_disabled,
    bsp_comm_mgr_state_error
} bsp_comm_mgr_state_enum;

/** @brief 通信接口调用返回的状态码（0 成功，负数错误码）。 */
typedef enum {
    bsp_comm_mgr_ok = 0,
    bsp_comm_mgr_err_param = -1,
    bsp_comm_mgr_err_state = -2,
    bsp_comm_mgr_err_unsupported = -3,
    bsp_comm_mgr_err_timeout = -4
} bsp_comm_mgr_status_enum;

/** @brief 通信端角色类型。 */
typedef enum {
    bsp_comm_mgr_role_debug = 0,
    bsp_comm_mgr_role_remote
} bsp_comm_mgr_role_enum;

/** @brief 通信承载介质类型。 */
typedef enum {
    bsp_comm_mgr_transport_uart = 0,
    bsp_comm_mgr_transport_can,
    bsp_comm_mgr_transport_usb,
    bsp_comm_mgr_transport_custom
} bsp_comm_mgr_transport_enum;

/** @brief 通信角色、承载介质与单帧长度上限配置。 */
typedef struct {
    bsp_comm_mgr_role_enum role;           /**< 链路角色 */
    bsp_comm_mgr_transport_enum transport; /**< 承载介质 */
    uint16_t maximum_payload_length;       /**< 单帧最大字节数（含协议帧尾） */
    uint32_t timeout_count;                /**< 保留：超时由 HAL UART 自身配置负责 */
    void *p_user_data;                     /**< 保留：用户上下文 */
} bsp_comm_mgr_config_t;

/** @brief 一帧数据在接收 DMA 环形缓冲中的位置与长度（参考实现 USART_RXBUFF_INFO_T）。 */
typedef struct {
    uint16_t start_offset;  /**< 帧在环形缓冲中的起始偏移 */
    uint16_t length;        /**< 帧长度（字节） */
} bsp_comm_mgr_frame_info_t;

/**
 * @brief 接收句柄（参考实现 USART_DMA_SYSTEM_T 中与接收有关的部分）。
 * @details DMA 环形缓冲 + 帧元数据环 + 整帧缓冲 + 游标：dma_buffer 由接收 DMA
 *          直接写入，其余字段由本层在 IDLE 中断与其后的取帧流程中维护。
 */
typedef struct {
    uint8_t dma_buffer[BSP_COMM_MGR_RX_DMA_BUFFER_SIZE];  /**< 接收 DMA 环形缓冲 */
    uint16_t dma_buffer_size;                             /**< 环形缓冲长度（字节） */
    uint16_t dma_cursor;                                  /**< 上次 IDLE 时的 DMA 写位置 */

    bsp_comm_mgr_frame_info_t frame_info[BSP_COMM_MGR_RX_FRAME_COUNT]; /**< 帧元数据环 */
    uint8_t write_index;                                  /**< 元数据环写索引 */
    uint8_t read_index;                                   /**< 元数据环读索引 */
    uint8_t frame_count;                                  /**< 待取帧数（环满时覆盖旧帧） */

    uint8_t frame_buffer[BSP_COMM_MGR_RX_FRAME_BUFFER_SIZE]; /**< 整帧提取缓冲 */
    uint16_t frame_length;                                /**< 最近一次取出的帧长度 */
    bool is_frame_ready;                                  /**< 最近一次取帧是否成功 */
} bsp_comm_mgr_rx_handle_t;

/** @brief 通信对象前向声明。 */
typedef struct bsp_comm_mgr bsp_comm_mgr_t;

/** @brief 通信链路对象（状态、配置副本、发送组帧缓冲与接收环形队列）。 */
struct bsp_comm_mgr {
    bsp_comm_mgr_state_enum state;                       /**< 对象状态 */
    bsp_comm_mgr_config_t config;                        /**< 板级配置副本 */
    uint8_t tx_frame[BSP_COMM_MGR_JUSTFLOAT_FRAME_SIZE]; /**< 组帧缓冲（含帧尾） */
    uint16_t tx_frame_length;                            /**< 上一帧总长度（字节） */
    uint8_t tx_channel_count;                            /**< 上一帧通道数 */
    bsp_comm_mgr_rx_handle_t rx;                         /**< 接收环形队列 */
};

/**
 * @brief 初始化通信链路：构建底层 UART 板级实例并把对象置为可用。
 * @details 经 HAL UART 函数表 hal_uart_ops->pf_build() 完成引脚组构建、UART 配置、
 *          收发 DMA 装配与使能（接收 DMA 的目的缓冲即本层 rx.dma_buffer），并绑定
 *          接收通知回调；随后下发板级配置、复位接收队列并清空组帧缓冲。
 *          本接口**不使能** IDLE 中断，需要收帧时由应用层实现 USART3_IRQHandler 后
 *          调用 bsp_comm_mgr_enable_rx_irq()。
 * @param p_object 通信对象指针。
 * @retval bsp_comm_mgr_ok 操作成功。
 * @retval bsp_comm_mgr_err_param 参数无效。
 * @retval bsp_comm_mgr_err_state 底层 UART 构建或回调绑定失败，对象置为 error。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_init(bsp_comm_mgr_t *p_object);

/**
 * @brief 按 JustFloat 协议发送一帧浮点数据。
 * @details 组帧规则：N 个通道依次按小端 IEEE-754 单精度搬入，帧尾 4 字节
 *          0x00 0x00 0x80 0x7F 补在末尾；整帧一次经 hal_uart_ops->pf_send()
 *          阻塞发出（阻塞语义见 HAL UART 发送接口）。
 * @param p_object 通信对象指针。
 * @param p_data 待发送浮点数组首地址（通道数不少于 channel_count）。
 * @param channel_count 本帧通道数（1 … BSP_COMM_MGR_JUSTFLOAT_MAX_CHANNELS）。
 * @retval bsp_comm_mgr_ok 操作成功。
 * @retval bsp_comm_mgr_err_param 参数无效、通道数越界或帧长超过配置上限。
 * @retval bsp_comm_mgr_err_state 对象未初始化或底层发送失败。
 * @retval bsp_comm_mgr_err_timeout 底层发送超时。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_send_justfloat(
    bsp_comm_mgr_t *p_object,
    const float *p_data,
    uint8_t channel_count);

/**
 * @brief 接收环形队列写入（IDLE 中断上下文）。
 * @details 由 HAL 层 hal_uart_rx_idle_handler() 经回调进入：把"写位置未变化"判为
 *          无新数据，否则按环形回绕算出本帧 (start_offset, length) 写入帧元数据环，
 *          并更新游标；环满时覆盖旧帧且不再增计数。
 * @param p_object 通信对象指针。
 * @param write_position DMA 在 rx.dma_buffer 中的当前写位置（0 … dma_buffer_size）。
 * @note 运行于中断上下文，内部只做常数级操作，不做数据拷贝。
 */
void bsp_comm_mgr_rx_idle_handler(bsp_comm_mgr_t *p_object,
                                  uint16_t write_position);

/**
 * @brief 从帧元数据环取出最早一帧（含环形回绕的两段拷贝）。
 * @details 对应参考实现 USART3_GetFrame()：frame_count 为 0 时返回；
 *          start + length 未越界时一次拷贝，越界时分两段拷贝；
 *          取出的整帧与长度写入 rx.frame_buffer / rx.frame_length，
 *          read_index 前移、frame_count 减一。
 * @param p_object 通信对象指针。
 * @retval bsp_comm_mgr_ok 成功取出一帧。
 * @retval bsp_comm_mgr_err_param 参数无效。
 * @retval bsp_comm_mgr_err_state 当前无待取帧，或帧元数据异常（该帧被丢弃）。
 * @note 取出的帧数据位于 p_object->rx.frame_buffer，长度位于 p_object->rx.frame_length。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_get_frame(bsp_comm_mgr_t *p_object);

/**
 * @brief 查询帧元数据环中是否有待取完整帧。
 * @param p_object 通信对象指针。
 * @retval true 有帧待取。
 * @retval false 无帧或对象无效。
 */
bool bsp_comm_mgr_is_data_available(const bsp_comm_mgr_t *p_object);

/**
 * @brief 使能/禁止接收 IDLE 空闲中断源（中断线已在 bsp_comm_mgr_init() 内配好）。
 * @details 经 HAL 层 hal_uart_ops->pf_enable_idle_irq() 在运行时开关 IDLE 中断源；
 *          抢占/子优先级由 HAL 层板级常量给出（抢占 1 / 子 0），与 hal_adc.c 的
 *          注入组中断配置方式一致，本层不再逐个传入。
 * @param p_object 通信对象指针。
 * @param enable true 使能 IDLE 中断源，false 关闭。
 * @retval bsp_comm_mgr_ok 操作成功。
 * @retval bsp_comm_mgr_err_param 参数无效。
 * @retval bsp_comm_mgr_err_state 使能时对象未初始化，或底层平台层拒绝。
 * @note 关闭（enable = false）不检查对象状态，任何时候调用都安全。
 * @warning 使能前应用层必须已实现 void USART3_IRQHandler(void) 并在其中调用
 *          hal_uart_ops->pf_rx_idle_handler(&hal_uart_comm)；否则启动文件里 .weak
 *          的空实现（1: j 1b 死循环）会被执行，CPU 直接跑飞。
 */
bsp_comm_mgr_status_enum bsp_comm_mgr_enable_rx_irq(bsp_comm_mgr_t *p_object,
                                                    bool enable);

/**
 * @brief 查询通信对象当前状态。
 * @param p_object 通信对象指针。
 * @return 对象状态；p_object 为 NULL 时返回 bsp_comm_mgr_state_error。
 */
bsp_comm_mgr_state_enum bsp_comm_mgr_get_state(const bsp_comm_mgr_t *p_object);

/* ==========================================================================
 * 板级实例与操作函数表
 * ========================================================================== */

/** @brief 调试通信链路实例（USART3 + JustFloat）。 */
extern bsp_comm_mgr_t bsp_comm_mgr_debug;

/** @brief 通信链路操作函数表（函数表本体在 bsp_comm_mgr.c 中私有，此处只暴露指针）。 */
typedef struct {
    /** @brief 初始化通信链路（见 bsp_comm_mgr_init）。 */
    bsp_comm_mgr_status_enum (*pf_init)(bsp_comm_mgr_t *p_object);
    /** @brief 按 JustFloat 协议发送一帧浮点数据（见 bsp_comm_mgr_send_justfloat）。 */
    bsp_comm_mgr_status_enum (*pf_send_justfloat)(
        bsp_comm_mgr_t *p_object,
        const float *p_data,
        uint8_t channel_count);
    /** @brief 接收环形队列写入（见 bsp_comm_mgr_rx_idle_handler）。 */
    void (*pf_rx_idle_handler)(bsp_comm_mgr_t *p_object,
                               uint16_t write_position);
    /** @brief 取出最早一帧（见 bsp_comm_mgr_get_frame）。 */
    bsp_comm_mgr_status_enum (*pf_get_frame)(bsp_comm_mgr_t *p_object);
    /** @brief 查询是否有待取帧（见 bsp_comm_mgr_is_data_available）。 */
    bool (*pf_is_data_available)(const bsp_comm_mgr_t *p_object);
    /** @brief 使能/禁止接收 IDLE 中断源（见 bsp_comm_mgr_enable_rx_irq）。 */
    bsp_comm_mgr_status_enum (*pf_enable_rx_irq)(bsp_comm_mgr_t *p_object,
                                                 bool enable);
    /** @brief 查询对象状态（见 bsp_comm_mgr_get_state）。 */
    bsp_comm_mgr_state_enum (*pf_get_state)(const bsp_comm_mgr_t *p_object);
} bsp_comm_mgr_ops_t;

/** @brief 通信链路操作函数表指针（在 bsp_comm_mgr.c 中定义并绑定）。 */
extern const bsp_comm_mgr_ops_t *const bsp_comm_mgr_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_COMM_MGR_H__ */
