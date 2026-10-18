/**
 * @file hal_uart.h
 * @brief UART 硬件抽象接口（含 CH32V203 板级实例与 DMA 收发）。
 * @details 与 GPIO/SPI/ADC 同构：对象操作函数同层实现，硬件访问统一经平台 UART
 *          操作函数表指针 plat_ch32v203_uart_ops 完成，引脚组构建经 HAL GPIO
 *          函数表指针 hal_gpio_ops 完成，本层不含厂商库符号。
 *          板级实例按"用途"区分，目前只有与 VOFA 上位机通信的一路：
 *          - hal_uart_comm：USART3（PB10/PB11）4Mbps + DMA 发送（普通模式）
 *            + DMA 循环接收（IDLE 空闲中断界定帧边界），对应参考实现
 *            drv_uart.c 的 Drv_USART_Init() / USART_DMA_Send()。
 *          接收侧本层只负责"硬件收数据"这一段，环形队列与整帧提取不在本层：
 *          - 接收 DMA 的目的缓冲由上层经 hal_uart_bind_rx_buffer() 提供；
 *          - hal_uart_rx_idle_handler() 清 IDLE 标志后，由接收 DMA 剩余计数算出
 *            当前 DMA 写位置，经 hal_uart_bind_rx_callback() 注册的回调上报上层；
 *          - 上层（BSP 层）据此维护帧元数据环并提取整帧。
 * @note 参考实现的帧格式没有帧头/长度/校验字段：帧边界完全由 USART 空闲（IDLE）
 *       中断界定，帧内容即 VOFA 命令 ASCII 字符串（见 vofa_Tool 下的 json 配置，
 *       例如 "DEVICE_IDLE"、"CURRENT_CTRL_MODE"、"Target_Iq = %f"）。
 * @note 本层不引入组配置/多视图机制：一个 hal_uart_t 直接对应一个 UART 实例。
 */
#ifndef __HAL_UART_H__
#define __HAL_UART_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_UART_* 帧格式/标志/中断源/传输请求取值宏 */
#include "platform_ch32v203_uart.h"
/* MCU 抽象接口：PLAT_DMA_* 通道配置取值宏（通用 DMA 通道的装配/状态在 plat_dma） */
#include "platform_ch32v203_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UART 对象前向声明。 */
typedef struct hal_uart hal_uart_t;

/** @brief UART对象的生命周期状态。 */
typedef enum {
    hal_uart_state_reset = 0,
    hal_uart_state_ready,
    hal_uart_state_enabled,
    hal_uart_state_disabled,
    hal_uart_state_error
} hal_uart_state_enum;

/** @brief UART接口调用返回的状态码。 */
typedef enum {
    hal_uart_ok = 0,
    hal_uart_err_param = -1,
    hal_uart_err_state = -2,
    hal_uart_err_unsupported = -3,
    hal_uart_err_timeout = -4
} hal_uart_status_enum;

/** @brief DMA 发送/等待标志的轮询次数上限（参考实现为死等，本层加超时保护）。 */
#define HAL_UART_DMA_TIMEOUT_COUNT         (1000000U)

/** @brief 发送 DMA 通道优先级（平台层 DMA 优先级宏，对应参考实现的 Medium）。 */
#define HAL_UART_DMA_TX_PRIORITY           PLAT_DMA_PRIORITY_MEDIUM

/** @brief 接收 DMA 通道优先级（平台层 DMA 优先级宏，对应参考实现的 VeryHigh）。 */
#define HAL_UART_DMA_RX_PRIORITY           PLAT_DMA_PRIORITY_VERY_HIGH

/**
 * @brief 接收数据通知回调：接收 DMA 写位置更新时由 IDLE 中断调用。
 * @param p_context 绑定时给出的上下文指针（原样回传）。
 * @param write_position DMA 在接收缓冲中的当前写位置（0 … buffer_size）。
 * @note 运行于中断上下文，回调内须保持精简。
 */
typedef void (*hal_uart_rx_callback_t)(void *p_context, uint16_t write_position);

/** @brief UART 波特率、帧格式与流控参数。 */
typedef struct {
    uint32_t baud_rate;             /**< 波特率（bps） */
    uint16_t word_length;           /**< PLAT_UART_WORD_LENGTH_x */
    uint16_t stop_bits;             /**< PLAT_UART_STOP_BITS_x */
    uint16_t parity;                /**< PLAT_UART_PARITY_x */
    uint16_t mode;                  /**< PLAT_UART_MODE_TX | PLAT_UART_MODE_RX */
    uint16_t flow_control;          /**< PLAT_UART_FLOW_CONTROL_x */
    uint32_t timeout_count;         /**< DMA 等待轮询次数上限 */
    void *p_user_data;              /**< 保留：用户上下文 */
} hal_uart_config_t;

/** @brief UART 对象定义（状态、实例基地址、配置与接收绑定）。 */
struct hal_uart {
    hal_uart_state_enum state;
    uintptr_t base_address;
    hal_uart_config_t config;
    uint8_t *p_rx_buffer;                   /**< 接收 DMA 目的缓冲（上层提供） */
    uint16_t rx_buffer_size;                /**< 接收 DMA 缓冲字节数 */
    hal_uart_rx_callback_t pf_rx_callback;  /**< 写位置更新回调 */
    void *p_rx_context;                     /**< 回调上下文 */
};

/**
 * @brief 绑定UART对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（PLAT_PERIPH_USARTx）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效或必需指针为空。
 * @note 本接口会清空接收缓冲/回调绑定，接收能力须由 hal_uart_bind_rx_buffer() 与
 *       hal_uart_bind_rx_callback() 重新绑定。
 */
hal_uart_status_enum hal_uart_bind(hal_uart_t *p_object,
                                   uintptr_t base_address,
                                   const hal_uart_config_t *p_config);

/**
 * @brief 绑定接收 DMA 缓冲（须在 hal_uart_init() 之前调用）。
 * @details hal_uart_init() 用该缓冲装配循环接收 DMA，故缓冲必须先绑定；
 *          hal_uart_bind() 会清空本绑定，不需要接收时可不调用本接口。
 * @param p_object 模块对象指针。
 * @param p_rx_buffer 接收缓冲首地址（由上层提供，DMA 直接写入）。
 * @param buffer_size 缓冲字节数（须与上层维护的环形队列长度一致）。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效或对象未绑定。
 */
hal_uart_status_enum hal_uart_bind_rx_buffer(hal_uart_t *p_object,
                                             uint8_t *p_rx_buffer,
                                             uint16_t buffer_size);

/**
 * @brief 绑定接收通知回调（接收 DMA 写位置更新时通知上层）。
 * @details 回调在 IDLE 中断上下文中执行；未绑定时 hal_uart_rx_idle_handler()
 *          只清 IDLE 标志，不上报写位置。
 * @param p_object 模块对象指针。
 * @param pf_callback 回调函数指针。
 * @param p_context 回调上下文（原样回传给回调）。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效或对象未绑定。
 */
hal_uart_status_enum hal_uart_bind_rx_callback(
    hal_uart_t *p_object,
    hal_uart_rx_callback_t pf_callback,
    void *p_context);

/**
 * @brief 初始化UART：时钟 + 复位 + 帧格式 + 收发DMA装配 + 传输请求 + 使能。
 * @details 顺序对应参考实现 drv_USART_DMA_init()：
 *          时钟/DMA 时钟 → USART_DeInit → USART_Init → 发送与接收 DMA 配置
 *          → 接收抽象通道（platform_ch32v203_dma_*，循环接收，缓冲取
 *          hal_uart_bind_rx_buffer() 绑定的内存）常开
 *          → USART_DMACmd(Tx/Rx) → USART_Cmd(ENABLE)。
 *          发送通道（普通模式）在接收通道之后一次装配完毕：除存储器地址与传输
 *          长度置 0 外（这两个是逐次发送的动态值，由 hal_uart_send() 写入），
 *          方向/宽度/模式/优先级等参数都在本接口内配好；**本接口不使能发送
 *          通道**，使能与关闭都在 hal_uart_send() 中完成。
 *          IDLE 中断线在本接口内按板级常量配置并使能（中断源不打开），中断源另由
 *          hal_uart_enable_idle_irq() 打开，避免应用层未实现 USART3_IRQHandler 时
 *          进入弱定义死循环。
 * @param p_object 模块对象指针。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 对象未绑定、实例不受支持或配置非法。
 */
hal_uart_status_enum hal_uart_init(hal_uart_t *p_object);

/** @brief 关闭并复位UART外设（含收发 DMA 通道）。 */
hal_uart_status_enum hal_uart_deinit(hal_uart_t *p_object);

/** @brief 使能UART收发。 */
hal_uart_status_enum hal_uart_enable(hal_uart_t *p_object);

/** @brief 禁用UART收发。 */
hal_uart_status_enum hal_uart_disable(hal_uart_t *p_object);

/**
 * @brief DMA 阻塞式发送一段数据（调用时给出缓冲区地址与数据长度即可）。
 * @details 对应参考实现 USART_DMA_Send()：写发送通道的存储器地址与数据长度
 *          （对应参考实现直写 DMA1_Channel2->MADDR / CNTR）→ 清传输完成标志
 *          → 使能通道 → 等剩余计数归零 → 等传输完成标志 → 清标志并关通道
 *          → 等 USART 发送完成（TC）标志。
 *          通道的方向/宽度/模式/优先级等参数已在 hal_uart_init() 中装配好，
 *          本接口只写上述两个逐次变化的寄存器。
 * @param p_object 模块对象指针。
 * @param p_tx_buf 待发送数据缓冲区（本次传输的存储器地址）。
 * @param data_length 待发送字节数（本次传输的数据长度）。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_state 对象未使能或平台层写入失败。
 * @retval hal_uart_err_timeout 等待 DMA/TC 完成超时。
 * @note 参考实现为死等；本实现按 config.timeout_count 加超时保护。
 */
hal_uart_status_enum hal_uart_send(hal_uart_t *p_object,
                                   const uint8_t *p_tx_buf,
                                   uint16_t data_length);

/**
 * @brief 轮询直读一段数据（逐字节等 RXNE 后读数据寄存器）。
 * @note 已绑定接收 DMA 缓冲时，CPU 与 DMA 会争用数据寄存器，此时应改用上层
 *       （BSP 层）的整帧提取接口；本接口用于未启用 DMA 接收的场景。
 * @param p_object 模块对象指针。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_timeout 等待 RXNE 超时。
 */
hal_uart_status_enum hal_uart_read(hal_uart_t *p_object, uint8_t *p_rx_buf,
                                   uint16_t data_length);

/**
 * @brief 查询发送通道是否仍在传输（忙判断）。
 * @param p_object 模块对象指针。
 * @retval true DMA 剩余计数非 0，发送尚未结束。
 * @retval false 发送空闲或对象无效。
 */
bool hal_uart_is_tx_busy(const hal_uart_t *p_object);

/**
 * @brief IDLE 空闲中断服务本体（空闲中断接收部分）。
 * @details 对应参考实现 usart3_dma_handle() 的前半段：先清 IDLE 标志（读状态
 *          寄存器再读数据寄存器）→ 由接收 DMA 剩余计数算出当前写位置
 *          （循环模式下 已收字节 = 缓冲长度 - 剩余）→ 经绑定的回调把写位置上报
 *          上层。环形队列写入与整帧提取不在本层（见 BSP 层通信链路模块）。
 * @param p_object 模块对象指针。
 * @note 应用层须自行实现 void USART3_IRQHandler(void) 并在其中调用本接口
 *       （例如：if (plat_ch32v203_uart_ops->pf_get_it_status(...)) ... ，
 *       最简形式为直接调用 hal_uart_rx_idle_handler(&hal_uart_comm)）。
 * @note 未绑定接收缓冲或未绑定回调时，本接口只清标志、不上报。
 */
void hal_uart_rx_idle_handler(hal_uart_t *p_object);

/**
 * @brief 使能/禁止 IDLE 空闲中断源（中断线已在 hal_uart_init() 中配好）。
 * @details 与 hal_adc.c 的 hal_adc_enable_injected_irq() 同一配置方式：中断抢占/
 *          子优先级由 hal_uart.c 内的板级常量
 *          HAL_UART_IDLE_IRQ_PREEMPTION_PRIORITY / HAL_UART_IDLE_IRQ_SUB_PRIORITY
 *          给出（抢占 1 / 子 0），并在 hal_uart_init() 中随中断线一并配置使能；
 *          本接口只执行 USART_ITConfig(IDLE, enable) 在运行时开关中断源。
 * @param p_object 模块对象指针。
 * @param enable true 使能 IDLE 中断源，false 关闭。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_state 平台层拒绝（实例不受支持）。
 * @warning 使能前应用层必须已实现 void USART3_IRQHandler(void) 并在其中调用
 *          hal_uart_rx_idle_handler()；否则本工程启动文件里 .weak 的空实现
 *          （1: j 1b 死循环）会被执行，CPU 直接跑飞。
 */
hal_uart_status_enum hal_uart_enable_idle_irq(hal_uart_t *p_object, bool enable);

/** @brief 查询UART对象当前状态。 */
hal_uart_state_enum hal_uart_get_state(const hal_uart_t *p_object);

/* --------------------------------------------------------------------------
 * 板级实例：VOFA 上位机串口（USART3 + DMA 收发）
 * -------------------------------------------------------------------------- */

/** @brief VOFA 串口实例（USART3，PB10/PB11，4Mbps，DMA 发送 + DMA 循环接收）。 */
extern hal_uart_t hal_uart_comm;

/**
 * @brief 构建板级 UART 实例（引脚组 + 配置 + DMA + 使能）。
 * @details 顺序：经 hal_gpio_ops->pf_build() 构建 hal_gpio_usart 引脚组
 *          （PB10 AF_PP / PB11 浮空输入）→ 绑定实例与配置 → 绑定接收 DMA 缓冲
 *          → hal_uart_init()（时钟、帧格式、收发 DMA、传输请求、使能）。
 *          默认**不使能** IDLE 中断，需要收帧时由调用方在实现 USART3_IRQHandler
 *          之后调用 hal_uart_enable_idle_irq()。
 * @param p_object UART 对象指针。
 * @param p_rx_buffer 接收 DMA 缓冲首地址，可为 NULL（只发送）。
 * @param rx_buffer_size 接收 DMA 缓冲字节数（p_rx_buffer 为 NULL 时忽略）。
 * @retval hal_uart_ok 操作成功。
 * @retval hal_uart_err_param 参数无效。
 * @retval hal_uart_err_state 引脚组构建或平台层配置失败。
 */
hal_uart_status_enum hal_uart_build(hal_uart_t *p_object,
                                    uint8_t *p_rx_buffer,
                                    uint16_t rx_buffer_size);

/**
 * @brief HAL 层 UART 函数表：板级构建 / 收发 / 接收通知 / 状态。
 * @details 表中的函数指针在 hal_uart.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_uart_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级 UART 实例（见 hal_uart_build）。 */
    hal_uart_status_enum (*pf_build)(hal_uart_t *p_object,
                                     uint8_t *p_rx_buffer,
                                     uint16_t rx_buffer_size);
    /** @brief 绑定接收 DMA 缓冲（见 hal_uart_bind_rx_buffer）。 */
    hal_uart_status_enum (*pf_bind_rx_buffer)(hal_uart_t *p_object,
                                              uint8_t *p_rx_buffer,
                                              uint16_t buffer_size);
    /** @brief 绑定接收通知回调（见 hal_uart_bind_rx_callback）。 */
    hal_uart_status_enum (*pf_bind_rx_callback)(
        hal_uart_t *p_object,
        hal_uart_rx_callback_t pf_callback,
        void *p_context);
    /** @brief 初始化 UART 外设（见 hal_uart_init）。 */
    hal_uart_status_enum (*pf_init)(hal_uart_t *p_object);
    /** @brief 关闭并复位 UART 外设（见 hal_uart_deinit）。 */
    hal_uart_status_enum (*pf_deinit)(hal_uart_t *p_object);
    /** @brief 使能 UART（见 hal_uart_enable）。 */
    hal_uart_status_enum (*pf_enable)(hal_uart_t *p_object);
    /** @brief 禁用 UART（见 hal_uart_disable）。 */
    hal_uart_status_enum (*pf_disable)(hal_uart_t *p_object);
    /** @brief DMA 阻塞式发送（见 hal_uart_send）。 */
    hal_uart_status_enum (*pf_send)(hal_uart_t *p_object,
                                    const uint8_t *p_tx_buf,
                                    uint16_t data_length);
    /** @brief 轮询直读（见 hal_uart_read）。 */
    hal_uart_status_enum (*pf_read)(hal_uart_t *p_object, uint8_t *p_rx_buf,
                                    uint16_t data_length);
    /** @brief 查询发送是否忙（见 hal_uart_is_tx_busy）。 */
    bool (*pf_is_tx_busy)(const hal_uart_t *p_object);
    /** @brief IDLE 空闲中断服务本体（见 hal_uart_rx_idle_handler）。 */
    void (*pf_rx_idle_handler)(hal_uart_t *p_object);
    /** @brief 使能/禁止 IDLE 空闲中断源（见 hal_uart_enable_idle_irq）。 */
    hal_uart_status_enum (*pf_enable_idle_irq)(hal_uart_t *p_object, bool enable);
    /** @brief 查询对象状态（见 hal_uart_get_state）。 */
    hal_uart_state_enum (*pf_get_state)(const hal_uart_t *p_object);
} hal_uart_ops_t;

/** @brief HAL 层 UART 函数表指针（在 hal_uart.c 中定义并绑定）。 */
extern const hal_uart_ops_t *const hal_uart_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_UART_H__ */
