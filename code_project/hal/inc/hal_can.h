/**
 * @file hal_can.h
 * @brief CAN 硬件抽象接口（含 CH32V203 板级实例与 HAL 层函数表）。
 * @details 与 GPIO/ADC/SPI 同构：对象操作函数同层实现，硬件访问统一经平台 CAN 操作
 *          函数表指针 plat_ch32v203_can_ops 完成，本层不含厂商库符号。
 *          板级实例只有一个"主机"用途：
 *          - hal_can_host：CAN1 + PA12（CAN 发送）/PA11（CAN 接收）默认引脚（无需重映射），
 *            1 Mbps、正常模式、过滤器组 0 掩码模式（标准帧 ID 0x100~0x1FF → FIFO0），
 *            对应参考实现 drv_can.c 的顶层初始化函数与标准帧发送函数。
 *          - 中断接收路径：本层提供 hal_can_rx_handler()（ISR 本体）与
 *            hal_can_enable_rx_irq()。CAN1 RX0 中断向量在本工程 startup 中是 .weak
 *            的 `1: j 1b` 死循环，故默认 build 不打开中断；应用层必须自行实现
 *            CAN1 RX0 中断服务函数并在其中调用 hal_can_rx_handler()，再调用
 *            hal_can_enable_rx_irq()，否则请使用轮询 hal_can_receive()。
 * @note 本层不引入组配置/多视图机制：一个 hal_can_t 直接对应一个 CAN 实例。
 */
#ifndef __HAL_CAN_H__
#define __HAL_CAN_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_CAN_* 模式/位时序/过滤器/帧类型/标志/中断取值宏 */
#include "platform_ch32v203_can.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CAN 对象前向声明。 */
typedef struct hal_can hal_can_t;

/** @brief CAN对象的生命周期状态。 */
typedef enum {
    hal_can_state_reset = 0,
    hal_can_state_ready,
    hal_can_state_enabled,
    hal_can_state_disabled,
    hal_can_state_error
} hal_can_state_enum;

/** @brief CAN接口调用返回的状态码。 */
typedef enum {
    hal_can_ok = 0,
    hal_can_err_param = -1,
    hal_can_err_state = -2,
    hal_can_err_unsupported = -3,
    hal_can_err_timeout = -4,
    hal_can_err_no_mailbox = -5  /**< 发送邮箱全忙，报文未挂入邮箱 */
} hal_can_status_enum;

/** @brief CAN 报文最大数据长度（字节）。 */
#define HAL_CAN_DATA_LENGTH_MAX        PLAT_CAN_DATA_LENGTH_MAX

/** @brief 标准帧标识符上限（11 位）。 */
#define HAL_CAN_STD_ID_MAX             PLAT_CAN_STD_ID_MAX

/** @brief 板级自测标准帧 ID（来源 drv_can.h 的发送 ID 宏，取值为 0x4）。 */
#define HAL_CAN_TEST_STD_ID            (0x4UL)

/** @brief 板级自测数据长度（来源 drv_can.h 的发送长度宏，取值为 4）。 */
#define HAL_CAN_TEST_DATA_LENGTH       (4U)

/**
 * @brief CAN1 RX0 中断线抢占优先级（来源 drv_can.c 中断配置：抢占优先级 = 1）。
 * @note 供 hal_can_enable_rx_irq() 调用方使用；实际优先级以调用方实参为准。
 */
#define HAL_CAN_RX_IRQ_PREEMPTION_PRIORITY (1U)
/**
 * @brief CAN1 RX0 中断线子优先级（来源 drv_can.c 中断配置：子优先级 = 1）。
 */
#define HAL_CAN_RX_IRQ_SUB_PRIORITY        (1U)

/** @brief CAN 位时序、工作模式与总线开关配置（对应标准库初始化结构的可移植子集）。 */
typedef struct {
    uint32_t mode;                       /**< PLAT_CAN_MODE_x */
    uint8_t synchronization_jump_width;  /**< PLAT_CAN_SJW_x */
    uint8_t bit_time_segment1;           /**< PLAT_CAN_BS1_x */
    uint8_t bit_time_segment2;           /**< PLAT_CAN_BS2_x */
    uint16_t prescaler;                  /**< 波特率预分频，1..1024 */
    bool is_time_trigger_enabled;        /**< 时间触发通信模式 */
    bool is_auto_bus_off_enabled;        /**< 自动离线管理 */
    bool is_auto_wakeup_enabled;         /**< 自动唤醒 */
    /**
     * @brief 自动重传：true 允许自动重传，false 禁止（对应标准库 NART 位取反）。
     * @note 参考实现 drv_can.c 中 NART 位使能（即禁止自动重传），
     *       因此板级实例本字段取 false。
     */
    bool is_auto_retransmission_enabled;
    bool is_receive_fifo_locked_enabled; /**< 接收 FIFO 锁定模式 */
    bool is_transmit_fifo_priority_enabled; /**< 发送 FIFO 优先级（按标识符） */
    uint32_t timeout_count;              /**< 保留：等待/超时计数 */
    void *p_user_data;                   /**< 保留：调用方私有数据 */
} hal_can_config_t;

/** @brief CAN 过滤器组配置（对应标准库过滤器初始化结构的可移植子集）。 */
typedef struct {
    uint16_t filter_number;    /**< 过滤器组编号，0..PLAT_CAN_FILTER_NUMBER_MAX */
    uint8_t mode;              /**< PLAT_CAN_FILTER_MODE_x */
    uint8_t scale;             /**< PLAT_CAN_FILTER_SCALE_x */
    uint16_t id_high;          /**< 标识符高 16 位 */
    uint16_t id_low;           /**< 标识符低 16 位 */
    uint16_t mask_id_high;     /**< 掩码/标识符列表高 16 位 */
    uint16_t mask_id_low;      /**< 掩码/标识符列表低 16 位 */
    uint16_t fifo_assignment;  /**< PLAT_CAN_FILTER_FIFO_x，同时用作接收 FIFO 号 */
    bool is_activated;         /**< true 激活该过滤器组 */
} hal_can_filter_config_t;

/** @brief CAN 报文（标准帧/扩展帧、数据帧/远程帧）。 */
typedef struct {
    uint32_t identifier;       /**< 标准帧 0..HAL_CAN_STD_ID_MAX，扩展帧 0..0x1FFFFFFF */
    uint8_t data_length;       /**< 数据长度，0..HAL_CAN_DATA_LENGTH_MAX */
    uint8_t data[HAL_CAN_DATA_LENGTH_MAX]; /**< 数据字节（不足部分补 0） */
    bool is_extended;          /**< true 扩展帧，false 标准帧 */
    bool is_remote;            /**< true 远程帧，false 数据帧 */
    uint8_t filter_index;      /**< 接收时有效：命中过滤器组编号 */
} hal_can_frame_t;

/** @brief CAN 对象定义（状态、实例基地址、配置、过滤器与中断接收缓存）。 */
struct hal_can {
    hal_can_state_enum state;
    uintptr_t base_address;
    hal_can_config_t config;
    hal_can_filter_config_t filter;
    hal_can_frame_t rx_frame;   /**< 中断路径接收缓存（由 hal_can_rx_handler 填写） */
    bool rx_finish_flag;        /**< true 表示缓存中有一帧待取 */
};

/**
 * @brief 绑定CAN对象：保存实例基地址、总线配置与过滤器配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（PLAT_PERIPH_CAN1）。
 * @param p_config 位时序与工作模式配置。
 * @param p_filter 过滤器组配置。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效或必需指针为空。
 */
hal_can_status_enum hal_can_bind(hal_can_t *p_object,
                                 uintptr_t base_address,
                                 const hal_can_config_t *p_config,
                                 const hal_can_filter_config_t *p_filter);

/**
 * @brief 初始化CAN：时钟 + 复位 + 位时序/模式配置 + 过滤器配置。
 * @details 标准库初始化在配置完成后已退出初始化模式（外设处于正常模式），
 *          故初始化成功即可轮询收发，无需额外的使能动作。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 * @retval hal_can_err_timeout 位时序配置未能退出初始化模式。
 * @retval hal_can_err_state 时钟/复位或过滤器配置失败。
 */
hal_can_status_enum hal_can_init(hal_can_t *p_object);

/** @brief 停止总线（请求睡眠模式）并复位CAN外设。 */
hal_can_status_enum hal_can_deinit(hal_can_t *p_object);

/** @brief 启动CAN总线（请求正常模式）。 */
hal_can_status_enum hal_can_enable(hal_can_t *p_object);

/** @brief 停止CAN总线（请求睡眠模式）。 */
hal_can_status_enum hal_can_disable(hal_can_t *p_object);

/**
 * @brief 重新配置过滤器组（并更新对象内的过滤器记录）。
 * @param p_object 模块对象指针。
 * @param p_filter 过滤器配置。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效。
 * @retval hal_can_err_state 平台层拒绝（实例或过滤器组编号非法）。
 */
hal_can_status_enum hal_can_config_filter(hal_can_t *p_object,
                                          const hal_can_filter_config_t *p_filter);

/**
 * @brief 使能 FIFO 消息挂起中断并配置 CAN1 RX0 中断线。
 * @warning 本工程 startup 中 CAN1 RX0 中断向量是 .weak 的死循环默认处理函数，
 *          调用本函数前应用层必须已实现该中断服务函数并在其中调用
 *          hal_can_rx_handler()，否则收到报文会死循环。
 * @param p_object 模块对象指针。
 * @param preemption_priority 抢占优先级（板级取值见
 *        HAL_CAN_RX_IRQ_PREEMPTION_PRIORITY，来源 drv_can.c）。
 * @param sub_priority 子优先级（板级取值见 HAL_CAN_RX_IRQ_SUB_PRIORITY）。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 * @retval hal_can_err_state 平台层中断配置失败。
 */
hal_can_status_enum hal_can_enable_rx_irq(hal_can_t *p_object,
                                          uint8_t preemption_priority,
                                          uint8_t sub_priority);

/**
 * @brief 关闭 FIFO 消息挂起中断。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 对象未绑定或实例不受支持。
 */
hal_can_status_enum hal_can_disable_rx_irq(hal_can_t *p_object);

/**
 * @brief CAN1 RX0 中断服务函数本体（应用层 ISR 中调用）。
 * @details 流程与参考实现 drv_can.c 的 RX0 中断服务函数一致：
 *          消息挂起中断有效且上一帧已被取走时，从过滤器关联 FIFO 读走一帧到
 *          对象接收缓存并置位 rx_finish_flag，最后清除中断挂起位。
 * @param p_object 模块对象指针。
 */
void hal_can_rx_handler(hal_can_t *p_object);

/**
 * @brief 发送一个CAN报文（请求挂入发送邮箱，成功即返回）。
 * @param p_object 模块对象指针。
 * @param p_frame 待发送报文。
 * @retval hal_can_ok 报文已挂入发送邮箱。
 * @retval hal_can_err_param 参数无效或标识符超出帧类型范围。
 * @retval hal_can_err_no_mailbox 三个发送邮箱全忙，报文未挂入。
 */
hal_can_status_enum hal_can_send(hal_can_t *p_object,
                                 const hal_can_frame_t *p_frame);

/**
 * @brief 发送一个标准数据帧（对应参考实现的标准帧发送函数）。
 * @param p_object 模块对象指针。
 * @param std_id 标准帧 ID，0..HAL_CAN_STD_ID_MAX。
 * @param p_data 数据缓冲区。
 * @param length 数据长度；超过 HAL_CAN_DATA_LENGTH_MAX 时按参考实现截断为 8。
 * @retval hal_can_ok 报文已挂入发送邮箱。
 * @retval hal_can_err_param 参数无效或 ID 越界。
 * @retval hal_can_err_no_mailbox 三个发送邮箱全忙，报文未挂入。
 */
hal_can_status_enum hal_can_send_std_data(hal_can_t *p_object,
                                          uint32_t std_id,
                                          const uint8_t *p_data,
                                          uint8_t length);

/**
 * @brief 发送板级自测帧（对应参考实现的自测发送函数）。
 * @details 标准帧 ID = HAL_CAN_TEST_STD_ID（0x4），数据 = hal_can_test_data
 *          （{0x01,0x02,0x03,0x04}），长度 = HAL_CAN_TEST_DATA_LENGTH（4）。
 * @param p_object 模块对象指针。
 * @retval hal_can_ok 报文已挂入发送邮箱。
 * @retval hal_can_err_param 对象未绑定。
 * @retval hal_can_err_no_mailbox 三个发送邮箱全忙，报文未挂入。
 */
hal_can_status_enum hal_can_send_test_frame(hal_can_t *p_object);

/**
 * @brief 查询指定发送邮箱的状态（对应标准库邮箱状态查询）。
 * @param p_object 模块对象指针。
 * @param mailbox 邮箱号 0..PLAT_CAN_MAILBOX_COUNT-1。
 * @param p_status 输出：PLAT_CAN_TX_STATUS_x。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效或邮箱号越界。
 */
hal_can_status_enum hal_can_get_transmit_status(const hal_can_t *p_object,
                                                uint8_t mailbox,
                                                uint8_t *p_status);

/**
 * @brief 接收一个CAN报文。
 * @details 优先取中断缓存（rx_finish_flag）；否则轮询过滤器关联 FIFO，
 *          无待处理报文时返回 hal_can_err_state。
 * @param p_object 模块对象指针。
 * @param p_frame 输出报文。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效。
 * @retval hal_can_err_state 当前无待处理报文。
 */
hal_can_status_enum hal_can_receive(hal_can_t *p_object,
                                    hal_can_frame_t *p_frame);

/**
 * @brief 查询待处理报文数（含中断缓存中的 1 帧）。
 * @param p_object 模块对象指针。
 * @return 过滤器关联 FIFO 待处理数 + 中断缓存帧数；对象为空或未绑定时返回 0。
 */
uint8_t hal_can_is_message_pending(const hal_can_t *p_object);

/** @brief 查询CAN对象当前状态。 */
hal_can_state_enum hal_can_get_state(const hal_can_t *p_object);

/* --------------------------------------------------------------------------
 * 板级实例：CAN1 主机
 * -------------------------------------------------------------------------- */

/** @brief CAN1 板级实例（PA12/PA11，1 Mbps，过滤器组 0 接受标准帧 0x100~0x1FF）。 */
extern hal_can_t hal_can_host;

/** @brief 板级自测数据（来源 drv_can.c 的发送缓存数组）。 */
extern const uint8_t hal_can_test_data[HAL_CAN_TEST_DATA_LENGTH];

/**
 * @brief 构建板级 CAN 实例（引脚组 + 时钟 + 位时序/模式 + 过滤器）。
 * @details 顺序与参考实现 drv_can.c 一致：先经 HAL GPIO 函数表构建 CAN 引脚组
 *          （PA12/PA11，默认映射无需重映射），再绑定 CAN1 板级配置并初始化
 *          （时钟 → 复位 → 位时序/模式 → 过滤器）。默认**不**打开 RX0 中断，
 *          build 成功后即可轮询收发；需要中断接收时由应用层实现 CAN1 RX0 中断
 *          服务函数（调用 hal_can_rx_handler()）后再调 hal_can_enable_rx_irq()。
 * @param p_object CAN 对象指针。
 * @retval hal_can_ok 操作成功。
 * @retval hal_can_err_param 参数无效。
 * @retval hal_can_err_state 引脚组构建或平台层时钟/复位/过滤器配置失败。
 * @retval hal_can_err_timeout 位时序配置未能退出初始化模式。
 */
hal_can_status_enum hal_can_build(hal_can_t *p_object);

/**
 * @brief HAL 层 CAN 函数表：板级构建 / 初始化 / 收发 / 状态。
 * @details 表中的函数指针在 hal_can.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_can_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级 CAN 实例（见 hal_can_build）。 */
    hal_can_status_enum (*pf_build)(hal_can_t *p_object);
    /** @brief 初始化 CAN（见 hal_can_init）。 */
    hal_can_status_enum (*pf_init)(hal_can_t *p_object);
    /** @brief 停止总线并复位 CAN 外设（见 hal_can_deinit）。 */
    hal_can_status_enum (*pf_deinit)(hal_can_t *p_object);
    /** @brief 启动 CAN 总线（见 hal_can_enable）。 */
    hal_can_status_enum (*pf_enable)(hal_can_t *p_object);
    /** @brief 停止 CAN 总线（见 hal_can_disable）。 */
    hal_can_status_enum (*pf_disable)(hal_can_t *p_object);
    /** @brief 重新配置过滤器组（见 hal_can_config_filter）。 */
    hal_can_status_enum (*pf_config_filter)(
        hal_can_t *p_object, const hal_can_filter_config_t *p_filter);
    /** @brief 使能 RX0 中断（见 hal_can_enable_rx_irq）。 */
    hal_can_status_enum (*pf_enable_rx_irq)(hal_can_t *p_object,
                                            uint8_t preemption_priority,
                                            uint8_t sub_priority);
    /** @brief 关闭 RX0 中断（见 hal_can_disable_rx_irq）。 */
    hal_can_status_enum (*pf_disable_rx_irq)(hal_can_t *p_object);
    /** @brief RX0 中断服务函数本体（见 hal_can_rx_handler）。 */
    void (*pf_rx_handler)(hal_can_t *p_object);
    /** @brief 发送一个报文（见 hal_can_send）。 */
    hal_can_status_enum (*pf_send)(hal_can_t *p_object,
                                   const hal_can_frame_t *p_frame);
    /** @brief 发送标准数据帧（见 hal_can_send_std_data）。 */
    hal_can_status_enum (*pf_send_std_data)(hal_can_t *p_object, uint32_t std_id,
                                            const uint8_t *p_data,
                                            uint8_t length);
    /** @brief 发送板级自测帧（见 hal_can_send_test_frame）。 */
    hal_can_status_enum (*pf_send_test_frame)(hal_can_t *p_object);
    /** @brief 接收一个报文（见 hal_can_receive）。 */
    hal_can_status_enum (*pf_receive)(hal_can_t *p_object,
                                      hal_can_frame_t *p_frame);
    /** @brief 查询待处理报文数（见 hal_can_is_message_pending）。 */
    uint8_t (*pf_is_message_pending)(const hal_can_t *p_object);
    /** @brief 查询对象状态（见 hal_can_get_state）。 */
    hal_can_state_enum (*pf_get_state)(const hal_can_t *p_object);
} hal_can_ops_t;

/** @brief HAL 层 CAN 函数表指针（在 hal_can.c 中定义并绑定）。 */
extern const hal_can_ops_t *const hal_can_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_CAN_H__ */
