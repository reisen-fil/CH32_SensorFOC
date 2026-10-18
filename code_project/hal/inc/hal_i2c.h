/**
 * @file hal_i2c.h
 * @brief I2C 硬件抽象接口（含 CH32V203 板级实例与 HAL 层函数表）。
 * @details 与 ADC/SPI 同构：对象操作函数同层实现，硬件访问统一经平台 I2C 操作函数表
 *          指针 plat_ch32v203_i2c_ops 完成，引脚组构建与 I2C1 重映射经 HAL/平台 GPIO
 *          函数表完成，本层不含任何厂商库符号。
 *          I2C 的**事务时序**（忙等待、START/重复 START、地址与数据阶段的事件轮询、
 *          超时计数、末字节 NACK+STOP、失败时释放总线）在本层以私有函数封装，平台层
 *          只提供"单次硬件动作"原语（与 SPI 把 transfer 细节放 HAL 的做法一致）。
 *          本层不引入组配置/多视图机制：一个 hal_i2c_t 直接对应一个 I2C 实例。
 *
 *          板级实例 hal_i2c_encoder：I2C1 @400kHz 快速模式，PB8(SCL)/PB9(SDA)
 *          （复用开漏 + I2C1 重映射），面向 AS5600 类磁编码器角度读取，
 *          对应参考实现 drv_i2c.c 的硬件 I2C 轮询路径（Hardware_I2C_Mode==1）。
 *
 *          关键配置来源（参考实现 drv_i2c.c / drv_i2c.h）：
 *          - 速率 400000（400kHz 快速模式）、占空比 Tlow/Thigh = 2、自身地址 0x00、
 *            应答使能、7 位应答地址、初始化后即使能外设；
 *          - 每阶段轮询超时 0x1000，BUSY 卡总线检测 0x10；
 *          - 目标器件地址 0x36（AS5600）、角度寄存器 0x0C、读 2 字节（高字节在前）；
 *          - 可选 DMA 路径（参考实现 Hardware_I2C_Mode==2：发送用 CH6、接收用 CH7）
 *            本工程板级链路未使用；通用 DMA 通道装配已统一归入平台层
 *            platform_ch32v203_dma.c（通道号由 plat_ch32v203_dma_ops->pf_channel_get()
 *            按 platform_ch32v203_dma_peripheral_i2c1 给出），I2C 平台层只保留
 *            I2C 侧的 DMA 请求控制与数据寄存器地址查询。
 *
 * @note 软件模拟 I2C 备选：参考实现 drv_i2c.h 注明"400kHz 硬件 I2C 读一次角度整帧
 *       约 157µs，软件模拟（汇编 NOP 延时）约 148µs，故最终选用软件模拟"。本工程按
 *       硬件 I2C1 编写链路；若需改用软件模拟，可用 HAL GPIO 位操作实现
 *       （hal_gpio_ops->pf_write()/pf_read() 直接翻转 SCL/SDA 并配以位延时），
 *       本层接口（mem_read / read_two_bytes / read_angle）可保持不变。
 */
#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_I2C_* 模式/占空比/应答/方向/事件/标志/超时取值宏 */
#include "platform_ch32v203_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief I2C 对象前向声明。 */
typedef struct hal_i2c hal_i2c_t;

/** @brief I2C对象的生命周期状态。 */
typedef enum {
    hal_i2c_state_reset = 0,
    hal_i2c_state_ready,
    hal_i2c_state_enabled,
    hal_i2c_state_disabled,
    hal_i2c_state_error
} hal_i2c_state_enum;

/** @brief I2C接口调用返回的状态码。 */
typedef enum {
    hal_i2c_ok = 0,
    hal_i2c_err_param = -1,
    hal_i2c_err_state = -2,
    hal_i2c_err_unsupported = -3,
    hal_i2c_err_timeout = -4
} hal_i2c_status_enum;

/** @brief I2C 速率、地址与应答配置（对应标准库 I2C 初始化结构体的可移植子集）。 */
typedef struct {
    uint32_t clock_speed_hz;       /**< PLAT_I2C_* 速率，本工程 400000（400kHz） */
    uint16_t mode;                 /**< PLAT_I2C_MODE_I2C */
    uint16_t duty_cycle;           /**< PLAT_I2C_DUTY_CYCLE_2 / 16_9 */
    uint16_t own_address;          /**< 自身地址（主机模式 0x00） */
    uint16_t ack;                  /**< PLAT_I2C_ACK_ENABLE / DISABLE */
    uint16_t acknowledged_address; /**< PLAT_I2C_ACKNOWLEDGED_ADDRESS_7BIT / 10BIT */
    uint32_t timeout_count;        /**< 单阶段事件/标志轮询超时计数（0x1000） */
    void *p_user_data;             /**< 用户数据指针，未使用 */
} hal_i2c_config_t;

/** @brief I2C 对象定义（状态、实例基地址与配置）。 */
struct hal_i2c {
    hal_i2c_state_enum state;
    uintptr_t base_address;
    hal_i2c_config_t config;
};

/**
 * @brief 绑定I2C对象：保存实例基地址与配置（不做任何硬件访问）。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（PLAT_PERIPH_I2Cx）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或必需指针为空。
 */
hal_i2c_status_enum hal_i2c_bind(hal_i2c_t *p_object,
                                 uintptr_t base_address,
                                 const hal_i2c_config_t *p_config);

/** @brief 初始化I2C：时钟使能 + 复位 + 配置 + 应答使能 + 外设使能。 */
hal_i2c_status_enum hal_i2c_init(hal_i2c_t *p_object);

/** @brief 关闭并复位I2C外设。 */
hal_i2c_status_enum hal_i2c_deinit(hal_i2c_t *p_object);

/** @brief 使能I2C外设（外设使能 + 应答使能）。 */
hal_i2c_status_enum hal_i2c_enable(hal_i2c_t *p_object);

/** @brief 禁止I2C外设。 */
hal_i2c_status_enum hal_i2c_disable(hal_i2c_t *p_object);

/**
 * @brief 向指定I2C设备写 N 字节（START + 地址(写) + 数据 + STOP）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址（如 0x36）。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param data_length 待发送字节数（不得为 0）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 事件/标志等待超时（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_write(hal_i2c_t *p_object,
                                  uint8_t device_address,
                                  const uint8_t *p_tx_buf,
                                  uint16_t data_length);

/**
 * @brief 从指定I2C设备读 N 字节（START + 地址(读) + N 字节 + NACK/STOP）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数（不得为 0）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 事件/标志等待超时（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_read(hal_i2c_t *p_object,
                                 uint8_t device_address,
                                 uint8_t *p_rx_buf,
                                 uint16_t data_length);

/**
 * @brief 从 I2C 设备指定寄存器读 N 字节（写寄存器地址 + 重复 START + 读）。
 * @details 参考实现 drv_i2c.c 的 drv_SoftI2C_ReadTwoBytes（Hardware_I2C_Mode==1
 *          轮询路径）即本函数的 2 字节特例：等总线空闲 → START → 事件 EVT5 →
 *          地址(写) → EVT6 → 写寄存器地址 → EVT8_2 → 重复 START → EVT5 →
 *          地址(读) → EVT6 → 逐字节等 RXNE，末字节前关应答并发 STOP。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址（角度传感器 0x36）。
 * @param register_address 起始寄存器地址（角度寄存器 0x0C，8 位地址）。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读取字节数（不得为 0）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 事件/标志等待超时，或从机未应答（AF）。
 */
hal_i2c_status_enum hal_i2c_mem_read(hal_i2c_t *p_object,
                                     uint8_t device_address,
                                     uint8_t register_address,
                                     uint8_t *p_rx_buf,
                                     uint16_t data_length);

/**
 * @brief 向 I2C 设备指定寄存器写 N 字节（START + 地址(写) + 寄存器地址 + 数据 + STOP）。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param register_address 起始寄存器地址（8 位地址）。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param data_length 待发送字节数（不得为 0）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 事件/标志等待超时（总线已释放）。
 */
hal_i2c_status_enum hal_i2c_mem_write(hal_i2c_t *p_object,
                                      uint8_t device_address,
                                      uint8_t register_address,
                                      const uint8_t *p_tx_buf,
                                      uint16_t data_length);

/**
 * @brief 从指定寄存器连续读 2 字节（对应参考实现 drv_SoftI2C_ReadTwoBytes）。
 * @details 本函数是 hal_i2c_mem_read(..., 2) 的语义化封装，便于角度类 16 位寄存器
 *          读取；结果按"高字节在前"写入 p_data[0] / p_data[1]。
 * @param p_object 模块对象指针。
 * @param device_address 原始 7 位器件地址。
 * @param register_address 起始寄存器地址。
 * @param p_data 输出缓冲区（至少 2 字节，p_data[0] 为高字节）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 事件/标志等待超时，或从机未应答（AF）。
 */
hal_i2c_status_enum hal_i2c_read_two_bytes(hal_i2c_t *p_object,
                                           uint8_t device_address,
                                           uint8_t register_address,
                                           uint8_t *p_data);

/**
 * @brief 读取板级角度传感器的原始角度值（板级核心用途）。
 * @details 设备地址 HAL_I2C_ENCODER_DEVICE_ADDRESS（0x36）、寄存器
 *          HAL_I2C_ENCODER_ANGLE_REGISTER（0x0C）、读 2 字节，高字节在前，
 *          取低 12 位（AS5600 原始角度分辨率，掩码 HAL_I2C_ENCODER_ANGLE_MASK）。
 * @param p_object 模块对象指针。
 * @param p_raw_angle 输出原始角度（12 位）。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效或对象未就绪。
 * @retval hal_i2c_err_timeout 事件/标志等待超时，或从机未应答（AF）。
 */
hal_i2c_status_enum hal_i2c_encoder_read_angle(hal_i2c_t *p_object,
                                               uint16_t *p_raw_angle);

/** @brief 查询I2C对象当前状态。 */
hal_i2c_state_enum hal_i2c_get_state(const hal_i2c_t *p_object);

/* --------------------------------------------------------------------------
 * 板级实例：I2C1 @400kHz + PB8/PB9（重映射）角度传感器
 * -------------------------------------------------------------------------- */

/** @brief 板级目标器件地址：AS5600 类磁编码器的 7 位地址（来自 drv_i2c.c）。 */
#define HAL_I2C_ENCODER_DEVICE_ADDRESS   (0x36U)
/**
 * @brief 板级角度寄存器地址：AS5600 RAW ANGLE 起始寄存器（来自 drv_i2c.c 注释）。
 */
#define HAL_I2C_ENCODER_ANGLE_REGISTER   (0x0CU)
/** @brief 两字节寄存器读的字节数（参考实现 drv_SoftI2C_ReadTwoBytes）。 */
#define HAL_I2C_TWO_BYTES_LENGTH         (2U)
/** @brief 板级角度读取字节数：RAW ANGLE 为 2 字节（高字节在前）。 */
#define HAL_I2C_ENCODER_ANGLE_LENGTH     HAL_I2C_TWO_BYTES_LENGTH
/** @brief 板级角度有效位掩码：AS5600 原始角度 12 位。 */
#define HAL_I2C_ENCODER_ANGLE_MASK       (0x0FFFU)

/** @brief 板级编码器 I2C 实例（I2C1 @400kHz，PB8/PB9 重映射）。 */
extern hal_i2c_t hal_i2c_encoder;

/**
 * @brief 构建板级 I2C 实例（I2C1 重映射 + 引脚组 + 配置 + 使能）。
 * @details 顺序为：① 经 plat_ch32v203_gpio_ops->pf_remap_enable() 使能 AFIO 时钟与
 *          I2C1 重映射（PB8/PB9 属 I2C1 重映射引脚，必须先于引脚组构建）；
 *          ② 经 hal_gpio_ops->pf_build() 构建 hal_gpio_i2c_hardware 引脚组
 *          （PB8=SCL、PB9=SDA，复用开漏）；③ 绑定板级配置并初始化（时钟 + 配置 +
 *          应答 + 使能）。
 * @param p_object I2C 对象指针。
 * @retval hal_i2c_ok 操作成功。
 * @retval hal_i2c_err_param 参数无效。
 * @retval hal_i2c_err_state 引脚组构建失败或平台层配置失败。
 */
hal_i2c_status_enum hal_i2c_build(hal_i2c_t *p_object);

/**
 * @brief HAL 层 I2C 函数表：板级构建 / 寄存器读写 / 两字节读 / 状态。
 * @details 表中的函数指针在 hal_i2c.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_i2c_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级 I2C 实例（见 hal_i2c_build）。 */
    hal_i2c_status_enum (*pf_build)(hal_i2c_t *p_object);
    /** @brief 初始化 I2C 外设（见 hal_i2c_init）。 */
    hal_i2c_status_enum (*pf_init)(hal_i2c_t *p_object);
    /** @brief 写 N 字节（见 hal_i2c_write）。 */
    hal_i2c_status_enum (*pf_write)(hal_i2c_t *p_object,
                                    uint8_t device_address,
                                    const uint8_t *p_tx_buf,
                                    uint16_t data_length);
    /** @brief 读 N 字节（见 hal_i2c_read）。 */
    hal_i2c_status_enum (*pf_read)(hal_i2c_t *p_object,
                                   uint8_t device_address,
                                   uint8_t *p_rx_buf,
                                   uint16_t data_length);
    /** @brief 读设备寄存器 N 字节（见 hal_i2c_mem_read）。 */
    hal_i2c_status_enum (*pf_mem_read)(hal_i2c_t *p_object,
                                       uint8_t device_address,
                                       uint8_t register_address,
                                       uint8_t *p_rx_buf,
                                       uint16_t data_length);
    /** @brief 写设备寄存器 N 字节（见 hal_i2c_mem_write）。 */
    hal_i2c_status_enum (*pf_mem_write)(hal_i2c_t *p_object,
                                        uint8_t device_address,
                                        uint8_t register_address,
                                        const uint8_t *p_tx_buf,
                                        uint16_t data_length);
    /** @brief 读设备寄存器 2 字节（见 hal_i2c_read_two_bytes）。 */
    hal_i2c_status_enum (*pf_read_two_bytes)(hal_i2c_t *p_object,
                                            uint8_t device_address,
                                            uint8_t register_address,
                                            uint8_t *p_data);
    /** @brief 查询对象状态（见 hal_i2c_get_state）。 */
    hal_i2c_state_enum (*pf_get_state)(const hal_i2c_t *p_object);
} hal_i2c_ops_t;

/** @brief HAL 层 I2C 函数表指针（在 hal_i2c.c 中定义并绑定）。 */
extern const hal_i2c_ops_t *const hal_i2c_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_I2C_H__ */
