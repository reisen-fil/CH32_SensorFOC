/**
 * @file platform_ch32v203_spi.h
 * @brief CH32V203 SPI 的 MCU 抽象接口（取值宏 + 类型接入宏 + 操作函数表）。
 * @details 与 GPIO 平台层同构，本层提供三类内容：
 *          - 取值宏：数据方向 / 主从 / 帧宽 / 极性 / 相位 / 片选 / 分频 / 位序、
 *            编码器片选引脚与 SPI1 重映射；
 *          - 类型接入宏：PLAT_SPI_INSTANCE_TYPE / PLAT_SPI_INIT_TYPE，把厂商类型名
 *            收敛到宏体中（延迟展开），使 platform_ch32v203_spi.c 内只出现 PLAT_* 名字；
 *          - 操作函数表 platform_ch32v203_spi_ops_t：把原先"以宏形式直接绑定厂商库"
 *            的操作接口（端口取址、SPI_Init、使能/禁用、复位、寄存器级收发与标志
 *            查询）改为在 platform_ch32v203_spi.c 中实现的函数，并通过
 *            plat_ch32v203_spi_ops 表对外绑定，由 hal_spi.c 调用。
 *          SPI 的对象模型、板级实例与初始化函数实现于 hal_spi.c / hal_spi.h。
 * @note 片选与重映射属于 GPIO/AFIO 级操作，统一经 GPIO 操作函数表
 *       （plat_ch32v203_gpio_ops）完成，SPI 表内不再重复表达。
 * @note 操作函数表只外露 MCU 抽象层面的收发原语（pf_send_data / pf_receive_data /
 *       pf_get_flag）与配置动作；"按帧宽步进、标志轮询、超时、片选夹持"等收发细节
 *       在 hal_spi.c 中封装，本层不参与。
 */
#ifndef __PLATFORM_CH32V203_SPI_H__
#define __PLATFORM_CH32V203_SPI_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203_gpio.h"

/* 注：SPI 初始化结构体平台别名不再使用 typedef，改由本头的类型接入宏
 * PLAT_SPI_INIT_TYPE 提供；宏体延迟展开，故不必在公共头
 * platform_ch32v203.h 中提前声明，解析期也不再依赖厂商类型。 */

/** @brief SPI数据线方向常量映射。 */
#define PLAT_SPI_DIRECTION_2_LINES_FULL_DUPLEX SPI_Direction_2Lines_FullDuplex
#define PLAT_SPI_DIRECTION_2_LINES_RX_ONLY    SPI_Direction_2Lines_RxOnly
#define PLAT_SPI_DIRECTION_1_LINE_RX          SPI_Direction_1Line_Rx
#define PLAT_SPI_DIRECTION_1_LINE_TX          SPI_Direction_1Line_Tx

/** @brief SPI主从模式常量映射。 */
#define PLAT_SPI_MODE_MASTER                  SPI_Mode_Master
#define PLAT_SPI_MODE_SLAVE                   SPI_Mode_Slave

/** @brief SPI数据帧宽度常量映射。 */
#define PLAT_SPI_DATA_SIZE_8BIT               SPI_DataSize_8b
#define PLAT_SPI_DATA_SIZE_16BIT              SPI_DataSize_16b

/** @brief SPI时钟极性常量映射。 */
#define PLAT_SPI_CPOL_LOW                     SPI_CPOL_Low
#define PLAT_SPI_CPOL_HIGH                    SPI_CPOL_High

/** @brief SPI时钟相位常量映射。 */
#define PLAT_SPI_CPHA_FIRST_EDGE              SPI_CPHA_1Edge
#define PLAT_SPI_CPHA_SECOND_EDGE             SPI_CPHA_2Edge

/** @brief SPI片选方式常量映射。 */
#define PLAT_SPI_NSS_SOFTWARE                 SPI_NSS_Soft
#define PLAT_SPI_NSS_HARDWARE                 SPI_NSS_Hard

/** @brief SPI波特率分频常量映射。 */
#define PLAT_SPI_BAUD_RATE_PRESCALER_2        SPI_BaudRatePrescaler_2
#define PLAT_SPI_BAUD_RATE_PRESCALER_4        SPI_BaudRatePrescaler_4
#define PLAT_SPI_BAUD_RATE_PRESCALER_8        SPI_BaudRatePrescaler_8
#define PLAT_SPI_BAUD_RATE_PRESCALER_16       SPI_BaudRatePrescaler_16
#define PLAT_SPI_BAUD_RATE_PRESCALER_32       SPI_BaudRatePrescaler_32
#define PLAT_SPI_BAUD_RATE_PRESCALER_64       SPI_BaudRatePrescaler_64
#define PLAT_SPI_BAUD_RATE_PRESCALER_128      SPI_BaudRatePrescaler_128
#define PLAT_SPI_BAUD_RATE_PRESCALER_256      SPI_BaudRatePrescaler_256

/** @brief SPI首位传输方向常量映射。 */
#define PLAT_SPI_FIRST_BIT_MSB                SPI_FirstBit_MSB
#define PLAT_SPI_FIRST_BIT_LSB                SPI_FirstBit_LSB

/** @brief SPI状态标志常量映射（供寄存器级收发直接调用标准库接口时使用）。 */
#define PLAT_SPI_FLAG_RXNE                    SPI_I2S_FLAG_RXNE
#define PLAT_SPI_FLAG_TXE                     SPI_I2S_FLAG_TXE
#define PLAT_SPI_FLAG_BSY                     SPI_I2S_FLAG_BSY

/** @brief SPI标志状态值映射。 */
#define PLAT_SPI_STATUS_SET                   PLAT_SET
#define PLAT_SPI_STATUS_RESET                 PLAT_RESET

/** @brief MT6701编码器片选GPIO端口。 */
#define PLAT_SPI_ENCODER_CS_PORT              PLAT_PERIPH_GPIOA
/** @brief MT6701编码器片选GPIO引脚。 */
#define PLAT_SPI_ENCODER_CS_PIN               PLAT_GPIO_PIN_15

/* 注：片选夹持与 SPI1 重映射不再提供宏封装，改为：
 *     - platform_ch32v203_spi.c 的 pf_transfer 内直接调用
 *       plat_ch32v203_gpio_ops->pf_write()；
 *     - hal_spi.c 的 hal_spi_init 内直接调用 plat_ch32v203_gpio_ops 的
 *       pf_remap_enable() / pf_write()。
 *     重映射与片选取值仍由上面的取值宏与 PLAT_GPIO_REMAP_* 提供。 */

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_spi.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_SPI_INSTANCE_TYPE             SPI_TypeDef
#define PLAT_SPI_INIT_TYPE                 SPI_InitTypeDef

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SPI 平台配置（原 PLAT_SPI_INIT 宏的实参集合）。
 * @details 字段取值来自上面的 PLAT_SPI_* 取值宏；方向固定为全双工，
 *          CRC 多项式固定为使能位，故不在此结构内。
 */
typedef struct {
    uint16_t data_size;          /**< PLAT_SPI_DATA_SIZE_8BIT / 16BIT */
    uint16_t mode;               /**< PLAT_SPI_MODE_MASTER / SLAVE */
    uint16_t clock_polarity;     /**< PLAT_SPI_CPOL_LOW / HIGH */
    uint16_t clock_phase;        /**< PLAT_SPI_CPHA_FIRST_EDGE / SECOND_EDGE */
    uint16_t nss;                /**< PLAT_SPI_NSS_SOFTWARE / HARDWARE */
    uint16_t baud_rate_prescaler;/**< PLAT_SPI_BAUD_RATE_PRESCALER_x */
    bool     is_msb_first;       /**< true = PLAT_SPI_FIRST_BIT_MSB */
} platform_ch32v203_spi_config_t;

/**
 * @brief CH32V203 SPI 操作函数表（原宏接口的函数化绑定）。
 * @details 全部函数以"实例基地址 + 值参数"形式描述，不含厂商类型与 HAL 类型；
 *          返回 bool，true 表示成功。
 */
typedef struct {
    /** @brief 使能 SPI 实例所在总线的外设时钟。 */
    bool (*pf_clock_enable)(uintptr_t instance);
    /** @brief 复位并配置 SPI（方向固定全双工）。 */
    bool (*pf_config)(uintptr_t instance,
                      const platform_ch32v203_spi_config_t *p_config);
    /** @brief 使能 SPI 收发。 */
    bool (*pf_enable)(uintptr_t instance);
    /** @brief 禁用 SPI 收发。 */
    bool (*pf_disable)(uintptr_t instance);
    /** @brief 禁用并把 SPI 寄存器恢复为复位状态。 */
    bool (*pf_deinit)(uintptr_t instance);
    /** @brief 发送一个帧（MCU 抽象发送接口的简单函数封装）。 */
    void (*pf_send_data)(uintptr_t instance, uint16_t data);
    /** @brief 接收一个帧（MCU 抽象接收接口的简单函数封装）。 */
    uint16_t (*pf_receive_data)(uintptr_t instance);
    /**
     * @brief 查询状态标志（MCU 抽象标志查询接口的简单函数封装）。
     * @return PLAT_SPI_STATUS_SET 表示置位，PLAT_SPI_STATUS_RESET 表示复位。
     */
    uint16_t (*pf_get_flag)(uintptr_t instance, uint16_t flag);
} platform_ch32v203_spi_ops_t;

/**
 * @brief CH32V203 SPI 操作函数表指针（在 platform_ch32v203_spi.c 中组装）。
 * @details HAL 层通过该指针调用平台 SPI 接口；函数表本体对本层私有。
 */
extern const platform_ch32v203_spi_ops_t *const plat_ch32v203_spi_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_SPI_H__ */
