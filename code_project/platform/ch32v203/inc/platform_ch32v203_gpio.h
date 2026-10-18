/**
 * @file platform_ch32v203_gpio.h
 * @brief CH32V203 GPIO 的 MCU 抽象接口（取值宏 + 类型接入宏 + 操作函数表）。
 * @details 本层提供三类内容：
 *          - 取值宏：引脚号 / 速率 / 模式 / 电平 / 复用重映射，供配置表字段填写；
 *          - 类型接入宏：PLAT_GPIO_*_TYPE，把厂商类型名收敛到宏体中（延迟展开），
 *            使 platform_ch32v203_gpio.c 内只出现 PLAT_* 名字；
 *          - 操作函数表 platform_ch32v203_gpio_ops_t：把原先"以宏形式直接绑定厂商
 *            库"的操作接口（端口取址、GPIO_Init、置位、复位、写电平、读电平、
 *            重映射）改为在 platform_ch32v203_gpio.c 中实现的函数，并通过
 *            plat_ch32v203_gpio_ops 表对外绑定，由 hal_gpio.c 调用。
 *          GPIO 的对象模型、双视图组配置（配置视图合并掩码 + 操作视图逐脚读写）、
 *          板级引脚表与各组初始化函数实现于 hal_gpio.c / hal_gpio.h。
 * @note 读操作语义为"掩码内任意一位为高"，逐脚读必须使用单 bit 掩码，
 *       该约束由 hal_gpio.c 的操作视图保证。
 */
#ifndef __PLATFORM_CH32V203_GPIO_H__
#define __PLATFORM_CH32V203_GPIO_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/* 注：GPIO 初始化结构体平台别名不再使用 typedef，改由本头的类型接入宏
 * PLAT_GPIO_INIT_TYPE 提供；宏体延迟展开，故不必在公共头
 * platform_ch32v203.h 中提前声明，公共头也无需感知厂商类型。 */

/** @brief GPIO引脚编号。 */
#define PLAT_GPIO_PIN_0             GPIO_Pin_0
#define PLAT_GPIO_PIN_1             GPIO_Pin_1
#define PLAT_GPIO_PIN_2             GPIO_Pin_2
#define PLAT_GPIO_PIN_3             GPIO_Pin_3
#define PLAT_GPIO_PIN_4             GPIO_Pin_4
#define PLAT_GPIO_PIN_5             GPIO_Pin_5
#define PLAT_GPIO_PIN_6             GPIO_Pin_6
#define PLAT_GPIO_PIN_7             GPIO_Pin_7
#define PLAT_GPIO_PIN_8             GPIO_Pin_8
#define PLAT_GPIO_PIN_9             GPIO_Pin_9
#define PLAT_GPIO_PIN_10            GPIO_Pin_10
#define PLAT_GPIO_PIN_11            GPIO_Pin_11
#define PLAT_GPIO_PIN_12            GPIO_Pin_12
#define PLAT_GPIO_PIN_13            GPIO_Pin_13
#define PLAT_GPIO_PIN_14            GPIO_Pin_14
#define PLAT_GPIO_PIN_15            GPIO_Pin_15
#define PLAT_GPIO_PIN_ALL           GPIO_Pin_All

/** @brief GPIO输出速度配置。 */
#define PLAT_GPIO_SPEED_2MHZ        GPIO_Speed_2MHz
#define PLAT_GPIO_SPEED_10MHZ       GPIO_Speed_10MHz
#define PLAT_GPIO_SPEED_50MHZ       GPIO_Speed_50MHz

/** @brief GPIO输入输出模式。 */
#define PLAT_GPIO_MODE_ANALOG       GPIO_Mode_AIN
#define PLAT_GPIO_MODE_INPUT_FLOAT  GPIO_Mode_IN_FLOATING
#define PLAT_GPIO_MODE_INPUT_PULL_DOWN GPIO_Mode_IPD
#define PLAT_GPIO_MODE_INPUT_PULL_UP GPIO_Mode_IPU
#define PLAT_GPIO_MODE_OUTPUT_OD    GPIO_Mode_Out_OD
#define PLAT_GPIO_MODE_OUTPUT_PP    GPIO_Mode_Out_PP
#define PLAT_GPIO_MODE_AF_OD        GPIO_Mode_AF_OD
#define PLAT_GPIO_MODE_AF_PP        GPIO_Mode_AF_PP

/** @brief GPIO电平取值。 */
#define PLAT_GPIO_LEVEL_RESET       Bit_RESET
#define PLAT_GPIO_LEVEL_SET         Bit_SET

/** @brief 外设复用重映射取值（作为 pf_remap_enable() 的 remap 实参）。 */
#define PLAT_GPIO_REMAP_I2C1        GPIO_Remap_I2C1
#define PLAT_GPIO_REMAP_SPI1        GPIO_Remap_SPI1

/** @brief AFIO 外设时钟（重映射前置条件，作为 pf_remap_enable() 的 afio_rcc 实参）。 */
#define PLAT_GPIO_REMAP_AFIO_RCC    PLAT_RCC_APB2_AFIO

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_gpio.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_GPIO_INSTANCE_TYPE            GPIO_TypeDef
#define PLAT_GPIO_INIT_TYPE                GPIO_InitTypeDef
#define PLAT_GPIO_MODE_TYPE                GPIOMode_TypeDef
#define PLAT_GPIO_SPEED_TYPE               GPIOSpeed_TypeDef

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CH32V203 GPIO 操作函数表（原宏接口的函数化绑定）。
 * @details 表中的函数均为"值参数 + 端口基地址"形式，不含厂商类型，
 *          因此 HAL 层无需包含 stdperiph 头文件即可完成 GPIO 访问。
 */
typedef struct {
    /** @brief 使能 GPIO 端口时钟。 */
    void (*pf_clock_enable)(uint32_t rcc);
    /** @brief 配置端口引脚的速率与模式（掩码可含多位同模式引脚）。 */
    void (*pf_pin_config)(uintptr_t port, uint16_t pin_mask,
                          uint8_t speed, uint8_t mode);
    /** @brief 置位：掩码内所有引脚输出高（原子写 BSHR）。 */
    void (*pf_set)(uintptr_t port, uint16_t pin_mask);
    /** @brief 复位：掩码内所有引脚输出低（原子写 BCR）。 */
    void (*pf_reset)(uintptr_t port, uint16_t pin_mask);
    /** @brief 写电平：掩码内所有引脚写成同一电平。 */
    void (*pf_write)(uintptr_t port, uint16_t pin_mask, bool level);
    /** @brief 读电平：掩码内任意一位为高则返回 true（逐脚读需单 bit 掩码）。 */
    bool (*pf_read)(uintptr_t port, uint16_t pin_mask);
    /** @brief 使能 AFIO 时钟并应用外设复用重映射。 */
    void (*pf_remap_enable)(uint32_t afio_rcc, uint32_t remap);
} platform_ch32v203_gpio_ops_t;

/**
 * @brief CH32V203 GPIO 操作函数表指针（在 platform_ch32v203_gpio.c 中组装）。
 * @details HAL 层通过该指针调用平台 GPIO 接口；函数表本体对本层私有。
 */
extern const platform_ch32v203_gpio_ops_t *const plat_ch32v203_gpio_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_GPIO_H__ */
