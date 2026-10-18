/**
 * @file platform_ch32v203_gpio.c
 * @brief CH32V203 GPIO 平台适配实现（函数表绑定的 MCU 抽象接口）。
 * @details 原先 platform_ch32v203_gpio.h 中以宏形式直接绑定厂商库的操作接口
 *          （端口取址 / GPIO_Init / 置位 / 复位 / 写电平 / 读电平 / 重映射）
 *          在此实现为独立函数，并组装成 plat_ch32v203_gpio_ops 函数表对外提供；
 *          hal_gpio.c 通过该表访问 GPIO 硬件，头文件中不再暴露厂商调用宏，
 *          HAL 层因此不必包含 stdperiph 类型。
 */
#include "platform_ch32v203_gpio.h"

/** @brief 由端口基地址取得 GPIO 端口实例。 */
#define PLAT_GPIO_PORT(base_address) ((PLAT_GPIO_INSTANCE_TYPE *)(base_address))

/**
 * @brief 使能 GPIO 端口时钟。
 * @param rcc APB2 外设时钟位，取 PLAT_RCC_APB2_GPIOx。
 */
static void plat_ch32v203_gpio_clock_enable(uint32_t rcc)
{
    PLAT_RCC_APB2_CLOCK_ENABLE(rcc);
}

/**
 * @brief 配置端口引脚的速率与模式（原 PLAT_GPIO_INIT 宏）。
 * @param port 端口基地址，取 PLAT_PERIPH_GPIOx。
 * @param pin_mask 引脚掩码，可含多位同模式引脚。
 * @param speed 输出速率，取 PLAT_GPIO_SPEED_xxx。
 * @param mode 工作模式，取 PLAT_GPIO_MODE_xxx。
 */
static void plat_ch32v203_gpio_pin_config(uintptr_t port,
                                          uint16_t pin_mask,
                                          uint8_t speed,
                                          uint8_t mode)
{
    PLAT_GPIO_INIT_TYPE init_config;

    init_config.GPIO_Pin = pin_mask;
    init_config.GPIO_Speed = (PLAT_GPIO_SPEED_TYPE)speed;
    init_config.GPIO_Mode = (PLAT_GPIO_MODE_TYPE)mode;

    GPIO_Init(PLAT_GPIO_PORT(port), &init_config);
}

/**
 * @brief 置位：掩码内所有引脚输出高（原 PLAT_GPIO_SET_BITS 宏）。
 * @param port 端口基地址。
 * @param pin_mask 引脚掩码。
 */
static void plat_ch32v203_gpio_set(uintptr_t port, uint16_t pin_mask)
{
    GPIO_SetBits(PLAT_GPIO_PORT(port), pin_mask);
}

/**
 * @brief 复位：掩码内所有引脚输出低（原 PLAT_GPIO_RESET_BITS 宏）。
 * @param port 端口基地址。
 * @param pin_mask 引脚掩码。
 */
static void plat_ch32v203_gpio_reset(uintptr_t port, uint16_t pin_mask)
{
    GPIO_ResetBits(PLAT_GPIO_PORT(port), pin_mask);
}

/**
 * @brief 写电平：掩码内所有引脚写成同一电平（原 PLAT_GPIO_WRITE_LEVEL 宏）。
 * @param port 端口基地址。
 * @param pin_mask 引脚掩码。
 * @param level true 输出高，false 输出低。
 */
static void plat_ch32v203_gpio_write(uintptr_t port,
                                     uint16_t pin_mask,
                                     bool level)
{
    GPIO_WriteBit(PLAT_GPIO_PORT(port),
                  pin_mask,
                  (level ? PLAT_BIT_SET : PLAT_BIT_RESET));
}

/**
 * @brief 读电平：掩码内任意一位为高则返回 true（原 PLAT_GPIO_READ_LEVEL 宏）。
 * @param port 端口基地址。
 * @param pin_mask 引脚掩码（逐脚读应传单 bit）。
 * @return 掩码内任意一位为高返回 true，否则 false。
 */
static bool plat_ch32v203_gpio_read(uintptr_t port, uint16_t pin_mask)
{
    return (GPIO_ReadInputDataBit(PLAT_GPIO_PORT(port), pin_mask) != 0U);
}

/**
 * @brief 使能 AFIO 时钟并应用外设复用重映射。
 * @param afio_rcc AFIO 外设时钟位，取 PLAT_GPIO_REMAP_AFIO_RCC。
 * @param remap 重映射取值，取 PLAT_GPIO_REMAP_xxx。
 */
static void plat_ch32v203_gpio_remap_enable(uint32_t afio_rcc, uint32_t remap)
{
    PLAT_RCC_APB2_CLOCK_ENABLE(afio_rcc);
    GPIO_PinRemapConfig(remap, PLAT_ENABLE);
}

/** @brief CH32V203 GPIO 操作函数表（本文件私有）。 */
static const platform_ch32v203_gpio_ops_t s_gpio_ops = {
    .pf_clock_enable = plat_ch32v203_gpio_clock_enable,
    .pf_pin_config = plat_ch32v203_gpio_pin_config,
    .pf_set = plat_ch32v203_gpio_set,
    .pf_reset = plat_ch32v203_gpio_reset,
    .pf_write = plat_ch32v203_gpio_write,
    .pf_read = plat_ch32v203_gpio_read,
    .pf_remap_enable = plat_ch32v203_gpio_remap_enable
};

/** @brief CH32V203 GPIO 操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_gpio_ops_t *const plat_ch32v203_gpio_ops = &s_gpio_ops;