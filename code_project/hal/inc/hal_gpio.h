/**
 * @file hal_gpio.h
 * @brief GPIO 接口（含 CH32V203 GPIO 适配与板级引脚表）。
 * @details 本模块由"GPIO 对象操作"与"CH32V203 GPIO 适配"合并而成：
 *          - 对象操作函数同层实现：hal_gpio_init 直接校验对象并访问寄存器；
 *            hal_gpio_write / hal_gpio_read 以"组对象 + 引脚索引"为入口，
 *            函数内部调用 hal_gpio_get_pin() 完成引脚配置判断后再访问寄存器，
 *            全模块无操作表分发层；
 *          - 双视图：配置视图（同属性引脚合并成多位掩码，每组一次 GPIO_Init）+
 *            操作视图（每脚一个单 bit 掩码对象，供逐脚读写）；
 *          - 板级引脚表：对外可见，由调用方用 hal_gpio_build() 构建进对应组对象
 *            （不再提供 hal_gpio_xxx_init() 之类的二次封装）。
 *          表项字段取值来自 MCU 抽象接口 platform_ch32v203_gpio.h 的 PLAT_GPIO_* 宏，
 *          硬件访问经平台 GPIO 操作函数表指针 plat_ch32v203_gpio_ops。
 * @note 写/读都作用在操作视图对象的单 bit 掩码上：写为 BSHR/BCR 原子写，
 *       读符合"掩码内任意一位为高"的硬件语义。
 */
#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_GPIO_* 引脚/模式/速率/电平映射与寄存器访问宏 */
#include "platform_ch32v203_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hal_gpio hal_gpio_t;

/** @brief GPIO对象的生命周期状态。 */
typedef enum {
    hal_gpio_state_reset = 0,
    hal_gpio_state_ready,
    hal_gpio_state_enabled,
    hal_gpio_state_disabled,
    hal_gpio_state_error
} hal_gpio_state_enum;

/** @brief GPIO接口调用返回的状态码。 */
typedef enum {
    hal_gpio_ok = 0,
    hal_gpio_err_param = -1,
    hal_gpio_err_state = -2,
    hal_gpio_err_unsupported = -3,
    hal_gpio_err_timeout = -4
} hal_gpio_status_enum;

/** @brief GPIO端口时钟、引脚、电气模式和初始电平配置。 */
typedef struct {
    uint32_t gpio_rcc;
    uint32_t pin_mask;
    uint8_t speed;
    uint8_t mode;
    bool initial_level;
} hal_gpio_config_t;

/** @brief GPIO 对象定义（对象操作函数为同层直接实现，无操作表分发）。 */
struct hal_gpio {
    hal_gpio_state_enum state;
    uintptr_t base_address;
    hal_gpio_config_t config;
    bool level;
};

/** @brief 配置视图容量：单个 GPIO 组内可合并的 (端口,模式,速率,电平) 组数上限。 */
#define HAL_GPIO_GROUP_OBJECT_MAX (4U)

/** @brief 操作视图容量：单个 GPIO 组内可逐脚操作的引脚数上限。 */
#define HAL_GPIO_PIN_OBJECT_MAX   (8U)

/** @brief 由 PLAT_PERIPH_GPIOx 取得 hal_gpio_bind() 所需的端口基地址。 */
#define HAL_GPIO_PORT(port)       ((uintptr_t)(port))

/**
 * @brief 单个引脚的完整属性描述表项。
 * @details pin_mask 只填 1 个 bit；同 (port, rcc, mode, speed, initial_level)
 *          的表项在构建时会被合并进同一个配置视图对象。
 */
typedef struct {
    uintptr_t port;          /**< PLAT_PERIPH_GPIOx */
    uint32_t  rcc;           /**< PLAT_RCC_APB2_GPIOx */
    uint16_t  pin_mask;      /**< 单 bit：PLAT_GPIO_PIN_n */
    uint8_t   mode;          /**< PLAT_GPIO_MODE_xxx */
    uint8_t   speed;         /**< PLAT_GPIO_SPEED_xxx */
    bool      initial_level; /**< 仅输出模式生效 */
} hal_gpio_pin_desc_t;

/** @brief 常用表项写法：50MHz。port 传 PLAT_PERIPH_GPIOx（指针型，需转 uintptr_t）。 */
#define HAL_GPIO_PIN_DESC(port, rcc, pin, mode, level) \
    { (uintptr_t)(port), (rcc), (pin), (mode), PLAT_GPIO_SPEED_50MHZ, (level) }

/** @brief 求引脚描述表项数量。 */
#define HAL_GPIO_TABLE_COUNT(table) \
    ((uint8_t)(sizeof(table) / sizeof((table)[0])))

/**
 * @brief GPIO 组对象：一个配置视图 + 一个操作视图。
 * @details group_object 由 hal_gpio_build() 合并掩码并初始化；
 *          pin_object 与描述表逐项对应，只做逐脚操作。
 */
typedef struct {
    hal_gpio_t group_object[HAL_GPIO_GROUP_OBJECT_MAX];
    hal_gpio_t pin_object[HAL_GPIO_PIN_OBJECT_MAX];
    uint8_t group_count;
    uint8_t pin_count;
} hal_gpio_group_t;

/** @brief 电机 PWM 组引脚顺序（与 s_tim_pin_table 逐项对应）。 */
typedef enum {
    hal_gpio_tim_pin_high_u = 0,    /**< PA8  TIM1_CH1  高端 U */
    hal_gpio_tim_pin_high_v,        /**< PA9  TIM1_CH2  高端 V */
    hal_gpio_tim_pin_high_w,        /**< PA10 TIM1_CH3  高端 W */
    hal_gpio_tim_pin_low_u,         /**< PB13 TIM1_CH1N 低端 U */
    hal_gpio_tim_pin_low_v,         /**< PB14 TIM1_CH2N 低端 V */
    hal_gpio_tim_pin_low_w,         /**< PB15 TIM1_CH3N 低端 W */
    hal_gpio_tim_pin_count
} hal_gpio_tim_pin_enum;

/** @brief 运放组引脚顺序。 */
typedef enum {
    hal_gpio_opamp_pin_a_in_p = 0,  /**< PA5 内部运放输入 */
    hal_gpio_opamp_pin_a_in_n,      /**< PA6 内部运放输入 */
    hal_gpio_opamp_pin_a_in_2,      /**< PA7 内部运放输入 */
    hal_gpio_opamp_pin_a_out_ext,   /**< PA3 外部运放输出 */
    hal_gpio_opamp_pin_a_out_int,   /**< PA4 内部运放输出 */
    hal_gpio_opamp_pin_b_in,        /**< PB0 内部运放输入 */
    hal_gpio_opamp_pin_b_out,       /**< PB1 内部运放输出 */
    hal_gpio_opamp_pin_count
} hal_gpio_opamp_pin_enum;

/** @brief ADC 常规通道组引脚顺序。 */
typedef enum {
    hal_gpio_adc_pin_bus_voltage = 0,   /**< PA0 母线电压采样 */
    hal_gpio_adc_pin_potentiometer,     /**< PA1 控制电位器采样 */
    hal_gpio_adc_pin_ntc,               /**< PA2 NTC 温度采样 */
    hal_gpio_adc_pin_count
} hal_gpio_adc_pin_enum;

/** @brief CAN 组引脚顺序。 */
typedef enum {
    hal_gpio_can_pin_tx = 0,        /**< PA12 CAN_TX */
    hal_gpio_can_pin_rx,            /**< PA11 CAN_RX */
    hal_gpio_can_pin_count
} hal_gpio_can_pin_enum;

/** @brief USART 组引脚顺序。 */
typedef enum {
    hal_gpio_usart_pin_tx = 0,      /**< PB10 USART3_TX */
    hal_gpio_usart_pin_rx,          /**< PB11 USART3_RX */
    hal_gpio_usart_pin_count
} hal_gpio_usart_pin_enum;

/** @brief 硬件 I2C 组引脚顺序。 */
typedef enum {
    hal_gpio_i2c_pin_scl = 0,       /**< PB8 I2C1_SCL（重映射） */
    hal_gpio_i2c_pin_sda,           /**< PB9 I2C1_SDA（重映射） */
    hal_gpio_i2c_pin_count
} hal_gpio_i2c_pin_enum;

/** @brief 硬件 SPI 组引脚顺序。 */
typedef enum {
    hal_gpio_spi_pin_sck = 0,       /**< PB3 SPI1_SCK（重映射） */
    hal_gpio_spi_pin_miso,          /**< PB4 SPI1_MISO（重映射） */
    hal_gpio_spi_pin_mosi,          /**< PB5 SPI1_MOSI（重映射） */
    hal_gpio_spi_pin_cs,            /**< PA15 软件片选 */
    hal_gpio_spi_pin_count
} hal_gpio_spi_pin_enum;

/** @brief 指示灯组引脚顺序。 */
typedef enum {
    hal_gpio_led_pin_green = 0,     /**< PC14 绿灯 */
    hal_gpio_led_pin_red,           /**< PC15 红灯 */
    hal_gpio_led_pin_count
} hal_gpio_led_pin_enum;

/** @brief 测试io引脚顺序。 */
typedef enum {
    hal_gpio_test_pin_1 = 0,        /**< PB6 */
    hal_gpio_test_pin_2,            /**< PB7 */
} hal_gpio_test_pin_enum;

/**
 * @brief 绑定GPIO对象：保存端口地址、引脚配置与初始状态。
 * @param p_object 模块对象指针。
 * @param base_address 外设寄存器基地址（传 PLAT_PERIPH_GPIOx 时用 HAL_GPIO_PORT() 转换）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或必需指针为空。
 */
hal_gpio_status_enum hal_gpio_bind(
    hal_gpio_t *p_object,
    uintptr_t base_address,
    const hal_gpio_config_t *p_config);

/** @brief 配置GPIO端口并设置初始电平。 */
hal_gpio_status_enum hal_gpio_init(hal_gpio_t *p_object);

/**
 * @brief 输出指定引脚的电平（引脚配置判断在函数内部完成）。
 * @details 函数内部调用 hal_gpio_get_pin() 把引脚索引解析成单 bit 操作视图对象
 *          并做越界检查，随后直接写寄存器（BSHR/BCR 原子掩码写）。
 * @param p_driver GPIO 组对象指针。
 * @param pin_index 引脚索引，取该组 hal_gpio_xxx_pin_enum。
 * @param level 待设置的 GPIO 电平。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 组对象为空或引脚索引越界。
 */
hal_gpio_status_enum hal_gpio_write(
    hal_gpio_group_t *p_driver,
    uint8_t pin_index,
    bool level);

/**
 * @brief 读取指定引脚的电平（引脚配置判断在函数内部完成）。
 * @details 函数内部调用 hal_gpio_get_pin() 取得单 bit 操作视图对象，读取掩码恒
 *          为单 bit，符合"掩码内任意一位为高"的硬件读语义。
 * @param p_driver GPIO 组对象指针。
 * @param pin_index 引脚索引，取该组 hal_gpio_xxx_pin_enum。
 * @param p_level 输出电平指针。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 组对象为空、索引越界或输出指针为空。
 */
hal_gpio_status_enum hal_gpio_read(
    hal_gpio_group_t *p_driver,
    uint8_t pin_index,
    bool *p_level);

/**
 * @brief 翻转GPIO当前输出电平。
 * @param p_object 模块对象指针。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_unsupported 当前平台未实现翻转。
 */
hal_gpio_status_enum hal_gpio_toggle(hal_gpio_t *p_object);

/** @brief 查询GPIO对象当前状态。 */
hal_gpio_state_enum hal_gpio_get_state(const hal_gpio_t *p_object);

/**
 * @brief 由引脚描述表构建 GPIO 组对象（配置视图 + 操作视图）。
 * @details 逐项绑定操作视图对象；把同属性表项合并进配置视图的多位掩码；
 *          全部合并完成后，每个合并组只调用一次 GPIO_Init。
 * @param p_driver GPIO 组对象指针。
 * @param p_pins 引脚描述表首地址。
 * @param pin_count 表项数量，不得超过 HAL_GPIO_PIN_OBJECT_MAX。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效，或引脚/合并组数量超出容量。
 */
hal_gpio_status_enum hal_gpio_build(
    hal_gpio_group_t *p_driver,
    const hal_gpio_pin_desc_t *p_pins,
    uint8_t pin_count);

/**
 * @brief 运行期追加一个引脚并立即完成配置。
 * @details 适用于启动后按需使能的引脚：追加操作视图对象、合并进配置视图后
 *          立即重新执行该组的 GPIO_Init。
 * @param p_driver GPIO 组对象指针。
 * @param p_pin 单引脚描述表项。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或容量不足。
 * @retval hal_gpio_err_state 追加成功但未找到所属配置视图。
 */
hal_gpio_status_enum hal_gpio_mcu_init(
    hal_gpio_group_t *p_driver,
    const hal_gpio_pin_desc_t *p_pin);

/**
 * @brief 取操作视图中的引脚对象。
 * @details hal_gpio_write() / hal_gpio_read() 内部即调用本函数完成引脚配置判断；
 *          调用方也可直接用它取出引脚对象（例如读取其 level 字段或掩码）。
 * @param p_driver GPIO 组对象指针。
 * @param pin_index 引脚索引，取该组 hal_gpio_xxx_pin_enum。
 * @return 引脚对象指针；越界或对象为空时返回 NULL。
 */
hal_gpio_t *hal_gpio_get_pin(hal_gpio_group_t *p_driver, uint8_t pin_index);

/**
 * @brief 取配置视图中的合并组对象。
 * @details 合并组对象由 hal_gpio_build() 初始化，其掩码含本组全部同属性引脚；
 *          对它调用 hal_gpio_write() 即为整组同电平写。
 * @param p_driver GPIO 组对象指针。
 * @param group_index 合并组索引，0 起，顺序为表项首次出现的顺序。
 * @return 组对象指针；越界或对象为空时返回 NULL。
 */
hal_gpio_t *hal_gpio_get_group(hal_gpio_group_t *p_driver, uint8_t group_index);

/** @brief 取操作视图已构建的引脚数。 */
uint8_t hal_gpio_get_pin_count(const hal_gpio_group_t *p_driver);

/** @brief 取配置视图已合并的组数。 */
uint8_t hal_gpio_get_group_count(const hal_gpio_group_t *p_driver);

/**
 * @brief HAL 层 GPIO 函数表：对外暴露的构建 / 写 / 读接口。
 * @details 表中的函数指针在 hal_gpio.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_gpio_ops 指针统一调用。
 */
typedef struct {
    /** @brief 由引脚描述表构建 GPIO 组对象（见 hal_gpio_build）。 */
    hal_gpio_status_enum (*pf_build)(hal_gpio_group_t *p_driver,
                                     const hal_gpio_pin_desc_t *p_pins,
                                     uint8_t pin_count);
    /** @brief 输出指定引脚电平（见 hal_gpio_write）。 */
    hal_gpio_status_enum (*pf_write)(hal_gpio_group_t *p_driver,
                                     uint8_t pin_index,
                                     bool level);
    /** @brief 读取指定引脚电平（见 hal_gpio_read）。 */
    hal_gpio_status_enum (*pf_read)(hal_gpio_group_t *p_driver,
                                    uint8_t pin_index,
                                    bool *p_level);
} hal_gpio_ops_t;

/** @brief HAL 层 GPIO 函数表指针（在 hal_gpio.c 中定义并绑定）。 */
extern const hal_gpio_ops_t *const hal_gpio_ops;

/* --------------------------------------------------------------------------
 * 板级引脚表与组对象：调用方用 hal_gpio_build() 直接把表构建进对应组对象，
 * 例如 hal_gpio_build(&hal_gpio_led, hal_gpio_led_pin_table,
 *                     HAL_GPIO_TABLE_COUNT(hal_gpio_led_pin_table));
 * 注：此处带元素个数声明，便于调用方用 HAL_GPIO_TABLE_COUNT() 取表长，
 *     并与 .c 中的初始化项数形成编译期一致性校验。
 * -------------------------------------------------------------------------- */

/** @brief 电机 PWM 引脚表（TIM1 六路，PA8/9/10 + PB13/14/15），组对象 hal_gpio_tim。 */
extern const hal_gpio_pin_desc_t hal_gpio_tim_pin_table[6];
/** @brief 电机 PWM 引脚组对象。 */
extern hal_gpio_group_t hal_gpio_tim;

/** @brief 运放引脚表（PA5/6/7、PA3/4、PB0、PB1），组对象 hal_gpio_opamp。 */
extern const hal_gpio_pin_desc_t hal_gpio_opamp_pin_table[7];
/** @brief 运放引脚组对象。 */
extern hal_gpio_group_t hal_gpio_opamp;

/** @brief ADC 常规通道引脚表（PA0 电位器、PA1 母线电压、PA2 NTC），组对象 hal_gpio_adc_reguler。 */
extern const hal_gpio_pin_desc_t hal_gpio_adc_reguler_pin_table[3];
/** @brief ADC 常规通道引脚组对象。 */
extern hal_gpio_group_t hal_gpio_adc_reguler;

/** @brief CAN 引脚表（PA12、PA11），组对象 hal_gpio_can。 */
extern const hal_gpio_pin_desc_t hal_gpio_can_pin_table[2];
/** @brief CAN 引脚组对象。 */
extern hal_gpio_group_t hal_gpio_can;

/** @brief 串口引脚表（PB10、PB11），组对象 hal_gpio_usart。 */
extern const hal_gpio_pin_desc_t hal_gpio_usart_pin_table[2];
/** @brief 串口引脚组对象。 */
extern hal_gpio_group_t hal_gpio_usart;

/**
 * @brief 硬件 I2C 引脚表（PB8、PB9），组对象 hal_gpio_i2c_hardware。
 * @note PB8/PB9 为 I2C1 重映射脚：构建本组前需先经
 *       plat_ch32v203_gpio_ops->pf_remap_enable(PLAT_GPIO_REMAP_AFIO_RCC,
 *       PLAT_GPIO_REMAP_I2C1) 使能重映射。
 */
extern const hal_gpio_pin_desc_t hal_gpio_i2c_hardware_pin_table[2];
/** @brief 硬件 I2C 引脚组对象。 */
extern hal_gpio_group_t hal_gpio_i2c_hardware;

/** @brief 硬件 SPI/SSI 引脚表（PB3、PB4、PB5、PA15），组对象 hal_gpio_spi_hardware。 */
extern const hal_gpio_pin_desc_t hal_gpio_spi_hardware_pin_table[4];
/** @brief 硬件 SPI 引脚组对象。 */
extern hal_gpio_group_t hal_gpio_spi_hardware;

/** @brief 指示灯引脚表（PC14、PC15），组对象 hal_gpio_led。 */
extern const hal_gpio_pin_desc_t hal_gpio_led_pin_table[2];
/** @brief 指示灯引脚组对象。 */
extern hal_gpio_group_t hal_gpio_led;

/** @brief IO 测试引脚表（PB6 验证 adc 触发位置、PB7 闲置备用），组对象 hal_gpio_io_test。 */
extern const hal_gpio_pin_desc_t hal_gpio_io_test_pin_table[2];
/** @brief IO 测试引脚组对象。 */
extern hal_gpio_group_t hal_gpio_io_test;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_GPIO_H__ */
