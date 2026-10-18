/**
 * @file platform_ch32v203_opamp.h
 * @brief CH32V203 内部运放（OPA）的 MCU 抽象接口（取值宏 + 操作函数表）。
 * @details 与 GPIO/SPI/ADC 平台层同构：
 *          - 取值宏：运放编号、正/负输入选择、输出模式；
 *          - 操作函数表 platform_ch32v203_opamp_ops_t：把标准库 OPA 接口
 *            （OPA_DeInit / OPA_StructInit / OPA_Init / OPA_Cmd）逐个做简单函数
 *            封装后绑定，并通过 plat_ch32v203_opamp_ops 指针对外外露。
 *          运放的输入/输出引脚属 GPIO 配置，由 hal_gpio 的运放引脚组负责；
 *          本层只管运放本体（参考实现 drv_ADC_OPAMP_init() 的 OPA 部分）。
 * @note 运放为全局外设（无实例基地址），故配置类接口以编号/配置结构为参数。
 */
#ifndef __PLATFORM_CH32V203_OPAMP_H__
#define __PLATFORM_CH32V203_OPAMP_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/** @brief 运放编号。 */
#define PLAT_OPAMP_NUM_1                      OPA1
#define PLAT_OPAMP_NUM_2                      OPA2
#define PLAT_OPAMP_NUM_3                      OPA3
#define PLAT_OPAMP_NUM_4                      OPA4

/** @brief 运放正输入选择。 */
#define PLAT_OPAMP_PSEL_CHP0                  CHP0
#define PLAT_OPAMP_PSEL_CHP1                  CHP1

/** @brief 运放负输入选择。 */
#define PLAT_OPAMP_NSEL_CHN0                  CHN0
#define PLAT_OPAMP_NSEL_CHN1                  CHN1

/** @brief 运放输出模式。 */
#define PLAT_OPAMP_OUT_IO_OUT0                OUT_IO_OUT0
#define PLAT_OPAMP_OUT_IO_OUT1                OUT_IO_OUT1

/* --------------------------------------------------------------------------
 * 类型接入宏：把平台层用到的厂商类型名收敛到本头的宏体里，
 * platform_ch32v203_opamp.c 内只出现 PLAT_* 名字，不再直接书写厂商类型。
 * 宏体延迟展开，本头仍无任何厂商类型（可经 hal.h 在 ch32v20x_conf.h 之前解析）。
 * -------------------------------------------------------------------------- */
#define PLAT_OPAMP_NUM_TYPE                   OPA_Num_TypeDef
#define PLAT_OPAMP_MODE_TYPE                  OPA_Mode_TypeDef
#define PLAT_OPAMP_PSEL_TYPE                  OPA_PSEL_TypeDef
#define PLAT_OPAMP_NSEL_TYPE                  OPA_NSEL_TypeDef
#define PLAT_OPAMP_INIT_TYPE                  OPA_InitTypeDef

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 运放平台配置（对应标准库 OPA_InitTypeDef）。 */
typedef struct {
    uint8_t num;    /**< PLAT_OPAMP_NUM_x */
    uint8_t psel;   /**< PLAT_OPAMP_PSEL_x */
    uint8_t nsel;   /**< PLAT_OPAMP_NSEL_x */
    uint8_t mode;   /**< PLAT_OPAMP_OUT_x */
} platform_ch32v203_opamp_config_t;

/**
 * @brief CH32V203 运放操作函数表（MCU 抽象接口的简单函数封装）。
 * @details 返回 bool，true 表示成功。
 */
typedef struct {
    /** @brief 复位所有运放（OPA_DeInit）。 */
    bool (*pf_deinit)(void);
    /** @brief 按配置初始化运放（OPA_StructInit + OPA_Init）。 */
    bool (*pf_config)(const platform_ch32v203_opamp_config_t *p_config);
    /** @brief 使能/禁用指定运放（OPA_Cmd）。 */
    bool (*pf_cmd)(uint8_t num, bool enable);
} platform_ch32v203_opamp_ops_t;

/** @brief CH32V203 运放操作函数表指针（在 platform_ch32v203_opamp.c 中组装）。 */
extern const platform_ch32v203_opamp_ops_t *const plat_ch32v203_opamp_ops;

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_OPAMP_H__ */
