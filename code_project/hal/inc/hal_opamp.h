/**
 * @file hal_opamp.h
 * @brief 内部运放（OPA）硬件抽象接口（含板级实例与 HAL 层函数表）。
 * @details 与 GPIO/SPI/ADC 同构：对象操作函数同层实现，硬件访问统一经平台运放
 *          操作函数表指针 plat_ch32v203_opamp_ops 完成，本层不含厂商库符号。
 *          板级实例为电流采样用的两只内部运放（对应参考实现 drv_ADC_OPAMP_init()）：
 *          OPA1、OPA2 均取 正输入 CHP1 / 负输入 CHN1 / 输出 OUT_IO_OUT1；
 *          输入脚 PA5/PA6/PA7/PB0 浮空、输出脚 PA3/PB1 与 PB1 模拟输入由
 *          hal_gpio 的运放引脚组（hal_gpio_opamp_pin_table）配置，构建运放时
 *          会自动经 HAL GPIO 函数表构建该引脚组。
 *          板级按用途选择配置：hal_opamp_build(p_object, hal_opamp_role_x) 经
 *          s_opamp_opa1_config / s_opamp_opa2_config 两张板级配置表分发。
 */
#ifndef __HAL_OPAMP_H__
#define __HAL_OPAMP_H__

#include <stdint.h>
#include <stdbool.h>

/* MCU 抽象接口：PLAT_OPAMP_* 编号/输入选择/输出模式取值宏 */
#include "platform_ch32v203_opamp.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 运放对象前向声明。 */
typedef struct hal_opamp hal_opamp_t;

/** @brief 运放对象的生命周期状态。 */
typedef enum {
    hal_opamp_state_reset = 0,
    hal_opamp_state_ready,
    hal_opamp_state_enabled,
    hal_opamp_state_disabled,
    hal_opamp_state_error
} hal_opamp_state_enum;

/** @brief 运放接口调用返回的状态码。 */
typedef enum {
    hal_opamp_ok = 0,
    hal_opamp_err_param = -1,
    hal_opamp_err_state = -2,
    hal_opamp_err_unsupported = -3,
    hal_opamp_err_timeout = -4
} hal_opamp_status_enum;

/** @brief 运放板级用途（与 hal_opamp.c 中两张板级配置表一一对应）。 */
typedef enum {
    hal_opamp_role_opa1 = 0, /**< OPA1：正输入 CHP1 / 负输入 CHN1 / 输出 OUT_IO_OUT1（电流采样 1） */
    hal_opamp_role_opa2      /**< OPA2：正输入 CHP1 / 负输入 CHN1 / 输出 OUT_IO_OUT1（电流采样 2） */
} hal_opamp_role_enum;

/** @brief 运放编号、正/负输入选择与输出模式配置。 */
typedef struct {
    uint8_t num;    /**< PLAT_OPAMP_NUM_x */
    uint8_t psel;   /**< PLAT_OPAMP_PSEL_x */
    uint8_t nsel;   /**< PLAT_OPAMP_NSEL_x */
    uint8_t mode;   /**< PLAT_OPAMP_OUT_x */
} hal_opamp_config_t;

/** @brief 运放对象定义（状态与配置）。 */
struct hal_opamp {
    hal_opamp_state_enum state;
    hal_opamp_config_t config;
};

/**
 * @brief 绑定运放对象：保存运放配置。
 * @param p_object 模块对象指针。
 * @param p_config 模块初始化配置指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 参数无效或必需指针为空。
 */
hal_opamp_status_enum hal_opamp_bind(hal_opamp_t *p_object,
                                     const hal_opamp_config_t *p_config);

/** @brief 初始化运放：按配置写入并使能。 */
hal_opamp_status_enum hal_opamp_init(hal_opamp_t *p_object);

/** @brief 使能运放。 */
hal_opamp_status_enum hal_opamp_enable(hal_opamp_t *p_object);

/** @brief 禁用运放。 */
hal_opamp_status_enum hal_opamp_disable(hal_opamp_t *p_object);

/** @brief 查询运放对象当前状态。 */
hal_opamp_state_enum hal_opamp_get_state(const hal_opamp_t *p_object);

/**
 * @brief 构建板级运放实例（引脚组 + 板级配置表 + 使能），按 role 选择配置。
 * @details 与 hal_tim_build 同构：按 hal_opamp_role_x 选择对应的板级配置表
 *          （s_opamp_opa1_config / s_opamp_opa2_config，两者均为 PSEL=CHP1、
 *          NSEL=CHN1、OUT_IO_OUT1，仅 num 分别为 PLAT_OPAMP_NUM_1 / NUM_2），
 *          先经 HAL GPIO 函数表构建运放引脚组（PA5/6/7、PB0 浮空，PA3/4、PB1 模拟；
 *          该引脚组由两只运放共用，可重复构建），再绑定配置并初始化使能。
 * @param p_object 运放对象指针。
 * @param role 板级用途，取 hal_opamp_role_x。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 参数无效或用途不支持。
 * @retval hal_opamp_err_state 引脚组构建或平台层配置失败。
 */
hal_opamp_status_enum hal_opamp_build(hal_opamp_t *p_object,
                                      hal_opamp_role_enum role);

/** @brief 板级 OPA1 实例（电流采样内部运放 1）。 */
extern hal_opamp_t hal_opamp_opa1;
/** @brief 板级 OPA2 实例（电流采样内部运放 2）。 */
extern hal_opamp_t hal_opamp_opa2;

/**
 * @brief HAL 层运放函数表：板级构建 / 使能 / 禁用 / 状态。
 * @details 表中的函数指针在 hal_opamp.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_opamp_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级运放实例（见 hal_opamp_build）。 */
    hal_opamp_status_enum (*pf_build)(hal_opamp_t *p_object,
                                      hal_opamp_role_enum role);
    /** @brief 使能运放（见 hal_opamp_enable）。 */
    hal_opamp_status_enum (*pf_enable)(hal_opamp_t *p_object);
    /** @brief 禁用运放（见 hal_opamp_disable）。 */
    hal_opamp_status_enum (*pf_disable)(hal_opamp_t *p_object);
    /** @brief 查询对象状态（见 hal_opamp_get_state）。 */
    hal_opamp_state_enum (*pf_get_state)(const hal_opamp_t *p_object);
} hal_opamp_ops_t;

/** @brief HAL 层运放函数表指针（在 hal_opamp.c 中定义并绑定）。 */
extern const hal_opamp_ops_t *const hal_opamp_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_OPAMP_H__ */
