/**
 * @file hal_opamp.c
 * @brief 内部运放对象操作 + 板级实例（OPA1/OPA2）+ HAL 层函数表。
 * @details 硬件访问统一经平台运放操作函数表指针 plat_ch32v203_opamp_ops 完成，
 *          引脚组构建经 HAL GPIO 函数表指针 hal_gpio_ops 完成，本文件不含厂商库符号。
 *          板级配置依据参考实现 drv_ADC_OPAMP_init()：OPA1/OPA2 均取
 *          正输入 CHP1、负输入 CHN1、输出 OUT_IO_OUT1。
 *          与 hal_tim.c 同构的板级选择方式：两张板级配置表
 *          （s_opamp_opa1_config / s_opamp_opa2_config）由
 *          hal_opamp_build(p_object, hal_opamp_role_x) 按用途分发到
 *          hal_opamp_build_opa1() / hal_opamp_build_opa2()。
 */
#include "hal_opamp.h"
#include "hal_gpio.h"
#include <stddef.h>

/** @brief 板级运放实例：OPA1。 */
hal_opamp_t hal_opamp_opa1 = {0};
/** @brief 板级运放实例：OPA2。 */
hal_opamp_t hal_opamp_opa2 = {0};

/** @brief 板级运放配置表：OPA1（电流采样内部运放 1）——CHP1 / CHN1 / OUT_IO_OUT1。 */
static const hal_opamp_config_t s_opamp_opa1_config = {
    .num = PLAT_OPAMP_NUM_1,
    .psel = PLAT_OPAMP_PSEL_CHP1,
    .nsel = PLAT_OPAMP_NSEL_CHN1,
    .mode = PLAT_OPAMP_OUT_IO_OUT1
};

/** @brief 板级运放配置表：OPA2（电流采样内部运放 2）——CHP1 / CHN1 / OUT_IO_OUT1。 */
static const hal_opamp_config_t s_opamp_opa2_config = {
    .num = PLAT_OPAMP_NUM_2,
    .psel = PLAT_OPAMP_PSEL_CHP1,
    .nsel = PLAT_OPAMP_NSEL_CHN1,
    .mode = PLAT_OPAMP_OUT_IO_OUT1
};

/**
 * @brief 检查运放对象是否有效。
 * @param p_object 模块对象指针。
 * @retval true 对象指针有效。
 * @retval false 对象为空。
 */
static bool hal_opamp_is_object_valid(const hal_opamp_t *p_object)
{
    return (p_object != NULL);
}

/**
 * @brief 绑定运放对象：保存运放配置。
 * @param p_object 模块对象指针。
 * @param p_config 模块初始化配置指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 参数无效或必需指针为空。
 */
hal_opamp_status_enum hal_opamp_bind(hal_opamp_t *p_object,
                                     const hal_opamp_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL)) {
        return hal_opamp_err_param;
    }

    p_object->state = hal_opamp_state_reset;
    p_object->config = *p_config;

    return hal_opamp_ok;
}

/**
 * @brief 初始化运放：按配置写入并使能。
 * @param p_object 模块对象指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 对象为空或平台层拒绝（编号非法）。
 */
hal_opamp_status_enum hal_opamp_init(hal_opamp_t *p_object)
{
    platform_ch32v203_opamp_config_t platform_config;

    if (!hal_opamp_is_object_valid(p_object)) {
        return hal_opamp_err_param;
    }

    platform_config.num = p_object->config.num;
    platform_config.psel = p_object->config.psel;
    platform_config.nsel = p_object->config.nsel;
    platform_config.mode = p_object->config.mode;

    if (!plat_ch32v203_opamp_ops->pf_config(&platform_config)) {
        p_object->state = hal_opamp_state_error;
        return hal_opamp_err_param;
    }

    if (!plat_ch32v203_opamp_ops->pf_cmd(p_object->config.num, true)) {
        p_object->state = hal_opamp_state_error;
        return hal_opamp_err_param;
    }

    p_object->state = hal_opamp_state_enabled;
    return hal_opamp_ok;
}

/**
 * @brief 使能运放。
 * @param p_object 模块对象指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 对象为空或编号非法。
 */
hal_opamp_status_enum hal_opamp_enable(hal_opamp_t *p_object)
{
    if (!hal_opamp_is_object_valid(p_object)) {
        return hal_opamp_err_param;
    }

    if (!plat_ch32v203_opamp_ops->pf_cmd(p_object->config.num, true)) {
        return hal_opamp_err_param;
    }

    p_object->state = hal_opamp_state_enabled;
    return hal_opamp_ok;
}

/**
 * @brief 禁用运放。
 * @param p_object 模块对象指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 对象为空或编号非法。
 */
hal_opamp_status_enum hal_opamp_disable(hal_opamp_t *p_object)
{
    if (!hal_opamp_is_object_valid(p_object)) {
        return hal_opamp_err_param;
    }

    if (!plat_ch32v203_opamp_ops->pf_cmd(p_object->config.num, false)) {
        return hal_opamp_err_param;
    }

    p_object->state = hal_opamp_state_disabled;
    return hal_opamp_ok;
}

/**
 * @brief 返回运放对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_opamp_state_enum hal_opamp_get_state(const hal_opamp_t *p_object)
{
    if (p_object == NULL) {
        return hal_opamp_state_error;
    }
    return p_object->state;
}

/**
 * @brief 构建 OPA1 板级实例（共用运放引脚组 + s_opamp_opa1_config + 使能）。
 * @param p_object 运放对象指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 参数无效。
 * @retval hal_opamp_err_state 引脚组构建或平台层配置失败。
 */
static hal_opamp_status_enum hal_opamp_build_opa1(hal_opamp_t *p_object)
{
    hal_opamp_status_enum status;

    /* 板级前置：运放输入/输出引脚组（两只运放共用，经 HAL GPIO 函数表构建，可重复调用） */
    if (hal_gpio_ops->pf_build(&hal_gpio_opamp,
                               hal_gpio_opamp_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_opamp_pin_table))
        != hal_gpio_ok) {
        return hal_opamp_err_state;
    }

    status = hal_opamp_bind(p_object, &s_opamp_opa1_config);
    if (status != hal_opamp_ok) {
        return status;
    }

    return hal_opamp_init(p_object);
}

/**
 * @brief 构建 OPA2 板级实例（共用运放引脚组 + s_opamp_opa2_config + 使能）。
 * @param p_object 运放对象指针。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 参数无效。
 * @retval hal_opamp_err_state 引脚组构建或平台层配置失败。
 */
static hal_opamp_status_enum hal_opamp_build_opa2(hal_opamp_t *p_object)
{
    hal_opamp_status_enum status;

    /* 板级前置：运放输入/输出引脚组（两只运放共用，经 HAL GPIO 函数表构建，可重复调用） */
    if (hal_gpio_ops->pf_build(&hal_gpio_opamp,
                               hal_gpio_opamp_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_opamp_pin_table))
        != hal_gpio_ok) {
        return hal_opamp_err_state;
    }

    status = hal_opamp_bind(p_object, &s_opamp_opa2_config);
    if (status != hal_opamp_ok) {
        return status;
    }

    return hal_opamp_init(p_object);
}

/**
 * @brief 构建板级运放实例（按 role 选择板级配置表 + 引脚组 + 使能）。
 * @param p_object 运放对象指针。
 * @param role 板级用途，取 hal_opamp_role_x。
 * @retval hal_opamp_ok 操作成功。
 * @retval hal_opamp_err_param 参数无效或用途不支持。
 * @retval hal_opamp_err_state 引脚组构建或平台层配置失败。
 */
hal_opamp_status_enum hal_opamp_build(hal_opamp_t *p_object,
                                      hal_opamp_role_enum role)
{
    if (p_object == NULL) {
        return hal_opamp_err_param;
    }

    if (role == hal_opamp_role_opa1) {
        return hal_opamp_build_opa1(p_object);
    }
    if (role == hal_opamp_role_opa2) {
        return hal_opamp_build_opa2(p_object);
    }

    return hal_opamp_err_param;
}

/* ==========================================================================
 * HAL 层运放函数表
 * ========================================================================== */

/** @brief HAL 层运放函数表（本文件私有，绑定本层对外接口）。 */
static const hal_opamp_ops_t s_hal_opamp_ops = {
    .pf_build = hal_opamp_build,
    /* 备用接口：与同名 HAL 函数一一对应，供外部按需接入 */
    .pf_enable = hal_opamp_enable,
    .pf_disable = hal_opamp_disable,
    .pf_get_state = hal_opamp_get_state
};

/** @brief HAL 层运放函数表指针（对外外露，声明于 hal_opamp.h）。 */
const hal_opamp_ops_t *const hal_opamp_ops = &s_hal_opamp_ops;
