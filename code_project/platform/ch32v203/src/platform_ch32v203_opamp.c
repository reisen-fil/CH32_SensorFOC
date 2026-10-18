/**
 * @file platform_ch32v203_opamp.c
 * @brief CH32V203 内部运放平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库 OPA 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_opamp_ops 函数表对外绑定；hal_opamp.c 通过该表访问运放。
 * @note 本文件对标准库接口一律直接调用，不经宏包装。
 */
#include "platform_ch32v203_opamp.h"
#include <stddef.h>

/**
 * @brief 判断运放编号是否受支持。
 * @param num 运放编号，取 PLAT_OPAMP_NUM_x。
 * @retval true 受支持。
 * @retval false 编号非法。
 */
static bool plat_ch32v203_opamp_is_valid(uint8_t num)
{
    return (num == (uint8_t)PLAT_OPAMP_NUM_1) ||
           (num == (uint8_t)PLAT_OPAMP_NUM_2) ||
           (num == (uint8_t)PLAT_OPAMP_NUM_3) ||
           (num == (uint8_t)PLAT_OPAMP_NUM_4);
}

/**
 * @brief 复位所有运放（OPA_DeInit）。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_opamp_deinit(void)
{
    OPA_DeInit();
    return true;
}

/**
 * @brief 按配置初始化运放（OPA_StructInit + OPA_Init）。
 * @param p_config 运放平台配置。
 * @retval true 操作成功。
 * @retval false 配置为空或运放编号非法。
 */
static bool plat_ch32v203_opamp_config(
    const platform_ch32v203_opamp_config_t *p_config)
{
    PLAT_OPAMP_INIT_TYPE init_config;

    if ((p_config == NULL) ||
        (!plat_ch32v203_opamp_is_valid(p_config->num))) {
        return false;
    }

    OPA_StructInit(&init_config);
    init_config.OPA_NUM = (PLAT_OPAMP_NUM_TYPE)p_config->num;
    init_config.PSEL = (PLAT_OPAMP_PSEL_TYPE)p_config->psel;
    init_config.NSEL = (PLAT_OPAMP_NSEL_TYPE)p_config->nsel;
    init_config.Mode = (PLAT_OPAMP_MODE_TYPE)p_config->mode;
    OPA_Init(&init_config);

    return true;
}

/**
 * @brief 使能/禁用指定运放（OPA_Cmd）。
 * @param num 运放编号，取 PLAT_OPAMP_NUM_x。
 * @param enable true 使能，false 禁用。
 * @retval true 操作成功。
 * @retval false 运放编号非法。
 */
static bool plat_ch32v203_opamp_cmd(uint8_t num, bool enable)
{
    if (!plat_ch32v203_opamp_is_valid(num)) {
        return false;
    }

    OPA_Cmd((PLAT_OPAMP_NUM_TYPE)num, enable ? PLAT_ENABLE : PLAT_DISABLE);
    return true;
}

/** @brief CH32V203 运放操作函数表（本文件私有）。 */
static const platform_ch32v203_opamp_ops_t s_opamp_ops = {
    .pf_deinit = plat_ch32v203_opamp_deinit,
    .pf_config = plat_ch32v203_opamp_config,
    .pf_cmd = plat_ch32v203_opamp_cmd
};

/** @brief CH32V203 运放操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_opamp_ops_t *const plat_ch32v203_opamp_ops =
    &s_opamp_ops;
