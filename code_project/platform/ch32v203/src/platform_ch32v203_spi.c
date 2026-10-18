/**
 * @file platform_ch32v203_spi.c
 * @brief CH32V203 SPI 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库中的 SPI 抽象接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_spi_ops 函数表对外绑定：
 *          - 配置类封装：时钟使能 / 配置 / 使能 / 禁用 / 复位（带实例校验）；
 *          - 收发类封装：pf_send_data / pf_receive_data / pf_get_flag，
 *            与标准库接口一一对应，是本层对外唯一外露的收发原语。
 * @note 本文件对标准库接口一律直接调用，不经宏包装；仅时钟使能沿用平台时钟宏
 *       PLAT_RCC_APBx_CLOCK_ENABLE。收发细节（帧宽步进、标志轮询、超时、
 *       片选夹持）不在此实现，已封装于 hal_spi.c。
 *       本层不持有 HAL SPI 对象，也不提供板级初始化函数（已上移到 hal_spi.c）。
 */
#include "platform_ch32v203_spi.h"
#include <stddef.h>

/**
 * @brief 判断实例是否为受支持的 SPI 外设。
 * @param instance 实例基地址，取 PLAT_PERIPH_SPI1 / PLAT_PERIPH_SPI2。
 * @retval true 受支持。
 * @retval false 其它外设。
 */
static bool plat_ch32v203_spi_is_valid(uintptr_t instance)
{
    PLAT_SPI_INSTANCE_TYPE *p_spi = (PLAT_SPI_INSTANCE_TYPE *)(instance);

    return (p_spi == PLAT_PERIPH_SPI1) || (p_spi == PLAT_PERIPH_SPI2);
}

/**
 * @brief 使能 SPI 实例所在总线的外设时钟（SPI1 在 APB2，SPI2 在 APB1）。
 * @note 时钟使能是本文件唯一通过平台宏完成的动作。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_spi_clock_enable(uintptr_t instance)
{
    if (!plat_ch32v203_spi_is_valid(instance)) {
        return false;
    }

    if ((PLAT_SPI_INSTANCE_TYPE *)(instance) == PLAT_PERIPH_SPI1) {
        PLAT_RCC_APB2_CLOCK_ENABLE(PLAT_RCC_APB2_SPI1);
    } else {
        PLAT_RCC_APB1_CLOCK_ENABLE(PLAT_RCC_APB1_SPI2);
    }

    return true;
}

/**
 * @brief 复位并配置 SPI（方向固定全双工）。
 * @param instance 实例基地址。
 * @param p_config 平台 SPI 配置。
 * @retval true 操作成功。
 * @retval false 实例不受支持、配置为空或帧宽非法。
 */
static bool plat_ch32v203_spi_config(
    uintptr_t instance,
    const platform_ch32v203_spi_config_t *p_config)
{
    PLAT_SPI_INSTANCE_TYPE *p_spi;
    PLAT_SPI_INIT_TYPE init_config;

    if ((p_config == NULL) ||
        ((p_config->data_size != PLAT_SPI_DATA_SIZE_8BIT) &&
         (p_config->data_size != PLAT_SPI_DATA_SIZE_16BIT)) ||
        (!plat_ch32v203_spi_is_valid(instance))) {
        return false;
    }

    p_spi = (PLAT_SPI_INSTANCE_TYPE *)(instance);

    init_config.SPI_Direction = PLAT_SPI_DIRECTION_2_LINES_FULL_DUPLEX;
    init_config.SPI_Mode = p_config->mode;
    init_config.SPI_DataSize = p_config->data_size;
    init_config.SPI_CPOL = p_config->clock_polarity;
    init_config.SPI_CPHA = p_config->clock_phase;
    init_config.SPI_NSS = p_config->nss;
    init_config.SPI_BaudRatePrescaler = p_config->baud_rate_prescaler;
    init_config.SPI_FirstBit = p_config->is_msb_first ?
                               PLAT_SPI_FIRST_BIT_MSB :
                               PLAT_SPI_FIRST_BIT_LSB;
    init_config.SPI_CRCPolynomial = 7U;

    SPI_I2S_DeInit(p_spi);
    SPI_Init(p_spi, &init_config);

    return true;
}

/**
 * @brief 使能 SPI 收发。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_spi_enable(uintptr_t instance)
{
    if (!plat_ch32v203_spi_is_valid(instance)) {
        return false;
    }

    SPI_Cmd((PLAT_SPI_INSTANCE_TYPE *)(instance), PLAT_ENABLE);
    return true;
}

/**
 * @brief 禁用 SPI 收发。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_spi_disable(uintptr_t instance)
{
    if (!plat_ch32v203_spi_is_valid(instance)) {
        return false;
    }

    SPI_Cmd((PLAT_SPI_INSTANCE_TYPE *)(instance), PLAT_DISABLE);
    return true;
}

/**
 * @brief 禁用并把 SPI 寄存器恢复为复位状态。
 * @param instance 实例基地址。
 * @retval true 操作成功。
 * @retval false 实例不受支持。
 */
static bool plat_ch32v203_spi_deinit(uintptr_t instance)
{
    PLAT_SPI_INSTANCE_TYPE *p_spi;

    if (!plat_ch32v203_spi_is_valid(instance)) {
        return false;
    }

    p_spi = (PLAT_SPI_INSTANCE_TYPE *)(instance);
    SPI_Cmd(p_spi, PLAT_DISABLE);
    SPI_I2S_DeInit(p_spi);
    return true;
}

/**
 * @brief 发送一个帧（SPI_I2S_SendData 的简单函数封装）。
 * @param instance 实例基地址。
 * @param data 待发送的帧数据。
 */
static void plat_ch32v203_spi_send_data(uintptr_t instance, uint16_t data)
{
    SPI_I2S_SendData((PLAT_SPI_INSTANCE_TYPE *)(instance), data);
}

/**
 * @brief 接收一个帧（SPI_I2S_ReceiveData 的简单函数封装）。
 * @param instance 实例基地址。
 * @return 接收到的帧数据。
 */
static uint16_t plat_ch32v203_spi_receive_data(uintptr_t instance)
{
    return (uint16_t)SPI_I2S_ReceiveData((PLAT_SPI_INSTANCE_TYPE *)(instance));
}

/**
 * @brief 查询状态标志（SPI_I2S_GetFlagStatus 的简单函数封装）。
 * @param instance 实例基地址。
 * @param flag 待查询标志，取 PLAT_SPI_FLAG_x。
 * @return PLAT_SPI_STATUS_SET 表示置位，PLAT_SPI_STATUS_RESET 表示复位。
 */
static uint16_t plat_ch32v203_spi_get_flag(uintptr_t instance, uint16_t flag)
{
    return (uint16_t)SPI_I2S_GetFlagStatus((PLAT_SPI_INSTANCE_TYPE *)(instance), flag);
}

/** @brief CH32V203 SPI 操作函数表（本文件私有）。 */
static const platform_ch32v203_spi_ops_t s_spi_ops = {
    .pf_clock_enable = plat_ch32v203_spi_clock_enable,
    .pf_config = plat_ch32v203_spi_config,
    .pf_enable = plat_ch32v203_spi_enable,
    .pf_disable = plat_ch32v203_spi_disable,
    .pf_deinit = plat_ch32v203_spi_deinit,
    .pf_send_data = plat_ch32v203_spi_send_data,
    .pf_receive_data = plat_ch32v203_spi_receive_data,
    .pf_get_flag = plat_ch32v203_spi_get_flag
};

/** @brief CH32V203 SPI 操作函数表指针（对外绑定的 MCU 抽象接口）。 */
const platform_ch32v203_spi_ops_t *const plat_ch32v203_spi_ops = &s_spi_ops;
