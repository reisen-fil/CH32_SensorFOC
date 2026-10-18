/**
 * @file hal_spi.c
 * @brief SPI 对象操作 + CH32V203 SPI 适配绑定 + 板级编码器实例。
 * @details 与 hal_gpio.c 同构：对象操作函数同层实现，硬件访问统一经平台 SPI 操作
 *          函数表指针 plat_ch32v203_spi_ops（SPI 外设收发/配置原语）与
 *          plat_ch32v203_gpio_ops（重映射、片选）完成，本文件不含厂商库符号。
 *          SPI 的**收发细节**（帧宽步进、TXE/RXNE/BSY 标志轮询、超时、16 位帧
 *          字节序、软件片选夹持）在本层以 hal_spi_transfer() 封装，平台层只提供
 *          MCU 抽象级的收发原语。
 *          本层不引入组配置/多视图机制：一个 hal_spi_t 直接对应一个 SPI 实例，
 *          配置写入该实例的 hal_spi_config_t，操作直接作用于该实例。
 */
#include "hal_spi.h"
#include "hal_gpio.h"
#include <stddef.h>

/**
 * @brief 检查SPI对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（实例基地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_spi_is_object_valid(const hal_spi_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 软件片选写：经 GPIO 操作函数表驱动板级片选脚。
 * @param p_object SPI 对象指针。
 * @param level true 拉高（置闲），false 拉低（选中）；硬件片选时为空操作。
 */
static void hal_spi_cs_write(const hal_spi_t *p_object, bool level)
{
    if (p_object->config.is_hardware_cs) {
        return;
    }

    (void)plat_ch32v203_gpio_ops->pf_write((uintptr_t)PLAT_SPI_ENCODER_CS_PORT,
                                           (uint16_t)PLAT_SPI_ENCODER_CS_PIN,
                                           level);
}

/**
 * @brief SPI 全双工收发详细处理（本层封装）。
 * @details 只使用平台函数表外露的收发原语：按配置的帧宽步进，逐帧轮询
 *          TXE（可写）与 RXNE（已收），最后等待 BSY 释放；每步以
 *          config.timeout_count 计数超时。16 位帧按大端高低字节拆分/拼装。
 * @param p_object SPI 对象指针。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 数据长度（16 位帧宽时应为偶数）。
 * @retval true 传输完成。
 * @retval false 等待标志超时，传输未完成。
 */
static bool hal_spi_transfer(hal_spi_t *p_object,
                            const uint8_t *p_tx_buf,
                            uint8_t *p_rx_buf,
                            uint16_t data_length)
{
    uintptr_t instance = p_object->base_address;
    uint16_t data_step;
    uint16_t index;
    uint16_t tx_data;
    uint16_t rx_data;
    uint32_t timeout;

    data_step = (p_object->config.data_size == PLAT_SPI_DATA_SIZE_16BIT) ?
                2U : 1U;

    for (index = 0U; index < data_length; index += data_step) {
        tx_data = p_tx_buf[index];
        if (data_step == 2U) {
            tx_data = ((uint16_t)p_tx_buf[index] << 8U) |
                      (uint16_t)p_tx_buf[index + 1U];
        }

        timeout = p_object->config.timeout_count;
        while (plat_ch32v203_spi_ops->pf_get_flag(instance,
                                                  PLAT_SPI_FLAG_TXE) ==
               PLAT_SPI_STATUS_RESET) {
            if (timeout-- == 0U) {
                return false;
            }
        }

        plat_ch32v203_spi_ops->pf_send_data(instance, tx_data);

        timeout = p_object->config.timeout_count;
        while (plat_ch32v203_spi_ops->pf_get_flag(instance,
                                                  PLAT_SPI_FLAG_RXNE) ==
               PLAT_SPI_STATUS_RESET) {
            if (timeout-- == 0U) {
                return false;
            }
        }

        rx_data = plat_ch32v203_spi_ops->pf_receive_data(instance);
        p_rx_buf[index] = (uint8_t)rx_data;
        if (data_step == 2U) {
            p_rx_buf[index] = (uint8_t)(rx_data >> 8U);
            p_rx_buf[index + 1U] = (uint8_t)rx_data;
        }
    }

    timeout = p_object->config.timeout_count;
    while (plat_ch32v203_spi_ops->pf_get_flag(instance,
                                              PLAT_SPI_FLAG_BSY) ==
           PLAT_SPI_STATUS_SET) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 绑定SPI对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址。
 * @param p_config 模块初始化配置指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 参数无效或必需指针为空。
 */
hal_spi_status_enum hal_spi_bind(
    hal_spi_t *p_object,
    uintptr_t base_address,
    const hal_spi_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) || (base_address == 0U)) {
        return hal_spi_err_param;
    }

    p_object->state = hal_spi_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;

    return hal_spi_ok;
}

/**
 * @brief 初始化SPI外设：重映射 + 时钟使能 + 寄存器配置 + 使能收发 + 片选置闲。
 * @details 重映射与片选属 GPIO/AFIO 级动作，经 GPIO 操作函数表
 *          （plat_ch32v203_gpio_ops）完成；SPI 外设本身经 SPI 操作函数表
 *          （plat_ch32v203_spi_ops）完成。
 * @param p_object 模块对象指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 对象未绑定、实例不受支持或配置非法。
 */
hal_spi_status_enum hal_spi_init(hal_spi_t *p_object)
{
    platform_ch32v203_spi_config_t platform_config;

    if (!hal_spi_is_object_valid(p_object)) {
        return hal_spi_err_param;
    }

    platform_config.data_size = p_object->config.data_size;
    platform_config.mode = p_object->config.mode;
    platform_config.clock_polarity = p_object->config.clock_polarity;
    platform_config.clock_phase = p_object->config.clock_phase;
    platform_config.nss = p_object->config.nss;
    platform_config.baud_rate_prescaler = p_object->config.baud_rate_prescaler;
    platform_config.is_msb_first = p_object->config.is_msb_first;

    if ((!plat_ch32v203_spi_ops->pf_clock_enable(p_object->base_address)) ||
        (!plat_ch32v203_spi_ops->pf_config(p_object->base_address,
                                           &platform_config)) ||
        (!plat_ch32v203_spi_ops->pf_enable(p_object->base_address))) {
        p_object->state = hal_spi_state_error;
        return hal_spi_err_param;
    }

    p_object->state = hal_spi_state_ready;
    return hal_spi_ok;
}

/**
 * @brief 关闭并释放SPI外设。
 * @param p_object 模块对象指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 对象未绑定或实例不受支持。
 */
hal_spi_status_enum hal_spi_deinit(hal_spi_t *p_object)
{
    if (!hal_spi_is_object_valid(p_object)) {
        return hal_spi_err_param;
    }

    if (!plat_ch32v203_spi_ops->pf_deinit(p_object->base_address)) {
        return hal_spi_err_param;
    }

    p_object->state = hal_spi_state_reset;
    return hal_spi_ok;
}

/**
 * @brief 使能SPI收发。
 * @param p_object 模块对象指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 对象未绑定或实例不受支持。
 */
hal_spi_status_enum hal_spi_enable(hal_spi_t *p_object)
{
    if (!hal_spi_is_object_valid(p_object)) {
        return hal_spi_err_param;
    }

    if (!plat_ch32v203_spi_ops->pf_enable(p_object->base_address)) {
        return hal_spi_err_param;
    }

    p_object->state = hal_spi_state_enabled;
    return hal_spi_ok;
}

/**
 * @brief 禁用SPI收发。
 * @param p_object 模块对象指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 对象未绑定或实例不受支持。
 */
hal_spi_status_enum hal_spi_disable(hal_spi_t *p_object)
{
    if (!hal_spi_is_object_valid(p_object)) {
        return hal_spi_err_param;
    }

    if (!plat_ch32v203_spi_ops->pf_disable(p_object->base_address)) {
        return hal_spi_err_param;
    }

    p_object->state = hal_spi_state_disabled;
    return hal_spi_ok;
}

/**
 * @brief 通过SPI同时发送并接收指定长度的数据。
 * @details 收发细节由 hal_spi_transfer() 封装；软件片选时在事务前后夹持片选。
 * @param p_object 模块对象指针。
 * @param p_tx_buf 待发送数据缓冲区。
 * @param p_rx_buf 接收数据缓冲区。
 * @param data_length 待读写的数据长度。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 参数无效或必需指针为空。
 * @retval hal_spi_err_timeout 等待标志超时（传输未完成）。
 */
hal_spi_status_enum hal_spi_receive(hal_spi_t *p_object,
                                    const uint8_t *p_tx_buf,
                                    uint8_t *p_rx_buf,
                                    uint16_t data_length)
{
    bool is_transferred;

    if ((!hal_spi_is_object_valid(p_object)) ||
        (p_tx_buf == NULL) ||
        (p_rx_buf == NULL) ||
        (data_length == 0U)) {
        return hal_spi_err_param;
    }
    if ((p_object->config.data_size == PLAT_SPI_DATA_SIZE_16BIT) &&
        ((data_length & 1U) != 0U)) {
        return hal_spi_err_param;
    }

    hal_spi_cs_write(p_object, PLAT_GPIO_LEVEL_RESET);
    is_transferred = hal_spi_transfer(p_object, p_tx_buf, p_rx_buf,
                                      data_length);
    hal_spi_cs_write(p_object, PLAT_GPIO_LEVEL_SET);

    return is_transferred ? hal_spi_ok : hal_spi_err_timeout;
}

/**
 * @brief 返回SPI对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_spi_state_enum hal_spi_get_state(const hal_spi_t *p_object)
{
    if (p_object == NULL) {
        return hal_spi_state_error;
    }
    return p_object->state;
}

/* ==========================================================================
 * 板级实例：MT6701 编码器 SPI1
 * ========================================================================== */

/**
 * @brief 编码器 SPI1 配置：16 位帧、主模式、CPOL 低、第二边沿、软件片选、
 *        /16 分频、MSB 先行（与 Drv_SPI_Init 一致）。
 */
static const hal_spi_config_t s_encoder_spi_config = {
    .max_frequency_hz = 9000000U,
    .data_size = PLAT_SPI_DATA_SIZE_16BIT,
    .mode = PLAT_SPI_MODE_MASTER,
    .clock_polarity = PLAT_SPI_CPOL_LOW,
    .clock_phase = PLAT_SPI_CPHA_SECOND_EDGE,
    .nss = PLAT_SPI_NSS_SOFTWARE,
    .baud_rate_prescaler = PLAT_SPI_BAUD_RATE_PRESCALER_16,
    .is_msb_first = true,
    .is_hardware_cs = false,
    .timeout_count = 100000U,
    .p_user_data = NULL
};

hal_spi_t hal_spi_encoder = {0};

/**
 * @brief 构建板级 SPI 实例：绑定板级配置并使能。
 * @details SPI1 的引脚重映射与片选置闲已在 hal_spi_init() 内经 GPIO 操作函数表完成。
 * @param p_driver SPI 对象指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 参数无效、实例不受支持或配置非法。
 */
hal_spi_status_enum hal_spi_build(hal_spi_t *p_driver)
{
    hal_spi_status_enum status;

    if (p_driver == NULL) {
        return hal_spi_err_param;
    }

        /* 板级前置：PA12（CAN 发送）/PA11（CAN 接收）引脚组，默认映射无需重映射 */
    if (hal_gpio_ops->pf_build(&hal_gpio_spi_hardware,
                               hal_gpio_spi_hardware_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_spi_hardware_pin_table))
        != hal_gpio_ok) {
        return hal_spi_err_state;
    }

    status = hal_spi_bind(p_driver,
                          (uintptr_t)PLAT_PERIPH_SPI1,
                          &s_encoder_spi_config);
    if (status != hal_spi_ok) {
        return status;
    }

    /* 板级前置①：SPI1 使用的 PB3/PB4/PB5 为复用重映射引脚 */
    if (p_driver->base_address == (uintptr_t)PLAT_PERIPH_SPI1) {
        plat_ch32v203_gpio_ops->pf_remap_enable(PLAT_GPIO_REMAP_AFIO_RCC,
                                                PLAT_GPIO_REMAP_SPI1);
    }

    status = hal_spi_init(p_driver);
    if (status != hal_spi_ok) {
        return status;
    }

    /* 板级前置②：软件片选置闲（拉高） */
    hal_spi_cs_write(p_driver, PLAT_GPIO_LEVEL_SET);

    return hal_spi_ok;
}

/* ==========================================================================
 * HAL 层 SPI 函数表
 * ========================================================================== */

/** @brief HAL 层 SPI 函数表（本文件私有，表中绑定本层对外的接口）。 */
static const hal_spi_ops_t s_hal_spi_ops = {
    .pf_build = hal_spi_build,
    .pf_receive = hal_spi_receive,
    /* 备用接口：与同名 HAL 函数一一对应，供外部按需接入 */
    .pf_get_state = hal_spi_get_state,    
    .pf_deinit = hal_spi_deinit,
    .pf_enable = hal_spi_enable,
    .pf_disable = hal_spi_disable
};

/** @brief HAL 层 SPI 函数表指针（对外外露，声明于 hal_spi.h）。 */
const hal_spi_ops_t *const hal_spi_ops = &s_hal_spi_ops;


