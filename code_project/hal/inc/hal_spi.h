/**
 * @file hal_spi.h
 * @brief SPI 接口（含 CH32V203 SPI 适配绑定与板级编码器实例）。
 * @details 与 GPIO 同构：对象操作函数同层实现，硬件访问统一经平台 SPI 操作函数表
 *          （plat_ch32v203_spi_ops）完成，本层不含厂商库符号。
 *          本层对外同时提供函数式接口与 HAL 层函数表（hal_spi_ops）。
 *          本层不引入组配置/多视图机制：一个 hal_spi_t 直接对应一个 SPI 外设实例，
 *          配置即"写入该实例的 hal_spi_config_t"。
 */
#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* MCU 抽象接口：PLAT_SPI_* 方向/主从/帧宽/极性/相位/片选/分频/位序取值宏 */
#include "platform_ch32v203_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief SPI 对象前向声明。 */
typedef struct hal_spi hal_spi_t;

/**
 * @brief SPI对象的生命周期状态。
 */
typedef enum {
    hal_spi_state_reset = 0,
    hal_spi_state_ready,
    hal_spi_state_enabled,
    hal_spi_state_disabled,
    hal_spi_state_error
} hal_spi_state_enum;

/**
 * @brief SPI接口调用返回的状态码。
 */
typedef enum {
    hal_spi_ok = 0,
    hal_spi_err_param = -1,
    hal_spi_err_state = -2,
    hal_spi_err_unsupported = -3,
    hal_spi_err_timeout = -4
} hal_spi_status_enum;

/**
 * @brief SPI 时钟、帧宽、极性/相位、片选、分频与位序配置。
 * @details 字段取值来自 MCU 抽象接口 platform_ch32v203_spi.h 的 PLAT_SPI_* 宏。
 */
typedef struct {
    uint32_t max_frequency_hz;       /**< 期望时钟频率（信息项，实际由分频决定） */
    uint16_t data_size;              /**< PLAT_SPI_DATA_SIZE_8BIT / 16BIT */
    uint16_t mode;                   /**< PLAT_SPI_MODE_MASTER / SLAVE */
    uint16_t clock_polarity;         /**< PLAT_SPI_CPOL_LOW / HIGH */
    uint16_t clock_phase;            /**< PLAT_SPI_CPHA_FIRST_EDGE / SECOND_EDGE */
    uint16_t nss;                    /**< PLAT_SPI_NSS_SOFTWARE / HARDWARE */
    uint16_t baud_rate_prescaler;    /**< PLAT_SPI_BAUD_RATE_PRESCALER_x */
    bool is_msb_first;               /**< true = MSB 先行 */
    bool is_hardware_cs;             /**< false 时由平台层夹持软件片选 */
    uint32_t timeout_count;          /**< 等待 TXE/RXNE/BSY 标志的轮询上限 */
    void *p_user_data;
} hal_spi_config_t;

/**
 * @brief SPI 对象定义。
 * @details 状态、实例基地址与配置；硬件访问经平台操作函数表，无对象内操作表。
 */
struct hal_spi {
    hal_spi_state_enum state;
    uintptr_t base_address;
    hal_spi_config_t config;
};

/**
 * @brief 绑定SPI对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址（传 PLAT_PERIPH_SPIx 时需转 uintptr_t）。
 * @param p_config 模块初始化配置指针。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 参数无效或必需指针为空。
 */
hal_spi_status_enum hal_spi_bind(
    hal_spi_t *p_object,
    uintptr_t base_address,
    const hal_spi_config_t *p_config);

/** @brief 初始化SPI外设：时钟使能 + 配置 + 使能。 */
hal_spi_status_enum hal_spi_init(hal_spi_t *p_object);

/** @brief 关闭并释放SPI外设。 */
hal_spi_status_enum hal_spi_deinit(hal_spi_t *p_object);

/** @brief 使能SPI收发功能。 */
hal_spi_status_enum hal_spi_enable(hal_spi_t *p_object);

/** @brief 禁用SPI收发功能。 */
hal_spi_status_enum hal_spi_disable(hal_spi_t *p_object);

/**
 * @brief 通过SPI同时发送并接收指定长度的数据。
 * @details 软件片选（is_hardware_cs = false）时由平台层在传输前后自动夹持片选。
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
                                    uint16_t data_length);

/** @brief 查询SPI对象当前状态。 */
hal_spi_state_enum hal_spi_get_state(const hal_spi_t *p_object);

/**
 * @brief 构建板级 SPI 实例：绑定板级配置并使能。
 * @param p_driver SPI 对象指针（通常传 &hal_spi_encoder）。
 * @retval hal_spi_ok 操作成功。
 * @retval hal_spi_err_param 参数无效、实例不受支持或配置非法。
 */
hal_spi_status_enum hal_spi_build(hal_spi_t *p_driver);

/**
 * @brief HAL 层 SPI 函数表：对外暴露的构建 / 收发 / 状态 / 备用接口（停用·使能·禁止）。
 * @details 表中的函数指针在 hal_spi.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_spi_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级 SPI 实例（见 hal_spi_build）。 */
    hal_spi_status_enum (*pf_build)(hal_spi_t *p_driver);
    /** @brief 全双工收发一段数据（见 hal_spi_receive）。 */
    hal_spi_status_enum (*pf_receive)(hal_spi_t *p_object,
                                      const uint8_t *p_tx_buf,
                                      uint8_t *p_rx_buf,
                                      uint16_t data_length);
    /** @brief 查询对象状态（见 hal_spi_get_state）。 */
    hal_spi_state_enum (*pf_get_state)(const hal_spi_t *p_object);

    /* --- 备用接口：与同名 HAL 函数一一对应，供外部按需接入 --- */
    /** @brief 关闭并复位SPI外设（见 hal_spi_deinit）。 */
    hal_spi_status_enum (*pf_deinit)(hal_spi_t *p_object);
    /** @brief 使能SPI外设（见 hal_spi_enable）。 */
    hal_spi_status_enum (*pf_enable)(hal_spi_t *p_object);
    /** @brief 禁止SPI外设（见 hal_spi_disable）。 */
    hal_spi_status_enum (*pf_disable)(hal_spi_t *p_object);
} hal_spi_ops_t;

/** @brief HAL 层 SPI 函数表指针（在 hal_spi.c 中定义并绑定）。 */
extern const hal_spi_ops_t *const hal_spi_ops;

/** @brief MT6701 编码器 SPI1 外设对象（板级单实例）。 */
extern hal_spi_t hal_spi_encoder;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_SPI_H__ */
