/**
 * @file hal_flash.h
 * @brief 片内 FLASH 硬件抽象接口（区域校验 + 页擦除 + 写入回读校验 + 256 字节快速页擦/写/读）。
 * @details 与 GPIO/SPI/ADC/DMA 等 HAL 层同构：对象操作函数同层实现，硬件访问统一经
 *          平台 FLASH 操作函数表指针 plat_ch32v203_flash_ops 完成，本层不含厂商库符号。
 *          板级实例只有一个：hal_flash_onchip（片内 FLASH 保留区，用于掉电保存参数）。
 * @note 本层只保证"地址/对齐/区域合法 + 擦写成功 + 回读一致"，不含参数分区、
 *       掉电安全等业务策略（由 bsp_param_store 负责）。
 * @warning FLASH 擦写期间取指会被阻塞，**不得在控制中断中调用**；应在停机或安全状态下调用。
 */
#ifndef __HAL_FLASH_H__
#define __HAL_FLASH_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203_flash.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief FLASH 对象的生命周期状态。 */
typedef enum {
    hal_flash_state_reset = 0,
    hal_flash_state_ready,
    hal_flash_state_error
} hal_flash_state_enum;

/** @brief FLASH 接口调用返回的状态码。 */
typedef enum {
    hal_flash_ok = 0,
    hal_flash_err_param = -1,
    hal_flash_err_state = -2,
    hal_flash_err_unsupported = -3,
    hal_flash_err_timeout = -4,
    hal_flash_err_verify = -5      /**< 回读校验不一致（擦除不完整或写入失败） */
} hal_flash_status_enum;

/**
 * @brief 片内 FLASH 保留区配置。
 * @details base_address + region_size 限定本模块可操作的地址范围，
 *          超出范围一律拒绝，避免误擦写代码区。
 */
typedef struct {
    uint32_t base_address;      /**< 保留区起始地址（页对齐） */
    uint32_t region_size;       /**< 保留区长度（字节） */
    uint16_t page_size;         /**< 擦除页大小（字节） */
    uint32_t timeout_count;     /**< 等待擦写完成的计数上限 */
} hal_flash_config_t;

/** @brief 片内 FLASH 对象。 */
typedef struct {
    hal_flash_state_enum state;   /**< 对象生命周期状态 */
    hal_flash_config_t config;    /**< 保留区配置 */
} hal_flash_t;

/**
 * @brief 片内 FLASH 保留区参数（按 CH32V203 64 KB 代码区顶部 4 KB 预留）。
 * @note 保留区共 4 KB，内部按**快速编程页（256 字节）**划分 A/B 两页：
 *       A 页在偏移 0、B 页在偏移 HAL_FLASH_FAST_PAGE_SIZE，二者互不覆盖；
 *       当前参数块仅几十字节，余量充足。
 */
#define HAL_FLASH_ONCHIP_BASE_ADDRESS   (0x0800F000UL)
#define HAL_FLASH_ONCHIP_REGION_SIZE    (4096UL)
/** @brief 标准擦除页大小：CH32V203 标准编程的擦除粒度为 4 KB（标准擦页接口）。 */
#define HAL_FLASH_ONCHIP_PAGE_SIZE      (4096U)
#define HAL_FLASH_ONCHIP_TIMEOUT_COUNT  (PLAT_FLASH_TIMEOUT_COUNT)

/**
 * @brief 快速编程页大小（字节）。
 * @note CH32V203 快速编程（快速擦页 / 快速写页两个平台接口）**页 = 256 字节**；
 *       本模块的 256 字节整页快速接口（hal_flash_erase_page_fast、
 *       hal_flash_write_page_fast、hal_flash_read_page_fast）均以该值为页单位。
 */
#define HAL_FLASH_FAST_PAGE_SIZE        (256U)

/**
 * @brief 擦除后读回的空白态特征（半字）。
 * @note **本芯片擦除后读回的空白态是 0xE339，不是 0xFF**：CH32FV2x_V3xRM 手册
 *       32.5.4（标准页擦除）与 32.5.7（快速页擦除）的注均写明"擦除成功后，字读
 *       0xE339E339，半字读 0xE339，偶地址字节读 0x39，奇地址字节读 0xE3"；
 *       本机用 WCH-LinkUtility 整片擦除后实测整片读回正是该图案。
 *       因此**擦除校验以本值为准**；而编程过的页读回就是写入的真实数据（已实测），
 *       写入校验仍按写入数据逐字节比对。
 * @note 判断某页里是否"有业务数据"仍以 bsp_param_store 的 magic/version/CRC 为准，
 *       空白态特征只用于判断擦除是否完成。
 */
#define HAL_FLASH_BLANK_HALFWORD        (0xE339U)

/**
 * @brief 兼容空白态特征（半字 0xFFFF）。
 * @note 快速页编程是**整页覆盖写**：镜像内以 0xFF 填充的未用字节被真正编程后，
 *       回读即为 0xFFFF；它与 0xE339 一样判为"空"。
 */
#define HAL_FLASH_BLANK_HALFWORD_ALT    (0xFFFFU)

/** @brief 板级片内 FLASH 实例（保留区，供 bsp_param_store 使用）。 */
extern hal_flash_t hal_flash_onchip;

/**
 * @brief 绑定 FLASH 对象：保存保留区配置。
 * @param p_object FLASH 对象指针。
 * @param p_config 保留区配置指针。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效。
 */
hal_flash_status_enum hal_flash_bind(hal_flash_t *p_object,
                                     const hal_flash_config_t *p_config);

/**
 * @brief 构建板级片内 FLASH 实例（绑定保留区配置并初始化）。
 * @note 内部即 hal_flash_bind + hal_flash_init，因此会一并把 FLASH 访问时钟设为主频二分频。
 * @param p_object FLASH 对象指针。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效。
 * @retval hal_flash_err_state 解锁失败（见 hal_flash_init）。
 */
hal_flash_status_enum hal_flash_build(hal_flash_t *p_object);

/**
 * @brief 初始化 FLASH 对象：配置 FLASH 访问时钟，并置对象状态为 ready。
 * @details 按 "解锁（pf_unlock）→ 配置访问时钟（pf_access_clock_cfg）→ 上锁（pf_lock）"
 *          调用平台接口，把 FLASH 访问（取指）时钟设为
 *          **PLAT_FLASH_ACCESS_CLOCK_HCLK_DIV2（主频二分频）**：该寄存器位只在解锁
 *          状态下可写，且降低 FLASH 访问时钟可减少取指等待，利于代码仍在普通 FLASH
 *          中执行时的快速编程（256 字节快速页擦/写）。
 * @note 本函数只做一次性静态配置，不擦写数据；擦写时各接口自行按需解锁/上锁。
 * @param p_object FLASH 对象指针。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 对象未绑定。
 * @retval hal_flash_err_state 解锁失败（对象置为 error 态，未配置访问时钟）。
 */
hal_flash_status_enum hal_flash_init(hal_flash_t *p_object);

/**
 * @brief 擦除一页并回读校验（整页应为擦除空白态）。
 * @note 擦除校验**接受两种空白态**：HAL_FLASH_BLANK_HALFWORD（0xE339，本芯片手册
 *       32.5.4/32.5.7 规定的擦除后特征值）与 HAL_FLASH_BLANK_HALFWORD_ALT
 *       （0xFFFF，镜像内 0xFF 填充被编程后读回的值）；两者任一即为"空"。
 * @note 标准擦除粒度为 4 KB（CH32V203），一次会擦掉整个 4 KB 保留区；
 *       若要按 256 字节页擦/写、保住 A/B 双页掉电安全语义，请用
 *       hal_flash_erase_page_fast() + hal_flash_write_page_fast()（快速编程）。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（须页对齐且位于保留区内）。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效、地址未对齐或越界。
 * @retval hal_flash_err_state 擦除失败。
 * @retval hal_flash_err_verify 回读发现仍有既非 0xE339 也非 0xFFFF 的内容。
 */
hal_flash_status_enum hal_flash_erase_page(hal_flash_t *p_object,
                                           uint32_t address);

/**
 * @brief 擦除一个 256 字节快速编程页，并对整页回读确认已擦除（空白态）。
 * @details 流程：校验（对象就绪 / 地址按 HAL_FLASH_FAST_PAGE_SIZE 对齐 / 整页位于
 *          保留区内）→ pf_unlock_fast（解锁）→ pf_erase_page_fast →
 *          pf_lock_fast（上锁）→ 逐半字回读确认整页为空白态。
 * @note **只需一次解锁调用 pf_unlock_fast**：底层 FLASH_Unlock_Fast 的密钥序列本身
 *       就是"先对 KEYR 写密钥解除普通 LOCK，再对 MODEKEYR 写密钥解除 FLOCK"，
 *       即普通 LOCK 已随该调用一并解开，**上层不再重复 pf_unlock**；
 *       收尾也只做 pf_lock_fast（恢复 FLOCK）。
 * @note **擦除校验接受两种空白态**：0xE339（本芯片手册 32.5.4/32.5.7 规定的擦除后
 *       特征值）与 0xFFFF（镜像内 0xFF 填充被编程后读回的值）。
 * @note **厂商快速擦除接口既不返回状态也无错误标志**，回读是唯一能检出擦除失败的
 *       手段，因此本函数在 pf_erase_page_fast 之后必须整页核对擦除态。
 * @note 使用 CH32V203 快速编程（页 = 256 字节）；手册原注 "快速编程相关函数，
 *       仅可放在零等待区 FLASH 中"，本工程未做零等待区 / RAM 代码搬运。
 * @warning **擦除失败时上层不应继续写入**：擦除不完整的页上做整页覆盖写会因
 *          "只能 1→0" 而得到按位与后的错误数据。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（须按 HAL_FLASH_FAST_PAGE_SIZE 对齐且整页位于保留区内）。
 * @retval hal_flash_ok 操作成功（擦除成功且整页回读为空白态）。
 * @retval hal_flash_err_param 参数无效、地址未对齐或越界。
 * @retval hal_flash_err_state 解锁或擦除失败（已重新上锁）。
 * @retval hal_flash_err_verify 回读发现整页内仍有既非 0xE339 也非 0xFFFF 的内容。
 */
hal_flash_status_enum hal_flash_erase_page_fast(hal_flash_t *p_object,
                                                uint32_t address);

/**
 * @brief 写入一个 256 字节快速编程页，写完回读 256 字节比对。
 * @details 流程：校验（对象就绪 / 地址按 HAL_FLASH_FAST_PAGE_SIZE 对齐 / 整页位于
 *          保留区内 / p_data 非空）→ 把 256 字节数据搬到 4 字节对齐的局部页缓冲
 *          → pf_unlock_fast（解锁）→ pf_program_page_fast → pf_lock_fast（上锁）
 *          → 逐字节回读比对。
 * @note **只需一次解锁调用 pf_unlock_fast**：底层 FLASH_Unlock_Fast 的密钥序列本身
 *       就是"先对 KEYR 写密钥解除普通 LOCK，再对 MODEKEYR 写密钥解除 FLOCK"，
 *       即普通 LOCK 已随该调用一并解开，**上层不再重复 pf_unlock**；
 *       收尾也只做 pf_lock_fast（恢复 FLOCK）。
 * @note **写后校验口径不变**：仍是"与写入数据 p_data 逐字节比对"（编程过的页读回
 *       就是写入的真实数据，与擦除后的 0xE339 空白态无关）。
 * @warning **调用本函数前该页必须已被擦除**（例如先调用 hal_flash_erase_page_fast()）：
 *          快速页编程是整页覆盖写，FLASH 只能把位由 1 写成 0，页内未擦除的旧内容
 *          会与新数据按位与，导致写入的数据损坏。
 * @note 使用 CH32V203 快速编程（页 = 256 字节）；手册原注 "快速编程相关函数，
 *       仅可放在零等待区 FLASH 中"，本工程未做零等待区 / RAM 代码搬运。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（须按 HAL_FLASH_FAST_PAGE_SIZE 对齐且整页位于保留区内）。
 * @param p_data 源数据缓冲区（必须提供完整 HAL_FLASH_FAST_PAGE_SIZE 字节）。
 * @retval hal_flash_ok 操作成功（写入且回读校验通过）。
 * @retval hal_flash_err_param 参数无效、地址未对齐或越界。
 * @retval hal_flash_err_state 解锁或写入失败（已重新上锁）。
 * @retval hal_flash_err_verify 回读与源数据不一致（页未擦除或写入失败）。
 */
hal_flash_status_enum hal_flash_write_page_fast(hal_flash_t *p_object,
                                                uint32_t address,
                                                const uint8_t *p_data);

/**
 * @brief 读取一个 256 字节快速编程页。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（整页须位于保留区内）。
 * @param p_data 目标缓冲区（必须提供完整 HAL_FLASH_FAST_PAGE_SIZE 字节）。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效或越界。
 */
hal_flash_status_enum hal_flash_read_page_fast(hal_flash_t *p_object,
                                               uint32_t address,
                                               uint8_t *p_data);

/**
 * @brief 写入数据并回读校验（不需先擦除时可直接调用）。
 * @details 按半字写入；长度为奇数时末字节高 8 位补 0xFF。
 * @param p_object FLASH 对象指针。
 * @param address 目标地址（半字对齐且位于保留区内）。
 * @param p_data 源数据缓冲区。
 * @param length 字节数。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效、未对齐或越界。
 * @retval hal_flash_err_state 写入失败。
 * @retval hal_flash_err_verify 回读与源数据不一致。
 */
hal_flash_status_enum hal_flash_write(hal_flash_t *p_object,
                                      uint32_t address,
                                      const uint8_t *p_data,
                                      uint16_t length);

/**
 * @brief 读取数据（片内 FLASH 可直接寻址）。
 * @param p_object FLASH 对象指针。
 * @param address 源地址（位于保留区内）。
 * @param p_data 目标缓冲区。
 * @param length 字节数。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效或越界。
 */
hal_flash_status_enum hal_flash_read(hal_flash_t *p_object,
                                     uint32_t address,
                                     uint8_t *p_data,
                                     uint16_t length);

/**
 * @brief 查询 FLASH 对象当前状态。
 * @param p_object FLASH 对象指针。
 * @return 当前对象状态；参数为空时返回 hal_flash_state_error。
 */
hal_flash_state_enum hal_flash_get_state(const hal_flash_t *p_object);

/**
 * @brief HAL 层 FLASH 函数表：标准编程 + 快速编程接口**同表外露**。
 * @details 一张表同时承载两类接口：
 *          - 生命周期：pf_build；
 *          - 标准编程（4 KB 擦页 + 2 字节半字写）：pf_erase_page / pf_write / pf_read；
 *          - 快速编程（256 字节整页擦 / 写 / 读）：pf_erase_page_fast /
 *            pf_write_page_fast / pf_read_page_fast；
 *          - 状态查询：pf_get_state。
 *          表中的函数指针在 hal_flash.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 hal_flash_ops 指针统一调用。
 */
typedef struct {
    /** @brief 构建板级实例（生命周期，见 hal_flash_build）。 */
    hal_flash_status_enum (*pf_build)(hal_flash_t *p_object);
    /** @brief 初始化（生命周期，见 hal_flash_init）。 */
    // hal_flash_status_enum (*pf_init)(hal_flash_t *p_object);
    /** @brief 擦除一页并校验（标准编程，见 hal_flash_erase_page，擦除粒度 4 KB）。 */
    hal_flash_status_enum (*pf_erase_page)(hal_flash_t *p_object, uint32_t address);
    /** @brief 写入并校验（标准编程，见 hal_flash_write，单次 2 字节半字写）。 */
    hal_flash_status_enum (*pf_write)(hal_flash_t *p_object, uint32_t address,
                                      const uint8_t *p_data, uint16_t length);
    /** @brief 读取（标准编程，见 hal_flash_read）。 */
    hal_flash_status_enum (*pf_read)(hal_flash_t *p_object, uint32_t address,
                                     uint8_t *p_data, uint16_t length);
    /** @brief 擦除 256 字节整页并回读确认已擦除（空白态 0xE339 或 0xFFFF；快速编程，见 hal_flash_erase_page_fast）。 */
    hal_flash_status_enum (*pf_erase_page_fast)(hal_flash_t *p_object,
                                                uint32_t address);
    /** @brief 写入 256 字节整页并回读校验（快速编程，见 hal_flash_write_page_fast；调用前该页须已擦除）。 */
    hal_flash_status_enum (*pf_write_page_fast)(hal_flash_t *p_object,
                                                uint32_t address,
                                                const uint8_t *p_data);
    /** @brief 读取 256 字节整页（快速编程，见 hal_flash_read_page_fast）。 */
    hal_flash_status_enum (*pf_read_page_fast)(hal_flash_t *p_object,
                                               uint32_t address,
                                               uint8_t *p_data);
    /** @brief 查询状态（见 hal_flash_get_state）。 */
    hal_flash_state_enum (*pf_get_state)(const hal_flash_t *p_object);
} hal_flash_ops_t;

/** @brief HAL 层 FLASH 函数表指针（标准编程 + 快速编程同表，在 hal_flash.c 中定义并绑定）。 */
extern const hal_flash_ops_t *const hal_flash_ops;

#ifdef __cplusplus
}
#endif

#endif /* __HAL_FLASH_H__ */
