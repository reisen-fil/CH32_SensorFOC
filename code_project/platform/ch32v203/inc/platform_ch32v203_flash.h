/**
 * @file platform_ch32v203_flash.h
 * @brief CH32V203 片内 FLASH 的 MCU 抽象接口（取值宏 + 操作函数表）。
 * @details 与 GPIO/SPI/ADC/DMA 等平台层同构：
 *          - 取值宏：FLASH 状态码（FLASH_Status）；
 *          - 操作函数表 platform_ch32v203_flash_ops_t：把标准库 FLASH 接口
 *            （FLASH_Access_Clock_Cfg、FLASH_Unlock/Lock、FLASH_ErasePage、
 *            FLASH_ProgramWord/HalfWord、FLASH_GetStatus、FLASH_WaitForLastOperation、
 *            FLASH_ClearFlag）逐个做简单函数封装后绑定；
 *            另含快速编程接口（FLASH_Unlock_Fast/Lock_Fast、FLASH_ErasePage_Fast、
 *            FLASH_ProgramPage_Fast，页 = 256 字节）的封装。
 *            并通过 plat_ch32v203_flash_ops 指针对外外露（表本体私有）。
 * @note 本层不提供任何业务语义（参数分区、掉电安全策略等），只做 MCU 抽象。
 * @note 本头可能经 hal.h 在 ch32v20x_conf.h 之前被解析，因此本头内
 *       **不得出现任何厂商类型**；厂商符号只允许出现在宏体中（延迟展开）
 *       以及 platform_ch32v203_flash.c 内。
 */
#ifndef __PLATFORM_CH32V203_FLASH_H__
#define __PLATFORM_CH32V203_FLASH_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203.h"

/* --------------------------------------------------------------------------
 * 1) 取值宏：标准库 FLASH 状态码与常用参数
 * -------------------------------------------------------------------------- */

/** @brief FLASH 操作状态码（对应标准库 FLASH_Status）。 */
#define PLAT_FLASH_STATUS_COMPLETE         FLASH_COMPLETE
#define PLAT_FLASH_STATUS_BUSY             FLASH_BUSY
#define PLAT_FLASH_STATUS_ERROR_PG         FLASH_ERROR_PG
#define PLAT_FLASH_STATUS_ERROR_WRP        FLASH_ERROR_WRP
#define PLAT_FLASH_STATUS_TIMEOUT          FLASH_TIMEOUT

/** @brief 等待 FLASH 操作完成的默认超时计数。 */
#define PLAT_FLASH_TIMEOUT_COUNT           (100000U)

/**
 * @brief 标准编程的擦除粒度（字节）。
 * @note CH32V203 标准编程流程（FLASH_Unlock → FLASH_ErasePage → FLASH_ProgramHalfWord
 *       → FLASH_Lock）中：**擦除粒度为 4 KB**（FLASH_ErasePage 一次擦除 4 KB 整页），
 *       写入为单次 2 字节（FLASH_ProgramHalfWord）。
 */
#define PLAT_FLASH_STANDARD_ERASE_SIZE     (4096U)

/**
 * @brief 快速编程的页大小（字节）。
 * @note CH32V203 快速编程流程（FLASH_Unlock_Fast → FLASH_ErasePage_Fast →
 *       FLASH_ProgramPage_Fast → FLASH_Lock_Fast）中：**页 = 256 字节**；
 *       FLASH_ErasePage_Fast 按 256 字节页擦除，FLASH_ProgramPage_Fast 一次连写
 *       64 个字（= 256 字节）。
 * @warning 手册原注：**"快速编程相关函数，仅可放在零等待区 FLASH 中"**。
 *          本工程未做零等待区 / RAM 代码搬运（快速编程封装仍链接在普通 FLASH 中执行），
 *          因此该约束未被满足；若实测擦写异常（返回数据错乱、擦写无效等），
 *          需把快速编程封装那段代码搬到零等待区（或 RAM）中执行。
 */
#define PLAT_FLASH_FAST_PAGE_SIZE          (256U)

/**
 * @brief FLASH 访问时钟（取指时钟）选项，对应标准库 FLASH_Access_Clock_Cfg 的入参。
 * @note 语义以标准库**函数说明**（ch32v20x_flash.c 中 FLASH_Access_Clock_Cfg 的注释）
 *       与宏名同时为准：
 *       - PLAT_FLASH_ACCESS_CLOCK_HCLK_DIV2：FLASH 访问时钟 = 主频（HCLK）**二分频**；
 *       - PLAT_FLASH_ACCESS_CLOCK_HCLK     ：FLASH 访问时钟 = 主频。
 * @warning 厂商头文件 ch32v20x_flash.h 中这两个宏旁的注释与函数说明**互相写反了**
 *          （把 FLASH_Access_SYSTEM_HALF 注成 "Enhance Clock = SYSTEM"），
 *          故此处按函数说明与宏名取值：0x00000000（清 FLASH_CTLR 的 bit25）= 二分频，
 *          0x02000000（置 bit25）= 主频。
 * @note 该寄存器位只在 FLASH 解锁状态下才能写入（标准库原注 "Need to unlock"），
 *       调用方需先 pf_unlock()、写完再 pf_lock()。
 */
#define PLAT_FLASH_ACCESS_CLOCK_HCLK_DIV2  FLASH_Access_SYSTEM_HALF
#define PLAT_FLASH_ACCESS_CLOCK_HCLK       FLASH_Access_SYSTEM

/**
 * @brief FLASH 状态标志位（对应标准库 FLASH_FLAG_x），供 pf_clear_flag 使用。
 * @note **这些标志是"写 1 清除"**：标准库 FLASH_ClearFlag(flags) 的实现就是
 *       `FLASH->STATR = flags`；写入对应位即清该标志，故清标志掩码用按位或拼。
 * @note 标准库头 ch32v20x_flash.h **没有定义 PGERR 宏**（只定义了 BSY/EOP/WRPRTERR/OPTERR），
 *       但寄存器位确实存在（ch32v20x.h：`FLASH_STATR_PGERR = 0x04`，即 STATR 的 bit2
 *       "Programming Error"），故此处直接绑定该位，避免"编程错误"这一类标志无法清除
 *       ——注意厂商的 FLASH_GetStatus/FLASH_GetBank1Status **只查 BSY 与 WRPRTERR、
 *       从不查 PGERR**，PGERR 一旦置位只能靠清标志复位。
 * @warning 标准库的 `FLASH_FLAG_OPTERR`(0x01) 与 BSY 同位、且属选项字节寄存器 OBR 的标志，
 *          **不要**把它放进本组 STATR 清标志掩码里。
 */
#define PLAT_FLASH_FLAG_BSY                FLASH_FLAG_BSY         /* 0x01 忙（只读位，写 1 无副作用） */
#define PLAT_FLASH_FLAG_PGERR              FLASH_STATR_PGERR      /* 0x04 编程错误（标准库无此宏） */
#define PLAT_FLASH_FLAG_WRPRTERR           FLASH_FLAG_WRPRTERR    /* 0x10 写保护错误 */
#define PLAT_FLASH_FLAG_EOP                FLASH_FLAG_EOP         /* 0x20 操作结束 */

/** @brief 一次清掉全部 STATR 挂起标志（BSY | PGERR | WRPRTERR | EOP），对应官方例程的清除掩码。 */
#define PLAT_FLASH_FLAG_CLEAR_ALL          (PLAT_FLASH_FLAG_BSY | PLAT_FLASH_FLAG_PGERR | \
                                            PLAT_FLASH_FLAG_WRPRTERR | PLAT_FLASH_FLAG_EOP)

/* --------------------------------------------------------------------------
 * 2) 类型接入宏
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * 3) 操作函数表
 * -------------------------------------------------------------------------- */

/** @brief CH32V203 FLASH 操作函数表（MCU 抽象接口的简单函数封装）。 */
typedef struct {
    /**
     * @brief 配置 FLASH 访问时钟（FLASH_Access_Clock_Cfg）。
     * @note 入参取 PLAT_FLASH_ACCESS_CLOCK_x；写该寄存器位前需先 pf_unlock()。
     */
    bool (*pf_access_clock_cfg)(uint32_t access_clock);
    /** @brief 解锁 FLASH 编程（FLASH_Unlock）。 */
    bool (*pf_unlock)(void);
    /** @brief 上锁 FLASH（FLASH_Lock）。 */
    bool (*pf_lock)(void);
    /** @brief 擦除一页（FLASH_ErasePage），地址需按页对齐。 */
    bool (*pf_erase_page)(uint32_t page_address);
    /** @brief 写入半字（FLASH_ProgramHalfWord），地址需 2 字节对齐。 */
    bool (*pf_program_halfword)(uint32_t address, uint16_t data);
    /** @brief 写入字（FLASH_ProgramWord），地址需 4 字节对齐。 */
    bool (*pf_program_word)(uint32_t address, uint32_t data);
    /** @brief 读取半字（片内 FLASH 可直接寻址读取）。 */
    uint16_t (*pf_read_halfword)(uint32_t address);
    /** @brief 查询最近一次操作状态（FLASH_GetStatus），取 PLAT_FLASH_STATUS_x。 */
    uint8_t (*pf_get_status)(void);
    /** @brief 等待最近一次操作完成（FLASH_WaitForLastOperation）。 */
    bool (*pf_wait_for_last_operation)(uint32_t timeout_count);
    /**
     * @brief 清除 FLASH 挂起标志（FLASH_ClearFlag，"写 1 清除"）。
     * @note 入参取 PLAT_FLASH_FLAG_x 的按位或（或直接用 PLAT_FLASH_FLAG_CLEAR_ALL）；
     *       建议在解锁之后、发起擦/写之前调用，避免上一次遗留的
     *       PGERR/WRPRTERR/EOP 影响本次操作（官方例程的标准编程流程第 3 步即此）。
     */
    bool (*pf_clear_flag)(uint32_t flags);
    /**
     * @brief 解锁快速编程/快速擦除模式（FLASH_Unlock_Fast），用于 256 字节快速页操作。
     * @note 该底层函数的密钥序列**已包含普通 LOCK 的解除**（先 KEYR 再 MODEKEYR），
     *       故快速编程流程只需调用本接口一次，**不要再前置 pf_unlock（重复解锁）**。
     */
    bool (*pf_unlock_fast)(void);
    /** @brief 上锁快速编程模式（FLASH_Lock_Fast，只置 FLOCK 位），用于 256 字节快速页操作。 */
    bool (*pf_lock_fast)(void);
    /** @brief 快速擦除一页（FLASH_ErasePage_Fast），**页 = 256 字节**，地址需按 256 字节对齐。 */
    bool (*pf_erase_page_fast)(uint32_t page_address);
    /** @brief 快速写入一页（FLASH_ProgramPage_Fast），**一次连写 256 字节（64 个字）**。 */
    bool (*pf_program_page_fast)(uint32_t page_address, const uint32_t *p_buffer);
} platform_ch32v203_flash_ops_t;

/** @brief 当前模块操作表（表本体私有，仅此指针外露）。 */
extern const platform_ch32v203_flash_ops_t *const plat_ch32v203_flash_ops;

#endif /* __PLATFORM_CH32V203_FLASH_H__ */
