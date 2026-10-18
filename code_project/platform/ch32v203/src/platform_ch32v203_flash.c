/**
 * @file platform_ch32v203_flash.c
 * @brief CH32V203 FLASH 平台适配实现（MCU 抽象函数的简单函数封装 + 操作函数表）。
 * @details 本文件把标准库 FLASH 接口逐个做一层简单函数封装，再组装成
 *          plat_ch32v203_flash_ops 函数表对外绑定；hal_flash.c 通过该表访问片内 FLASH。
 * @note 同时提供标准擦写接口与快速编程接口（页 = 256 字节）两套封装：
 *       标准接口（FLASH_ErasePage，粒度 4 KB）对 "代码在 FLASH 中执行" 的场景更稳妥；
 *       快速接口（FLASH_ErasePage_Fast / FLASH_ProgramPage_Fast）用于按 256 字节页
 *       整页擦写，需注意手册原注 "快速编程相关函数，仅可放在零等待区 FLASH 中"。
 * @note 另提供访问时钟配置接口（FLASH_Access_Clock_Cfg）：把 FLASH 访问（取指）时钟
 *       切为主频或主频二分频；该寄存器位需在解锁状态下写入，故由上层按
 *       "解锁 → 配置 → 上锁" 顺序调用（见 hal_flash_init）。
 * @note 另提供挂起标志清除接口（FLASH_ClearFlag，写 1 清除）：入参用 PLAT_FLASH_FLAG_x
 *       掩码，建议在解锁后、擦/写之前调用，清掉上一次遗留的 PGERR/WRPRTERR/EOP
 *       （厂商的 FLASH_GetStatus 只查 BSY 与 WRPRTERR，从不查 PGERR，故只能靠清标志复位）。
 * @note 厂商符号只出现在本文件，对外一律为 bool / 纯计数类型。
 */
#include "platform_ch32v203_flash.h"
#include <stddef.h>

/**
 * @brief 配置 FLASH 访问时钟（FLASH_Access_Clock_Cfg）。
 * @details 该函数改写 FLASH_CTLR 中的访问时钟位（bit25），
 *          **只在 FLASH 解锁状态下可写**（标准库原注 "Need to unlock"），
 *          故调用方须先 plat_ch32v203_flash_unlock()、写完后 plat_ch32v203_flash_lock()
 *          （hal_flash_init 即按 "解锁 → 配置 → 上锁" 顺序调用本接口）。
 * @param access_clock 访问时钟选项，取 PLAT_FLASH_ACCESS_CLOCK_xx
 *                     （HCLK_DIV2 = 主频二分频，HCLK = 主频）。
 * @retval true 操作成功。
 * @note 厂商函数返回 void 且无错误标志可查，故恒返回 true。
 */
static bool plat_ch32v203_flash_access_clock_cfg(uint32_t access_clock)
{
    FLASH_Access_Clock_Cfg(access_clock);
    return true;
}

/**
 * @brief 解锁 FLASH 编程（FLASH_Unlock）。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_flash_unlock(void)
{
    FLASH_Unlock();
    return true;
}

/**
 * @brief 上锁 FLASH（FLASH_Lock）。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_flash_lock(void)
{
    FLASH_Lock();
    return true;
}

/**
 * @brief 擦除一页（FLASH_ErasePage）。
 * @param page_address 页起始地址（需按芯片页大小对齐）。
 * @retval true 擦除完成。
 * @retval false 擦除失败（错误标志或超时）。
 */
static bool plat_ch32v203_flash_erase_page(uint32_t page_address)
{
    return (FLASH_ErasePage(page_address) == PLAT_FLASH_STATUS_COMPLETE);
}

/**
 * @brief 写入半字（FLASH_ProgramHalfWord）。
 * @param address 目标地址（2 字节对齐）。
 * @param data 待写入数据。
 * @retval true 写入完成。
 * @retval false 写入失败。
 */
static bool plat_ch32v203_flash_program_halfword(uint32_t address, uint16_t data)
{
    return (FLASH_ProgramHalfWord(address, (uint16_t)data) == PLAT_FLASH_STATUS_COMPLETE);
}

/**
 * @brief 写入字（FLASH_ProgramWord）。
 * @param address 目标地址（4 字节对齐）。
 * @param data 待写入数据。
 * @retval true 写入完成。
 * @retval false 写入失败。
 */
static bool plat_ch32v203_flash_program_word(uint32_t address, uint32_t data)
{
    return (FLASH_ProgramWord(address, data) == PLAT_FLASH_STATUS_COMPLETE);
}

/**
 * @brief 读取半字（片内 FLASH 可直接寻址读取）。
 * @param address 目标地址。
 * @return 读取到的半字。
 */
static uint16_t plat_ch32v203_flash_read_halfword(uint32_t address)
{
    return *(volatile uint16_t *)address;
}

/**
 * @brief 查询最近一次操作状态（FLASH_GetStatus）。
 * @return 状态码，取 PLAT_FLASH_STATUS_x。
 */
static uint8_t plat_ch32v203_flash_get_status(void)
{
    return (uint8_t)FLASH_GetStatus();
}

/**
 * @brief 等待最近一次操作完成（FLASH_WaitForLastOperation）。
 * @param timeout_count 等待计数上限。
 * @retval true 操作完成。
 * @retval false 超时。
 */
static bool plat_ch32v203_flash_wait_for_last_operation(uint32_t timeout_count)
{
    return (FLASH_WaitForLastOperation(timeout_count) == PLAT_FLASH_STATUS_COMPLETE);
}

/**
 * @brief 清除 FLASH 挂起标志（FLASH_ClearFlag）。
 * @details 厂商函数实现即 `FLASH->STATR = FLASH_FLAG`，属**写 1 清除**；
 *          建议在解锁之后、发起擦/写之前调用（官方例程标准编程流程的第 3 步），
 *          以免上一次遗留的 PGERR / WRPRTERR / EOP 影响本次操作。
 * @param flags 待清除的标志掩码，取 PLAT_FLASH_FLAG_x 的按位或
 *              （或直接用 PLAT_FLASH_FLAG_CLEAR_ALL）。
 * @retval true 操作成功。
 * @note 厂商函数返回 void 且无错误标志可查，故恒返回 true。
 * @note 本接口只清 STATR；选项字节的 `FLASH_FLAG_OPTERR`(0x01) 属 OBR，不在本掩码内。
 */
static bool plat_ch32v203_flash_clear_flag(uint32_t flags)
{
    FLASH_ClearFlag(flags);
    return true;
}

/**
 * @brief 解锁快速编程/快速擦除模式（FLASH_Unlock_Fast）。
 * @details 厂商函数的密钥序列为：**先写 KEYR 解除普通 LOCK，再写 MODEKEYR 解除
 *          FLOCK**（两次序列在函数内部连续执行），之后才可调用
 *          FLASH_ErasePage_Fast / FLASH_ProgramPage_Fast（256 字节页）。
 * @note 因为普通 LOCK 已包含在本调用内，**上层快速编程流程不需要再前置 pf_unlock()**，
 *       重复解锁属于多余操作。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_flash_unlock_fast(void)
{
    FLASH_Unlock_Fast();
    return true;
}

/**
 * @brief 上锁快速编程模式（FLASH_Lock_Fast）。
 * @note 厂商函数只置 FLOCK 位（CR_FLOCK_Set），不恢复普通 LOCK。
 * @retval true 操作成功。
 */
static bool plat_ch32v203_flash_lock_fast(void)
{
    FLASH_Lock_Fast();
    return true;
}

/**
 * @brief 快速擦除一页（FLASH_ErasePage_Fast，**256 字节快速页**）。
 * @param page_address 页起始地址（按 256 字节对齐；函数内部会再掩码低 8 位）。
 * @retval true 擦除完成。
 * @note 厂商函数无返回值，内部自等 BSY 结束（无错误标志可查），故恒返回 true。
 */
static bool plat_ch32v203_flash_erase_page_fast(uint32_t page_address)
{
    FLASH_ErasePage_Fast(page_address);
    return true;
}

/**
 * @brief 快速写入一页（FLASH_ProgramPage_Fast，一次连写 64 个字 = **256 字节**）。
 * @param page_address 页起始地址（按 256 字节对齐）。
 * @param p_buffer 源缓冲区（须至少 256 字节；调用方负责 4 字节对齐，本层仅去 const）。
 * @retval true 写入完成。
 * @note 厂商函数形参为非 const 指针，此处强制去 const 后传入（函数只读该缓冲区）；
 *       厂商函数无返回值，内部自等 BSY 结束，故恒返回 true。
 */
static bool plat_ch32v203_flash_program_page_fast(uint32_t page_address,
                                                 const uint32_t *p_buffer)
{
    uint32_t *p_write_buffer = (uint32_t *)p_buffer;

    FLASH_ProgramPage_Fast(page_address, p_write_buffer);
    return true;
}

/* ==========================================================================
 * 平台层 FLASH 函数表
 * ========================================================================== */

/** @brief 平台层 FLASH 函数表（本文件私有，绑定本层对外接口）。 */
static const platform_ch32v203_flash_ops_t s_flash_ops = {
    .pf_access_clock_cfg = plat_ch32v203_flash_access_clock_cfg,
    .pf_unlock = plat_ch32v203_flash_unlock,
    .pf_lock = plat_ch32v203_flash_lock,
    .pf_erase_page = plat_ch32v203_flash_erase_page,
    .pf_program_halfword = plat_ch32v203_flash_program_halfword,
    .pf_program_word = plat_ch32v203_flash_program_word,
    .pf_read_halfword = plat_ch32v203_flash_read_halfword,
    .pf_get_status = plat_ch32v203_flash_get_status,
    .pf_wait_for_last_operation = plat_ch32v203_flash_wait_for_last_operation,
    .pf_clear_flag = plat_ch32v203_flash_clear_flag,
    .pf_unlock_fast = plat_ch32v203_flash_unlock_fast,
    .pf_lock_fast = plat_ch32v203_flash_lock_fast,
    .pf_erase_page_fast = plat_ch32v203_flash_erase_page_fast,
    .pf_program_page_fast = plat_ch32v203_flash_program_page_fast
};

/** @brief 平台层 FLASH 函数表指针（对外外露，声明于 platform_ch32v203_flash.h）。 */
const platform_ch32v203_flash_ops_t *const plat_ch32v203_flash_ops = &s_flash_ops;
