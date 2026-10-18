/**
 * @file hal_flash.c
 * @brief 片内 FLASH HAL 实现（区域/对齐校验 + 页擦除 + 写入 + 回读校验
 *        + 256 字节快速页擦/写/读）。
 * @details 硬件访问统一经平台 FLASH 操作函数表指针 plat_ch32v203_flash_ops 完成；
 *          擦除后与写入后都以回读方式校验，任何不一致都返回 hal_flash_err_verify，
 *          避免"页大小假设不符"或写入失败被静默放过。
 */
#include "hal_flash.h"
#include <stddef.h>
#include <string.h>

/** @brief 板级片内 FLASH 实例（保留区）。 */
hal_flash_t hal_flash_onchip = {0};

/** @brief 板级片内 FLASH 保留区配置。 */
static const hal_flash_config_t s_flash_onchip_config = {
    .base_address = HAL_FLASH_ONCHIP_BASE_ADDRESS,
    .region_size = HAL_FLASH_ONCHIP_REGION_SIZE,
    .page_size = HAL_FLASH_ONCHIP_PAGE_SIZE,
    .timeout_count = HAL_FLASH_ONCHIP_TIMEOUT_COUNT
};

/**
 * @brief 判断对象是否已就绪。
 * @param p_object FLASH 对象指针。
 * @retval true 对象非空且状态为 ready。
 * @retval false 其它情况。
 */
static bool hal_flash_is_ready(const hal_flash_t *p_object)
{
    return (p_object != NULL) && (p_object->state == hal_flash_state_ready);
}

/**
 * @brief 判断地址区间是否位于保留区内。
 * @param p_object FLASH 对象指针。
 * @param address 起始地址。
 * @param length 长度（字节）。
 * @retval true 完全位于保留区内。
 * @retval false 越界（含回绕）。
 */
static bool hal_flash_is_in_region(const hal_flash_t *p_object,
                                  uint32_t address,
                                  uint32_t length)
{
    uint32_t base = p_object->config.base_address;
    uint32_t end = base + (uint32_t)p_object->config.region_size;

    if ((address < base) || (length == 0U)) {
        return false;
    }
    if (address > (end - 1U)) {
        return false;
    }
    if (length > (end - address)) {
        return false;
    }

    return true;
}

/**
 * @brief 判断一段区域是否处于"擦除后空白态"。
 * @details 逐半字回读（沿用文件内其它回读路径的方式，经平台函数表
 *          plat_ch32v203_flash_ops->pf_read_halfword，地址按 2 字节对齐取下）；
 *          **每个半字等于 HAL_FLASH_BLANK_HALFWORD（0xE339，本芯片手册 32.5.4/32.5.7
 *          规定的擦除后特征值）或 HAL_FLASH_BLANK_HALFWORD_ALT（0xFFFF，镜像内 0xFF
 *          填充被真正编程后读回的值）即算空白**，否则返回 false。
 * @note 判定页里是否"有业务数据"仍以 bsp_param_store 的 magic/version/CRC 为准，
 *       本函数只回答"擦除是否完成"。
 * @param address 起始地址（页起始，须 2 字节对齐）。
 * @param length 长度（字节，须为偶数——本模块的页大小都是偶数）。
 * @retval true 整段均为空白态。
 * @retval false 长度为 0 或奇数，或存在既非 0xE339 也非 0xFFFF 的内容。
 */
static bool hal_flash_is_blank(uint32_t address, uint32_t length)
{
    uint32_t offset;
    uint16_t halfword;

    if ((length == 0U) || ((length & 1U) != 0U)) {
        return false;
    }

    for (offset = 0U; offset < length; offset += 2U) {
        halfword = plat_ch32v203_flash_ops->pf_read_halfword(
            (address + offset) & ~1U);

        if ((halfword != HAL_FLASH_BLANK_HALFWORD) &&
            (halfword != HAL_FLASH_BLANK_HALFWORD_ALT)) {
            return false;
        }
    }

    return true;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 绑定 FLASH 对象：保存保留区配置。
 * @param p_object FLASH 对象指针。
 * @param p_config 保留区配置指针。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效。
 */
hal_flash_status_enum hal_flash_bind(hal_flash_t *p_object,
                                     const hal_flash_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) ||
        (p_config->page_size == 0U) || (p_config->region_size == 0U)) {
        return hal_flash_err_param;
    }

    p_object->config = *p_config;
    p_object->state = hal_flash_state_reset;

    return hal_flash_ok;
}

/**
 * @brief 初始化 FLASH 对象：配置 FLASH 访问时钟，并置对象状态为 ready。
 * @details 顺序为 **解锁 → 配置访问时钟 → 上锁**：
 *          1) FLASH_Access_Clock_Cfg 改写的访问时钟位只在 FLASH 解锁状态下可写
 *             （标准库原注 "Need to unlock"），故这里自行做一次解锁/上锁；
 *          2) 访问时钟取 PLAT_FLASH_ACCESS_CLOCK_HCLK_DIV2，即**把 FLASH 访问（取指）
 *             时钟设为主频（HCLK）的二分频**——降低 FLASH 读访问时钟可减少取指等待，
 *             使本模块的快速编程（256 字节快速页擦/写，手册要求"仅可放在零等待区
 *             FLASH 中"）在代码仍在普通 FLASH 中执行的前提下更稳妥；
 *          3) 配置完成即上锁，擦/写时再按需解锁（本模块所有擦写路径自带解锁/上锁）。
 * @note 本函数只做一次性的静态配置，不擦写任何数据；擦写期间取指被阻塞，
 *       因此**不得在控制中断中调用**。
 * @param p_object FLASH 对象指针。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 对象未绑定（page_size 为 0）。
 * @retval hal_flash_err_state 解锁失败，对象置为 error 态。
 */
hal_flash_status_enum hal_flash_init(hal_flash_t *p_object)
{
    if ((p_object == NULL) || (p_object->config.page_size == 0U)) {
        return hal_flash_err_param;
    }

    /* 解锁 → 配置 FLASH 访问时钟（主频二分频）→ 上锁 */
    if (!plat_ch32v203_flash_ops->pf_unlock_fast()) {
        p_object->state = hal_flash_state_error;
        return hal_flash_err_state;
    }
    (void)plat_ch32v203_flash_ops->pf_access_clock_cfg(
        PLAT_FLASH_ACCESS_CLOCK_HCLK_DIV2);
    (void)plat_ch32v203_flash_ops->pf_lock_fast();

    p_object->state = hal_flash_state_ready;
    return hal_flash_ok;
}

/**
 * @brief 擦除一页并回读校验（整页应为擦除空白态）。
 * @note 擦除校验接受两种空白态：0xE339（本芯片手册 32.5.4 规定的擦除后特征值）
 *       与 0xFFFF（镜像内 0xFF 填充被编程后读回的值）。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（须页对齐且位于保留区内）。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效、地址未对齐或越界。
 * @retval hal_flash_err_state 擦除失败。
 * @retval hal_flash_err_verify 回读发现仍有既非 0xE339 也非 0xFFFF 的内容。
 */
hal_flash_status_enum hal_flash_erase_page(hal_flash_t *p_object,
                                           uint32_t address)
{
    if (!hal_flash_is_ready(p_object)) {
        return hal_flash_err_param;
    }
    if ((address % (uint32_t)p_object->config.page_size) != 0U) {
        return hal_flash_err_param;
    }
    if (!hal_flash_is_in_region(p_object, address,
                                (uint32_t)p_object->config.page_size)) {
        return hal_flash_err_param;
    }

    if (!plat_ch32v203_flash_ops->pf_unlock()) {
        return hal_flash_err_state;
    }
    if (!plat_ch32v203_flash_ops->pf_erase_page(address)) {
        (void)plat_ch32v203_flash_ops->pf_lock();
        return hal_flash_err_state;
    }
    if (!plat_ch32v203_flash_ops->pf_wait_for_last_operation(
            p_object->config.timeout_count)) {
        (void)plat_ch32v203_flash_ops->pf_lock();
        return hal_flash_err_timeout;
    }
    (void)plat_ch32v203_flash_ops->pf_lock();

    /* 回读校验：整页应为擦除空白态（0xE339，兼容 0xFFFF） */
    if (!hal_flash_is_blank(address, (uint32_t)p_object->config.page_size)) {
        return hal_flash_err_verify;
    }

    return hal_flash_ok;
}

/**
 * @brief 擦除一个 256 字节快速编程页，并整页回读确认已擦除（空白态）。
 * @details 流程：校验 → pf_unlock_fast（解锁）→ pf_erase_page_fast
 *          → pf_lock_fast（上锁）→ 回读核对。
 * @note **只需调用一次 pf_unlock_fast**：底层 FLASH_Unlock_Fast 的密钥序列本身
 *       就是"先对 KEYR 写密钥解除普通 LOCK，再对 MODEKEYR 写密钥解除 FLOCK"，
 *       即普通 LOCK 已随该调用一并解开，**上层无需再单独 pf_unlock（重复解锁）**；
 *       同理收尾只做 pf_lock_fast（恢复 FLOCK），不再额外 pf_lock。
 * @note 擦除校验接受两种空白态：0xE339（手册 32.5.7 规定的擦除后特征值）与
 *       0xFFFF（镜像内 0xFF 填充被编程后读回的值）。
 * @note 厂商快速擦除接口没有返回值也没有错误标志，回读是唯一失败检出手段。
 * @warning 擦除失败时上层不应继续写入：未擦净的页做整页覆盖写会按位与出错误数据。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（须按 HAL_FLASH_FAST_PAGE_SIZE 对齐且整页位于保留区内）。
 * @retval hal_flash_ok 操作成功（擦除成功且整页回读为空白态）。
 * @retval hal_flash_err_param 参数无效、地址未对齐或越界。
 * @retval hal_flash_err_state 解锁或擦除失败（已重新上锁）。
 * @retval hal_flash_err_verify 回读发现整页内仍有既非 0xE339 也非 0xFFFF 的内容。
 */
hal_flash_status_enum hal_flash_erase_page_fast(hal_flash_t *p_object,
                                                uint32_t address)
{
    if (!hal_flash_is_ready(p_object)) {
        return hal_flash_err_param;
    }
    if ((address % (uint32_t)HAL_FLASH_FAST_PAGE_SIZE) != 0U) {
        return hal_flash_err_param;
    }
    if (!hal_flash_is_in_region(p_object, address,
                                (uint32_t)HAL_FLASH_FAST_PAGE_SIZE)) {
        return hal_flash_err_param;
    }

    /*
     * 快速编程需要解除 FLOCK（手册 32.5.5 / 32.5.7），而底层 FLASH_Unlock_Fast
     * 已在该调用内部**先解除普通 LOCK、再解除 FLOCK**（两次密钥序列连续执行），
     * 因此这里只调一次 pf_unlock_fast，不再重复 pf_unlock；
     * 任一失败都补一次 pf_lock_fast，不留半解锁状态。
     */
    if (!plat_ch32v203_flash_ops->pf_unlock_fast()) {
        (void)plat_ch32v203_flash_ops->pf_lock_fast();
        return hal_flash_err_state;
    }
    if (!plat_ch32v203_flash_ops->pf_erase_page_fast(address)) {
        (void)plat_ch32v203_flash_ops->pf_lock_fast();
        return hal_flash_err_state;
    }
    (void)plat_ch32v203_flash_ops->pf_lock_fast();

    /* 回读校验：逐半字确认整页为擦除空白态（快速擦除函数无返回值也无错误标志） */
    if (!hal_flash_is_blank(address, (uint32_t)HAL_FLASH_FAST_PAGE_SIZE)) {
        return hal_flash_err_verify;
    }

    return hal_flash_ok;
}

/**
 * @brief 写入一个 256 字节快速编程页，写完回读 256 字节比对。
 * @details 流程：校验 → 数据搬到 4 字节对齐的局部页缓冲 → pf_unlock_fast（解锁）
 *          → pf_program_page_fast → pf_lock_fast（上锁）→ 逐字节回读比对。
 * @note **只需调用一次 pf_unlock_fast**：底层 FLASH_Unlock_Fast 的密钥序列本身
 *       就是"先对 KEYR 写密钥解除普通 LOCK，再对 MODEKEYR 写密钥解除 FLOCK"，
 *       即普通 LOCK 已随该调用一并解开，**上层无需再单独 pf_unlock（重复解锁）**；
 *       同理收尾只做 pf_lock_fast（恢复 FLOCK），不再额外 pf_lock。
 * @note 写后校验口径不变：仍与写入数据 p_data 逐字节比对（编程过的页读回就是写入的
 *       真实数据，与擦除后 0xE339 的空白态无关）。
 * @warning 调用本函数前该页必须已被擦除（如先调 hal_flash_erase_page_fast）；
 *          快速页编程是整页覆盖写，未擦除时写入数据会与旧内容按位与而损坏。
 * @note 使用 CH32V203 快速编程（页 = 256 字节）；手册原注 "快速编程相关函数，
 *       仅可放在零等待区 FLASH 中"，本工程未做零等待区 / RAM 代码搬运。
 * @param p_object FLASH 对象指针。
 * @param address 页起始地址（须按 HAL_FLASH_FAST_PAGE_SIZE 对齐且整页位于保留区内）。
 * @param p_data 源数据缓冲区（必须提供完整 HAL_FLASH_FAST_PAGE_SIZE 字节）。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效、地址未对齐或越界。
 * @retval hal_flash_err_state 解锁或写入失败（已重新上锁）。
 * @retval hal_flash_err_verify 回读与源数据不一致（页未擦除或写入失败）。
 */
hal_flash_status_enum hal_flash_write_page_fast(hal_flash_t *p_object,
                                                uint32_t address,
                                                const uint8_t *p_data)
{
    /* 快速编程按整页（64 个字）写入，缓冲区需 4 字节对齐，故先搬到局部字数组 */
    uint32_t page_buffer[HAL_FLASH_FAST_PAGE_SIZE / 4U];
    uint32_t index;

    if ((!hal_flash_is_ready(p_object)) || (p_data == NULL)) {
        return hal_flash_err_param;
    }
    if ((address % (uint32_t)HAL_FLASH_FAST_PAGE_SIZE) != 0U) {
        return hal_flash_err_param;
    }
    if (!hal_flash_is_in_region(p_object, address,
                                (uint32_t)HAL_FLASH_FAST_PAGE_SIZE)) {
        return hal_flash_err_param;
    }

    memcpy((void *)page_buffer, (const void *)p_data,
           (size_t)HAL_FLASH_FAST_PAGE_SIZE);

    /*
     * 与快速擦除同理，这里只调一次 pf_unlock_fast：
     * 底层 FLASH_Unlock_Fast 内部已经**先解除普通 LOCK（KEYR 密钥序列）、
     * 再解除 FLOCK（MODEKEYR 密钥序列）**，故上层不再重复 pf_unlock；
     * 手册要求密钥序列按序连续执行，该序列由厂商函数在内部一次连续完成。
     * 任一失败都补一次 pf_lock_fast，不留半解锁状态。
     */
    if (!plat_ch32v203_flash_ops->pf_unlock_fast()) {
        (void)plat_ch32v203_flash_ops->pf_lock_fast();
        return hal_flash_err_state;
    }
    if (!plat_ch32v203_flash_ops->pf_program_page_fast(address, page_buffer)) {
        (void)plat_ch32v203_flash_ops->pf_lock_fast();
        return hal_flash_err_state;
    }
    (void)plat_ch32v203_flash_ops->pf_lock_fast();

    /* 回读校验：逐字节比对（快速写页函数不返回状态，只能靠回读发现失败） */
    for (index = 0U; index < (uint32_t)HAL_FLASH_FAST_PAGE_SIZE; index++) {
        uint16_t read_halfword =
            plat_ch32v203_flash_ops->pf_read_halfword((address + index) & ~1U);
        uint8_t read_byte = ((index & 1U) == 0U) ?
                            (uint8_t)(read_halfword & 0xFFU) :
                            (uint8_t)(read_halfword >> 8U);

        if (read_byte != p_data[index]) {
            return hal_flash_err_verify;
        }
    }

    return hal_flash_ok;
}

/**
 * @brief 写入数据并回读校验。
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
                                      uint16_t length)
{
    uint32_t index;
    uint16_t halfword;

    if ((!hal_flash_is_ready(p_object)) || (p_data == NULL)) {
        return hal_flash_err_param;
    }
    if ((address % 2U) != 0U) {
        return hal_flash_err_param;
    }
    if (!hal_flash_is_in_region(p_object, address, (uint32_t)length)) {
        return hal_flash_err_param;
    }

    if (!plat_ch32v203_flash_ops->pf_unlock()) {
        return hal_flash_err_state;
    }

    for (index = 0U; index < (uint32_t)length; index += 2U) {
        if ((index + 1U) < (uint32_t)length) {
            halfword = (uint16_t)((uint16_t)p_data[index] |
                                  ((uint16_t)p_data[index + 1U] << 8U));
        } else {
            /* 长度为奇数：末字节高 8 位补 0xFF（擦除态） */
            halfword = (uint16_t)((uint16_t)p_data[index] | 0xFF00U);
        }

        if (!plat_ch32v203_flash_ops->pf_program_halfword(address + index,
                                                          halfword)) {
            (void)plat_ch32v203_flash_ops->pf_lock();
            return hal_flash_err_state;
        }
    }

    if (!plat_ch32v203_flash_ops->pf_wait_for_last_operation(
            p_object->config.timeout_count)) {
        (void)plat_ch32v203_flash_ops->pf_lock();
        return hal_flash_err_timeout;
    }
    (void)plat_ch32v203_flash_ops->pf_lock();

    /* 回读校验：逐字节比对 */
    for (index = 0U; index < (uint32_t)length; index++) {
        uint16_t read_halfword =
            plat_ch32v203_flash_ops->pf_read_halfword(address + (index & ~1U));
        uint8_t read_byte = ((index & 1U) == 0U) ?
                            (uint8_t)(read_halfword & 0xFFU) :
                            (uint8_t)(read_halfword >> 8U);

        if (read_byte != p_data[index]) {
            return hal_flash_err_verify;
        }
    }

    return hal_flash_ok;
}

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
                                     uint16_t length)
{
    uint32_t index;

    if ((p_object == NULL) || (p_data == NULL)) {
        return hal_flash_err_param;
    }
    if (!hal_flash_is_in_region(p_object, address, (uint32_t)length)) {
        return hal_flash_err_param;
    }

    for (index = 0U; index < (uint32_t)length; index++) {
        uint16_t read_halfword =
            plat_ch32v203_flash_ops->pf_read_halfword((address + index) & ~1U);

        p_data[index] = ((index & 1U) == 0U) ?
                        (uint8_t)(read_halfword & 0xFFU) :
                        (uint8_t)(read_halfword >> 8U);
    }

    return hal_flash_ok;
}

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
                                               uint8_t *p_data)
{
    uint32_t index;

    if ((p_object == NULL) || (p_data == NULL)) {
        return hal_flash_err_param;
    }
    if (!hal_flash_is_in_region(p_object, address,
                                (uint32_t)HAL_FLASH_FAST_PAGE_SIZE)) {
        return hal_flash_err_param;
    }

    for (index = 0U; index < (uint32_t)HAL_FLASH_FAST_PAGE_SIZE; index++) {
        uint16_t read_halfword =
            plat_ch32v203_flash_ops->pf_read_halfword((address + index) & ~1U);

        p_data[index] = ((index & 1U) == 0U) ?
                        (uint8_t)(read_halfword & 0xFFU) :
                        (uint8_t)(read_halfword >> 8U);
    }

    return hal_flash_ok;
}

/**
 * @brief 查询 FLASH 对象当前状态。
 * @param p_object FLASH 对象指针。
 * @return 当前对象状态；参数为空时返回 hal_flash_state_error。
 */
hal_flash_state_enum hal_flash_get_state(const hal_flash_t *p_object)
{
    if (p_object == NULL) {
        return hal_flash_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * HAL 层 FLASH 函数表
 * ========================================================================== */

/* ==========================================================================
 * 板级构建
 * ========================================================================== */

/**
 * @brief 构建板级片内 FLASH 实例（绑定保留区配置并初始化）。
 * @note 内部为 hal_flash_bind + hal_flash_init，故会一并把 FLASH 访问时钟设为主频二分频。
 * @param p_object FLASH 对象指针。
 * @retval hal_flash_ok 操作成功。
 * @retval hal_flash_err_param 参数无效。
 * @retval hal_flash_err_state 解锁失败（见 hal_flash_init）。
 */
hal_flash_status_enum hal_flash_build(hal_flash_t *p_object)
{
    if (hal_flash_bind(p_object, &s_flash_onchip_config) != hal_flash_ok) {
        return hal_flash_err_param;
    }

    return hal_flash_init(p_object);
}

/* ==========================================================================
 * HAL 层 FLASH 函数表
 * ========================================================================== */

/** @brief HAL 层 FLASH 函数表（本文件私有）：标准编程 + 快速编程接口同表绑定。 */
static const hal_flash_ops_t s_hal_flash_ops = {
    .pf_build = hal_flash_build,
    // .pf_init = hal_flash_init,
    .pf_erase_page = hal_flash_erase_page,
    .pf_write = hal_flash_write,
    .pf_read = hal_flash_read,
    .pf_erase_page_fast = hal_flash_erase_page_fast,
    .pf_write_page_fast = hal_flash_write_page_fast,
    .pf_read_page_fast = hal_flash_read_page_fast,
    /* 备用接口：与同名 HAL 函数一一对应，供外部按需接入 */    
    .pf_get_state = hal_flash_get_state
};

/** @brief HAL 层 FLASH 函数表指针（标准编程 + 快速编程同表外露，声明于 hal_flash.h）。 */
const hal_flash_ops_t *const hal_flash_ops = &s_hal_flash_ops;
