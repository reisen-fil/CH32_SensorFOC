/**
 * @file bsp_param_store.c
 * @brief 板级参数存储 BSP 实现（A/B 双页交替 + 序号 + CRC-16 的掉电安全参数保存）。
 * @details 结构与本层其它 BSP 模块同构：
 *          - bsp_param_store_init() 经 HAL FLASH 函数表 hal_flash_ops->pf_build 构建片内
 *            FLASH 保留区，再从 A/B 两页中挑出"CRC 有效且序号最大"的一块载入；
 *          - bsp_param_store_save() 把当前参数写进**非活动页**（先 256 字节快速页擦除、
 *            再 256 字节快速页写入，两次独立调用且各自回读校验；两步都成功后才把活动页
 *            切过去）——因此任意时刻至少有一页保存着完整参数，掉电不会丢。
 *          本模块独占片内 FLASH 保留区（hal_flash_onchip），其它模块不得写入该区域。
 * @note **擦除后空白页的读回特征**：本芯片擦除后读回的是 0xE339（半字）而非 0xFF
 *       （手册 32.5.4/32.5.7 与本机实测）。因此整片擦除 / 首次上电时，两页的 magic
 *       都对不上 → is_block_valid() 判无效 → init() 自动取默认值，这是预期行为。
 *       HAL 的擦除校验按"0xE339 或 0xFFFF 均为空白态"判定；写入校验仍是逐字节比对
 *       写入数据（编程过的页读回就是写入的数据），save() 用 0xFF 填充页内未用字节
 *       经整页快速编程后读回即 0xFFFF，没有问题。
 */

/*
    待测项：
    1、A/B区初次写入现象验证（完成）
    2、A/B多次写入后区域切换写入验证（工具链限制，暂不测试）

*/

#include "bsp_param_store.h"
#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * 板级常量与默认参数
 * ========================================================================== */

/**
 * @brief A/B 两页的偏移间隔（字节）。
 * @note 快速编程页为 256 字节，A/B 两页间隔一个 256 字节页；
 *       写入走整页快速擦写（hal_flash_ops 上的 pf_erase_page_fast +
 *       pf_write_page_fast 两次调用），
 *       "擦 A 不会连带擦掉 B"，从而保住掉电安全语义。
 */
#define BSP_PARAM_STORE_PAGE_STRIDE   (256U)

/** @brief CRC-16/CCITT 多项式（x^16 + x^12 + x^5 + 1）。 */
#define BSP_PARAM_STORE_CRC16_POLY    (0x1021U)
/** @brief CRC-16 初值。 */
#define BSP_PARAM_STORE_CRC16_INIT    (0xFFFFU)

/** @brief FLASH 中的参数块布局（RAM 中定义，整体按字节写入 FLASH）。 */
typedef struct {
    uint32_t magic;         /**< 标识 BSP_PARAM_STORE_MAGIC */
    uint16_t version;       /**< 结构版本 BSP_PARAM_STORE_VERSION */
    uint16_t payload_size;  /**< payload 字节数（结构变化时用于判无效） */
    uint32_t sequence;      /**< 递增序号，载入时取最大者 */
    uint16_t crc16;         /**< 覆盖 payload 的 CRC-16/CCITT */
    uint16_t reserved;      /**< 保留（写入 0xFFFF 便于对齐） */
    bsp_param_store_payload_t payload;  /**< 业务参数 */
} bsp_param_store_block_t;

/** @brief 默认参数（FLASH 无有效数据时使用；请按实际电机/板卡填写）。 */
static const bsp_param_store_payload_t s_default_payload = {
    .encoder_zero_offset = 0U,
    .current_offset_u = 0,
    .current_offset_v = 0,
    .bus_divider_num = 1U,
    .bus_divider_den = 1U,
    .motor_pole_pairs = 0U,
    .over_temperature_dc = 1000,     /* 100.0 ℃ */
    .over_current_ma = 5000U,        /* 5 A */
    .speed_pid_kp = 0U,
    .speed_pid_ki = 0U,
    .current_pid_kp = 0U,
    .current_pid_ki = 0U
};

/** @brief 板级参数存储对象（单实例）。 */
bsp_param_store_t bsp_param_store_board = {0};

/** @brief 编译期校验：参数块必须能放进一个快速编程页（256 字节）。 */
typedef char bsp_param_store_block_check
    [(sizeof(bsp_param_store_block_t) <= HAL_FLASH_FAST_PAGE_SIZE) ? 1 : -1];

/* ==========================================================================
 * 内部工具
 * ========================================================================== */

/**
 * @brief CRC-16/CCITT（逐位实现，不查表、不引库）。
 * @param p_data 数据缓冲区。
 * @param length 字节数。
 * @return CRC-16 值。
 */
static uint16_t bsp_param_store_crc16(const uint8_t *p_data, uint32_t length)
{
    uint16_t crc = BSP_PARAM_STORE_CRC16_INIT;
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++) {
        crc ^= (uint16_t)((uint16_t)p_data[index] << 8U);
        for (bit = 8U; bit > 0U; bit--) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((uint16_t)(crc << 1U) ^ BSP_PARAM_STORE_CRC16_POLY);
            } else {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}

/**
 * @brief 判断从 FLASH 读回的块是否有效（标识、版本、长度、CRC 全部匹配）。
 * @note **擦除后的空白页读回不是 0xFF，而是本芯片的特征值 0xE339**
 *       （手册 32.5.4/32.5.7：半字 0xE339，偶地址字节 0x39、奇地址字节 0xE3）。
 *       空白页的 magic 与 BSP_PARAM_STORE_MAGIC 不匹配，因此这里判为无效 →
 *       **首次上电 / 整片擦除后自动取默认值，这是预期行为**，不是读 FLASH 失败。
 * @param p_block 已读回的块。
 * @retval true 有效。
 * @retval false 无效（含空白页）。
 */
static bool bsp_param_store_is_block_valid(const bsp_param_store_block_t *p_block)
{
    if ((p_block->magic != BSP_PARAM_STORE_MAGIC) ||
        (p_block->version != BSP_PARAM_STORE_VERSION) ||
        (p_block->payload_size != (uint16_t)sizeof(bsp_param_store_payload_t))) {
        return false;
    }

    return bsp_param_store_crc16((const uint8_t *)&p_block->payload,
                                 (uint32_t)sizeof(p_block->payload)) ==
           p_block->crc16;
}

/**
 * @brief 读取指定页的块（先整页读入 256 字节页缓冲，再从中取出块）。
 * @details 页缓冲是本函数局部变量，A/B 两次调用复用同一块栈空间，
 *          因此不会同时占用两份 256 字节缓冲。
 * @param offset 页偏移（0 = A 页，BSP_PARAM_STORE_PAGE_STRIDE = B 页）。
 * @param p_block 输出块。
 * @retval true 读取成功。
 * @retval false 读取失败（HAL 拒绝）。
 */
static bool bsp_param_store_read_page(uint32_t offset,
                                     bsp_param_store_block_t *p_block)
{
    uint8_t page_buffer[HAL_FLASH_FAST_PAGE_SIZE];

    if (hal_flash_ops->pf_read_page_fast(&hal_flash_onchip,
                                     HAL_FLASH_ONCHIP_BASE_ADDRESS + offset,
                                     page_buffer) != hal_flash_ok) {
        return false;
    }

    memcpy((void *)p_block, (const void *)page_buffer, sizeof(*p_block));

    return true;
}

/**
 * @brief 把 HAL FLASH 状态码映射为本层状态码。
 * @param status HAL 返回状态。
 * @return 对应的参数存储状态码。
 */
static bsp_param_store_status_enum bsp_param_store_from_flash_status(
    hal_flash_status_enum status)
{
    switch (status) {
    case hal_flash_ok:
        return bsp_param_store_ok;
    case hal_flash_err_param:
        return bsp_param_store_err_param;
    case hal_flash_err_unsupported:
        return bsp_param_store_err_unsupported;
    case hal_flash_err_timeout:
        return bsp_param_store_err_timeout;
    case hal_flash_err_verify:
        return bsp_param_store_err_verify;
    default:
        return bsp_param_store_err_state;
    }
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化参数存储：构建 FLASH 并载入参数（无有效数据时取默认值）。
 * @param p_object 参数存储对象指针。
 * @retval bsp_param_store_ok 操作成功。
 * @retval bsp_param_store_err_param 参数无效。
 * @retval bsp_param_store_err_state FLASH 构建失败。
 */
bsp_param_store_status_enum bsp_param_store_init(bsp_param_store_t *p_object)
{
    bsp_param_store_block_t block_a;
    bsp_param_store_block_t block_b;
    bool is_a_valid;
    bool is_b_valid;

    if (p_object == NULL) {
        return bsp_param_store_err_param;
    }

    /* 板级片内 FLASH 保留区（区域 + 页大小 + 初始化）——经 HAL FLASH 函数表构建 */
    if (hal_flash_ops->pf_build(&hal_flash_onchip) != hal_flash_ok) {
        p_object->state = bsp_param_store_state_error;
        return bsp_param_store_err_state;
    }

    is_a_valid = bsp_param_store_read_page(0U, &block_a) &&
                 bsp_param_store_is_block_valid(&block_a);
    is_b_valid = bsp_param_store_read_page(BSP_PARAM_STORE_PAGE_STRIDE, &block_b) &&
                 bsp_param_store_is_block_valid(&block_b);

    /* 取"CRC 有效且序号最大"的一页作为活动页 */
    if (is_a_valid && (!is_b_valid || (block_a.sequence >= block_b.sequence))) {
        p_object->payload = block_a.payload;
        p_object->sequence = block_a.sequence;
        p_object->active_page = 0U;
        p_object->is_from_flash = true;
    } else if (is_b_valid) {
        p_object->payload = block_b.payload;
        p_object->sequence = block_b.sequence;
        p_object->active_page = 1U;
        p_object->is_from_flash = true;
    } else {
        /* 两页都无效：取默认值，等待上层确认后 save() 落盘 */
        p_object->payload = s_default_payload;
        p_object->sequence = 0U;
        p_object->active_page = 1U;   /* 下次写入 A 页 */
        p_object->is_from_flash = false;
    }

    p_object->state = bsp_param_store_state_ready;
    return bsp_param_store_ok;
}

/**
 * @brief 把当前参数写入 FLASH（写入非活动页，成功后切换活动页）。
 * @param p_object 参数存储对象指针。
 * @retval bsp_param_store_ok 操作成功。
 * @retval bsp_param_store_err_param 参数无效或对象未就绪。
 * @retval bsp_param_store_err_state 擦除或写入失败。
 * @retval bsp_param_store_err_verify 写后回读校验失败。
 */
bsp_param_store_status_enum bsp_param_store_save(bsp_param_store_t *p_object)
{
    bsp_param_store_block_t block;
    uint8_t page_buffer[HAL_FLASH_FAST_PAGE_SIZE];
    hal_flash_status_enum flash_status;
    uint32_t target_offset;

    if ((p_object == NULL) || (p_object->state == bsp_param_store_state_error)) {
        return bsp_param_store_err_param;
    }

    /* 组装待写入的块：序号递增，CRC 覆盖 payload */
    block.magic = BSP_PARAM_STORE_MAGIC;
    block.version = BSP_PARAM_STORE_VERSION;
    block.payload_size = (uint16_t)sizeof(block.payload);
    block.sequence = p_object->sequence + 1U;
    block.reserved = 0xFFFFU;
    block.payload = p_object->payload;
    block.crc16 = bsp_param_store_crc16((const uint8_t *)&block.payload,
                                        (uint32_t)sizeof(block.payload));

    /*
     * 组装整页：快速编程以 256 字节页为单位，块之外的页内字节填 0xFF，
     * 保证传给 HAL 的是完整一页，避免按页读入时越过块边界。
     *
     * 填 0xFF 没有问题：快速页编程是**整页覆盖写**——先把整页（256 字节）按手册流程
     * 擦除，再整页编程，编程后读回就是写入的真实数据（含这些 0xFF 字节，即读回 0xFFFF）。
     * 注意：**芯片擦除后的空白态是 0xE339、不是 0xFF**（手册 32.5.4/32.5.7 与本机实测），
     * 所以擦除校验由 HAL 按"0xE339 或 0xFFFF 均算空"判定，与这里填 0xFF 并不冲突。
     */
    (void)memset(page_buffer, 0xFF, sizeof(page_buffer));
    (void)memcpy(page_buffer, &block, sizeof(block));

    /*
     * 目标页 = 非活动页（活动页上的旧参数在写入成功前始终保留 → 掉电安全）。
     * 先擦除（pf_erase_page_fast 内含整页回读确认为空白态：0xE339 或 0xFFFF），
     * 成功后再写入（pf_write_page_fast 内含写后逐字节回读比对）；两步独立调用、
     * 各自映射状态码。
     *
     * 掉电安全说明：这两次调用之间掉电依然安全——被擦除的是**非活动页**，
     * 活动页上的旧参数块完好无损，下次上电 init() 仍能按 CRC + 序号载入旧参数；
     * 只有"擦除与写入都成功"之后才切换活动页，因此不存在两页同时无效的窗口。
     */
    target_offset = (p_object->active_page == 0U) ? BSP_PARAM_STORE_PAGE_STRIDE : 0U;

    flash_status = hal_flash_ops->pf_erase_page_fast(
        &hal_flash_onchip,
        HAL_FLASH_ONCHIP_BASE_ADDRESS + target_offset);
    if (flash_status != hal_flash_ok) {
        return bsp_param_store_from_flash_status(flash_status);
    }

    flash_status = hal_flash_ops->pf_write_page_fast(
        &hal_flash_onchip,
        HAL_FLASH_ONCHIP_BASE_ADDRESS + target_offset,
        page_buffer);
    if (flash_status != hal_flash_ok) {
        return bsp_param_store_from_flash_status(flash_status);
    }

    /* 擦除与写入两步都成功后才切换活动页 */
    p_object->sequence = block.sequence;
    p_object->active_page = (p_object->active_page == 0U) ? 1U : 0U;
    p_object->is_from_flash = true;
    p_object->state = bsp_param_store_state_ready;

    return bsp_param_store_ok;
}

/**
 * @brief RAM 内恢复默认参数（不写 FLASH）。
 * @param p_object 参数存储对象指针。
 * @retval bsp_param_store_ok 操作成功。
 * @retval bsp_param_store_err_param 参数无效。
 */
bsp_param_store_status_enum bsp_param_store_restore_default(
    bsp_param_store_t *p_object)
{
    if (p_object == NULL) {
        return bsp_param_store_err_param;
    }

    p_object->payload = s_default_payload;
    p_object->state = bsp_param_store_state_dirty;

    return bsp_param_store_ok;
}

/**
 * @brief 取当前参数只读指针。
 * @param p_object 参数存储对象指针。
 * @return 参数指针；参数为空时返回 NULL。
 */
const bsp_param_store_payload_t *bsp_param_store_get_payload(
    const bsp_param_store_t *p_object)
{
    if (p_object == NULL) {
        return NULL;
    }

    return &p_object->payload;
}

/**
 * @brief 查询参数存储对象当前状态。
 * @param p_object 参数存储对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_param_store_state_error。
 */
bsp_param_store_state_enum bsp_param_store_get_state(
    const bsp_param_store_t *p_object)
{
    if (p_object == NULL) {
        return bsp_param_store_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * BSP 参数存储函数表
 * ========================================================================== */

/** @brief BSP 参数存储函数表（本文件私有，绑定板级四个接口）。 */
static const bsp_param_store_ops_t s_bsp_param_store_ops = {
    .pf_init = bsp_param_store_init,
    .pf_save = bsp_param_store_save,
    .pf_restore_default = bsp_param_store_restore_default,
    .pf_get_payload = bsp_param_store_get_payload
};

/** @brief BSP 参数存储函数表指针（对外外露，声明于 bsp_param_store.h）。 */
const bsp_param_store_ops_t *const bsp_param_store_ops = &s_bsp_param_store_ops;
