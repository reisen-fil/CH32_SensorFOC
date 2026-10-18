/**
 * @file bsp_param_store.h
 * @brief 板级参数存储 BSP 接口（掉电保存电机参数与标定值）。
 * @details 结构与本层 bsp_encoder / bsp_motor_ctrl / bsp_analog_sense 同构：
 *          对象只保存状态，接口 + 一张 BSP 函数表：
 *          - bsp_param_store_init()：从片内 FLASH 载入参数（无效则取默认值）；
 *          - bsp_param_store_save()：把当前参数写入 FLASH（A/B 双页交替，掉电安全）；
 *          - bsp_param_store_restore_default()：RAM 内恢复默认值（不写 FLASH）；
 *          - bsp_param_store_get_payload()：取当前参数只读指针。
 *          **掉电安全**：A/B 两页交替写入，新块带递增序号 + CRC-16；每次只擦除非活动页，
 *          因此任意时刻至少有一页保存着完整校验通过的参数；载入时取"CRC 有效且序号最大"的一页。
 *          硬件访问一律经 HAL FLASH 函数表指针 hal_flash_ops；本模块独占片内 FLASH 保留区
 *          （hal_flash_onchip）。
 * @note **擦除后空白页的读回特征不是 0xFF，而是本芯片的特征值 0xE339**
 *       （CH32FV2x_V3xRM 手册 32.5.4/32.5.7：擦除成功后字读 0xE339E339、半字读 0xE339、
 *       偶地址字节读 0x39、奇地址字节读 0xE3；本机整片擦除实测一致）。
 *       因此 `bsp_param_store_is_block_valid()` 会把空白页判为无效（magic 不匹配）→
 *       **首次上电 / 整片擦除后自动取默认值，这是预期行为**。
 *       而 `save()` 用 0xFF 填充页内未用字节没有问题：快速页编程是整页覆盖写，
 *       编程后读回即写入的真实数据（已被手册流程与实测证实），HAL 的写入校验仍是
 *       逐字节与写入数据比对；擦除校验则由 HAL 按"0xE339 或 0xFFFF 均算空白"判定。
 * @warning FLASH 擦写期间取指被阻塞，**不得在控制中断中调用 save()**；
 *          应在停机或安全状态下调用，并注意写入耗时（毫秒级）。
 * @warning save() 走**快速编程**（256 字节页：快速解锁 → 快速擦页 → 快速写页 →
 *          快速上锁；底层 FLASH_Unlock_Fast 的密钥序列已含"先解 LOCK 再解 FLOCK"，
 *          故不再单独做普通解锁/上锁）。
 *          依据 CH32V203 手册原注 "快速编程相关函数，仅可放在零等待区 FLASH 中"——
 *          本工程**未做**零等待区 / RAM 代码搬运，该封装仍链接在普通 FLASH 中执行；
 *          若实测保存异常（写入无效、回读校验失败等），请把该段封装搬到零等待区（或 RAM）中执行。
 */
#ifndef __BSP_PARAM_STORE_H__
#define __BSP_PARAM_STORE_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 参数存储对象的生命周期状态。 */
typedef enum {
    bsp_param_store_state_reset = 0,
    bsp_param_store_state_ready,   /**< 已载入参数（来自 FLASH 或默认值） */
    bsp_param_store_state_dirty,   /**< RAM 中参数已修改，尚未写回 FLASH */
    bsp_param_store_state_error
} bsp_param_store_state_enum;

/** @brief 参数存储接口调用返回的状态码。 */
typedef enum {
    bsp_param_store_ok = 0,
    bsp_param_store_err_param = -1,
    bsp_param_store_err_state = -2,
    bsp_param_store_err_unsupported = -3,
    bsp_param_store_err_timeout = -4,
    bsp_param_store_err_verify = -5   /**< 写后回读校验失败 */
} bsp_param_store_status_enum;

/** @brief 参数表标识与版本（用于判断 FLASH 内容是否属于本工程且结构匹配）。 */
#define BSP_PARAM_STORE_MAGIC      (0x4D435450UL)   /**< "PTCM" */
#define BSP_PARAM_STORE_VERSION    (0x0001U)

/**
 * @brief 掉电保存的参数集合（业务参数，全部为整数域）。
 * @note 新增字段时请同步修改 BSP_PARAM_STORE_VERSION 与默认值表。
 */
typedef struct {
    uint32_t encoder_zero_offset;   /**< 编码器零位偏置（计数） */
    int32_t current_offset_u;       /**< U 相电流零漂（ADC 原始计数） */
    int32_t current_offset_v;       /**< V 相电流零漂（ADC 原始计数） */
    uint16_t bus_divider_num;       /**< 母线分压比分子 */
    uint16_t bus_divider_den;       /**< 母线分压比分母 */
    uint8_t motor_pole_pairs;       /**< 电机极对数 */
    int16_t over_temperature_dc;    /**< 过温阈值（0.1 ℃） */
    uint16_t over_current_ma;       /**< 过流阈值（mA） */
    uint32_t speed_pid_kp;          /**< 速度环 Kp（定点） */
    uint32_t speed_pid_ki;          /**< 速度环 Ki（定点） */
    uint32_t current_pid_kp;        /**< 电流环 Kp（定点） */
    uint32_t current_pid_ki;        /**< 电流环 Ki（定点） */
} bsp_param_store_payload_t;

/** @brief 参数存储对象：生命周期状态 + 当前参数 + 活动页记录。 */
typedef struct {
    bsp_param_store_state_enum state;    /**< 对象生命周期状态 */
    bsp_param_store_payload_t payload;   /**< 当前参数（RAM 副本） */
    uint32_t sequence;                   /**< 当前参数对应的序号 */
    uint8_t active_page;                 /**< 当前活动页号（0=A，1=B） */
    bool is_from_flash;                  /**< true 载自 FLASH，false 为默认值 */
} bsp_param_store_t;

/** @brief 板级参数存储对象（单实例）。 */
extern bsp_param_store_t bsp_param_store_board;

/**
 * @brief 初始化参数存储：构建 FLASH 并载入参数（无有效数据时取默认值）。
 * @param p_object 参数存储对象指针。
 * @retval bsp_param_store_ok 操作成功（含"无有效数据、已取默认值"）。
 * @retval bsp_param_store_err_param 参数无效。
 * @retval bsp_param_store_err_state FLASH 构建失败。
 */
bsp_param_store_status_enum bsp_param_store_init(bsp_param_store_t *p_object);

/**
 * @brief 把当前参数写入 FLASH（写入非活动页，成功后切换活动页）。
 * @param p_object 参数存储对象指针。
 * @retval bsp_param_store_ok 操作成功。
 * @retval bsp_param_store_err_param 参数无效或对象未就绪。
 * @retval bsp_param_store_err_state 擦除或写入失败。
 * @retval bsp_param_store_err_verify 写后回读校验失败。
 */
bsp_param_store_status_enum bsp_param_store_save(bsp_param_store_t *p_object);

/**
 * @brief RAM 内恢复默认参数（不写 FLASH，需再调用 save 才会掉电保存）。
 * @param p_object 参数存储对象指针。
 * @retval bsp_param_store_ok 操作成功。
 * @retval bsp_param_store_err_param 参数无效。
 */
bsp_param_store_status_enum bsp_param_store_restore_default(
    bsp_param_store_t *p_object);

/**
 * @brief 取当前参数只读指针。
 * @param p_object 参数存储对象指针。
 * @return 参数指针；参数为空时返回 NULL。
 */
const bsp_param_store_payload_t *bsp_param_store_get_payload(
    const bsp_param_store_t *p_object);

/**
 * @brief 查询参数存储对象当前状态。
 * @param p_object 参数存储对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_param_store_state_error。
 */
bsp_param_store_state_enum bsp_param_store_get_state(
    const bsp_param_store_t *p_object);

/**
 * @brief BSP 参数存储函数表：初始化 / 写回 / 恢复默认 / 取参数。
 * @details 表中的函数指针在 bsp_param_store.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 bsp_param_store_ops 指针统一调用。
 */
typedef struct {
    /** @brief 初始化并载入（见 bsp_param_store_init）。 */
    bsp_param_store_status_enum (*pf_init)(bsp_param_store_t *p_object);
    /** @brief 写回 FLASH（见 bsp_param_store_save）。 */
    bsp_param_store_status_enum (*pf_save)(bsp_param_store_t *p_object);
    /** @brief RAM 内恢复默认（见 bsp_param_store_restore_default）。 */
    bsp_param_store_status_enum (*pf_restore_default)(bsp_param_store_t *p_object);
    /** @brief 取参数只读指针（见 bsp_param_store_get_payload）。 */
    const bsp_param_store_payload_t *(*pf_get_payload)(
        const bsp_param_store_t *p_object);
} bsp_param_store_ops_t;

/** @brief BSP 参数存储函数表指针（在 bsp_param_store.c 中定义并绑定）。 */
extern const bsp_param_store_ops_t *const bsp_param_store_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PARAM_STORE_H__ */
