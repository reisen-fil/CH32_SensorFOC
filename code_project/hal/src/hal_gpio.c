/**
 * @file hal_gpio.c
 * @brief GPIO 对象操作 + CH32V203 GPIO 适配 + 板级引脚表实现。
 * @details 本文件由原 hal_gpio.c 与 platform_ch32v203_gpio.c 合并而成，分为三段：
 *          1) 对象操作函数（同一层级）：hal_gpio_bind / hal_gpio_init /
 *             hal_gpio_write / hal_gpio_read / hal_gpio_toggle /
 *             hal_gpio_get_state，直接完成对象校验、寄存器访问与状态记录；
 *          2) 组配置逻辑：配置视图（合并掩码，每组一次 GPIO_Init）+
 *             操作视图（单 bit 掩码，逐脚读写）；
 *          3) 板级引脚表与 8 个组初始化函数。
 * @note 分层合并：原 "hal_gpio_xxx → 操作表 pf_xxx → hal_gpio_hw_xxx" 三层结构
 *       已合并为单层，不再保留 hw_* 实现函数、分发操作表以及对象内的 p_vtable
 *       字段；引脚级入口也已并入 hal_gpio_write / hal_gpio_read —— 两者以
 *       "组对象 + 引脚索引"为入参，函数内部调用 hal_gpio_get_pin() 完成引脚
 *       配置判断（索引 → 单 bit 操作视图对象 + 越界检查）后访问硬件。
 *       硬件访问统一经平台 GPIO 操作函数表指针 plat_ch32v203_gpio_ops 完成，
 *       本文件不含任何厂商库符号与厂商类型。
 */
#include "hal_gpio.h"
#include <stddef.h>

/* ==========================================================================
 * 1) 对象操作函数（同一层级：对象校验 + 寄存器访问 + 状态记录）
 *    原三层结构 "hal_gpio_xxx → 操作表 pf_xxx → hal_gpio_hw_xxx" 已合并为单层：
 *    hal_gpio_init / hal_gpio_write / hal_gpio_read 直接完成寄存器访问，
 *    不再保留 hw_* 实现函数和分发用操作表。
 * ========================================================================== */

/**
 * @brief 判断是否为 GPIO 输出模式（模拟/复用/输入模式不设置输出电平）。
 * @param mode PLAT_GPIO_MODE_xxx。
 * @retval true 推挽或开漏输出模式。
 * @retval false 其他模式。
 */
static bool hal_gpio_is_output_mode(uint8_t mode)
{
    return (mode == PLAT_GPIO_MODE_OUTPUT_PP) ||
           (mode == PLAT_GPIO_MODE_OUTPUT_OD);
}

/**
 * @brief 检查GPIO对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（端口地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_gpio_is_object_valid(const hal_gpio_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 绑定GPIO对象：保存端口地址、引脚配置与初始状态。
 * @param p_object 模块对象指针。
 * @param base_address 外设寄存器基地址。
 * @param p_config 模块初始化配置指针。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或必需指针为空。
 */
hal_gpio_status_enum hal_gpio_bind(
    hal_gpio_t *p_object,
    uintptr_t base_address,
    const hal_gpio_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) || (base_address == 0U)) {
        return hal_gpio_err_param;
    }

    p_object->state = hal_gpio_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;
    p_object->level = p_config->initial_level;

    return hal_gpio_ok;
}

/**
 * @brief 配置GPIO端口并设置初始电平。
 * @details GPIO_Init 只写 CFGLR/CFGHR（IPD/IPU 会顺带写 BSHR/BCR），
 *          输出电平必须在此显式写入，否则 initial_level 不生效。
 * @param p_object 模块对象指针。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或必需指针为空。
 */
hal_gpio_status_enum hal_gpio_init(hal_gpio_t *p_object)
{
    if (!hal_gpio_is_object_valid(p_object)) {
        return hal_gpio_err_param;
    }

    plat_ch32v203_gpio_ops->pf_clock_enable(p_object->config.gpio_rcc);
    plat_ch32v203_gpio_ops->pf_pin_config(p_object->base_address,
                              (uint16_t)p_object->config.pin_mask,
                              p_object->config.speed,
                              p_object->config.mode);

    if (hal_gpio_is_output_mode(p_object->config.mode)) {
        plat_ch32v203_gpio_ops->pf_write(p_object->base_address,
                             (uint16_t)p_object->config.pin_mask,
                             p_object->config.initial_level);
    }

    p_object->level = p_object->config.initial_level;
    p_object->state = hal_gpio_state_ready;
    return hal_gpio_ok;
}

/**
 * @brief 输出指定引脚的电平（引脚配置判断在函数内部完成）。
 * @details 内部调用 hal_gpio_get_pin() 完成"引脚索引 → 单 bit 操作视图对象"的
 *          配置判断与越界检查，随后直接写寄存器；CH32 的 BSHR/BCR 为原子掩码写。
 * @param p_driver GPIO 组对象指针。
 * @param pin_index 引脚索引，取该组 hal_gpio_xxx_pin_enum。
 * @param level 待设置的 GPIO 电平。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 组对象为空或引脚索引越界。
 */
hal_gpio_status_enum hal_gpio_write(
    hal_gpio_group_t *p_driver,
    uint8_t pin_index,
    bool level)
{
    hal_gpio_t *p_pin = hal_gpio_get_pin(p_driver, pin_index);

    if (p_pin == NULL) {
        return hal_gpio_err_param;
    }

    plat_ch32v203_gpio_ops->pf_write(p_pin->base_address,
                         (uint16_t)p_pin->config.pin_mask,
                         level);

    p_pin->level = level;
    return hal_gpio_ok;
}

/**
 * @brief 读取指定引脚的电平（引脚配置判断在函数内部完成）。
 * @details 内部调用 hal_gpio_get_pin() 取得单 bit 操作视图对象，因此读取掩码
 *          恒为单 bit，天然符合"掩码内任意一位为高"的硬件读语义。
 * @param p_driver GPIO 组对象指针。
 * @param pin_index 引脚索引，取该组 hal_gpio_xxx_pin_enum。
 * @param p_level 输出电平指针。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 组对象为空、索引越界或输出指针为空。
 */
hal_gpio_status_enum hal_gpio_read(
    hal_gpio_group_t *p_driver,
    uint8_t pin_index,
    bool *p_level)
{
    hal_gpio_t *p_pin = hal_gpio_get_pin(p_driver, pin_index);

    if ((p_pin == NULL) || (p_level == NULL)) {
        return hal_gpio_err_param;
    }

    *p_level = plat_ch32v203_gpio_ops->pf_read(p_pin->base_address,
                                   (uint16_t)p_pin->config.pin_mask);

    p_pin->level = *p_level;
    return hal_gpio_ok;
}

/**
 * @brief 翻转GPIO电平。
 * @param p_object 模块对象指针。
 * @retval hal_gpio_err_unsupported 当前平台未实现翻转（BSHR/BCR 无翻转位）。
 */
hal_gpio_status_enum hal_gpio_toggle(hal_gpio_t *p_object)
{
    if (!hal_gpio_is_object_valid(p_object)) {
        return hal_gpio_err_param;
    }

    return hal_gpio_err_unsupported;
}

/**
 * @brief 返回GPIO对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_gpio_state_enum hal_gpio_get_state(const hal_gpio_t *p_object)
{
    if (p_object == NULL) {
        return hal_gpio_state_error;
    }
    return p_object->state;
}

/* ==========================================================================
 * 2) 组配置逻辑（原 platform_ch32v203_gpio.c：双视图构建与逐脚操作）
 * ========================================================================== */

/**
 * @brief 合并组配置参数（匹配键）：同键的引脚才能合并成一次 GPIO_Init。
 * @details 键只包含"写入后不再变化"的组属性，刻意不含 pin_mask ——
 *          该字段会被后续同名引脚按位累加，若纳入匹配键会导致第二轮匹配失败。
 */
typedef struct {
    uintptr_t port;          /**< PLAT_PERIPH_GPIOx */
    uint32_t  rcc;           /**< PLAT_RCC_APB2_GPIOx */
    uint8_t   mode;          /**< PLAT_GPIO_MODE_xxx */
    uint8_t   speed;         /**< PLAT_GPIO_SPEED_xxx */
    bool      initial_level; /**< 仅输出模式生效 */
} hal_gpio_group_key_t;

/**
 * @brief 由引脚描述表项生成本次追加所需的组配置参数。
 * @param p_pin 引脚描述表项。
 * @param p_key 输出：组配置参数（匹配键）。
 */
static void hal_gpio_make_group_key(
    const hal_gpio_pin_desc_t *p_pin,
    hal_gpio_group_key_t *p_key)
{
    p_key->port = p_pin->port;
    p_key->rcc = p_pin->rcc;
    p_key->mode = p_pin->mode;
    p_key->speed = p_pin->speed;
    p_key->initial_level = p_pin->initial_level;
}

/**
 * @brief 用组配置参数判断已有合并组是否同键（同键才能合并成一次 GPIO_Init）。
 * @param p_group 已有配置视图对象（必须已由 hal_gpio_bind_group 配置过）。
 * @param p_key 组配置参数。
 * @retval true 可以合并。
 * @retval false 需要新建合并组。
 */
static bool hal_gpio_match_group(
    const hal_gpio_t *p_group,
    const hal_gpio_group_key_t *p_key)
{
    return (p_group->base_address == p_key->port) &&
           (p_group->config.gpio_rcc == p_key->rcc) &&
           (p_group->config.mode == p_key->mode) &&
           (p_group->config.speed == p_key->speed) &&
           (p_group->config.initial_level == p_key->initial_level);
}

/**
 * @brief 按组配置参数新建一个合并组对象。
 * @details 组对象保存的匹配字段全部取自 p_key，保证"匹配键"与"实际配置"同源，
 *          后续同键引脚必然能匹配上；first_pin_mask 只作为掩码初值。
 * @param p_group 待绑定的配置视图对象。
 * @param p_key 组配置参数。
 * @param first_pin_mask 本组第一个引脚的单 bit 掩码。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或必需指针为空。
 */
static hal_gpio_status_enum hal_gpio_bind_group(
    hal_gpio_t *p_group,
    const hal_gpio_group_key_t *p_key,
    uint16_t first_pin_mask)
{
    hal_gpio_config_t group_config;

    if ((p_group == NULL) || (p_key == NULL)) {
        return hal_gpio_err_param;
    }

    group_config.gpio_rcc = p_key->rcc;
    group_config.pin_mask = first_pin_mask;
    group_config.speed = p_key->speed;
    group_config.mode = p_key->mode;
    group_config.initial_level = p_key->initial_level;

    return hal_gpio_bind(p_group, p_key->port, &group_config);
}

/**
 * @brief 追加一个引脚：绑定操作视图 + 按组配置参数合并/新建配置视图。
 * @details 处理顺序：
 *          ① 组成本引脚的独立配置（操作视图用，单 bit 掩码）；
 *          ② 组成本引脚的组配置参数（匹配键，不含会被累加的 pin_mask）；
 *          ③ 用组配置参数在已有合并组中匹配同键组；
 *          ④ 未命中则该键新建一个合并组（组对象属性与键同源）；
 *          ⑤ 把本引脚掩码并入所选合并组，最后绑定操作视图对象。
 * @param p_driver GPIO 组对象指针。
 * @param p_pin 单引脚描述表项。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或视图容量不足。
 */
static hal_gpio_status_enum hal_gpio_append_pin(
    hal_gpio_group_t *p_driver,
    const hal_gpio_pin_desc_t *p_pin)
{
    hal_gpio_config_t config;
    hal_gpio_group_key_t group_key;
    hal_gpio_status_enum status;
    uint8_t index;
    uint8_t group_index;

    if ((p_driver == NULL) || (p_pin == NULL) || (p_pin->pin_mask == 0U)) {
        return hal_gpio_err_param;
    }
    if (p_driver->pin_count >= HAL_GPIO_PIN_OBJECT_MAX) {
        return hal_gpio_err_param;
    }

    /* ① 引脚独立配置（操作视图：单 bit 掩码） */
    config.gpio_rcc = p_pin->rcc;
    config.pin_mask = p_pin->pin_mask;
    config.speed = p_pin->speed;
    config.mode = p_pin->mode;
    config.initial_level = p_pin->initial_level;

    /* ② 组配置参数（匹配键）：决定本引脚能与哪些已有引脚合并成一次 GPIO_Init */
    hal_gpio_make_group_key(p_pin, &group_key);

    /* ③ 用组配置参数在已有合并组里找同键组 */
    group_index = HAL_GPIO_GROUP_OBJECT_MAX;
    for (index = 0U; index < p_driver->group_count; index++) {
        if (hal_gpio_match_group(&p_driver->group_object[index], &group_key)) {
            group_index = index;
            break;
        }
    }

    /* ④ 未命中：按同一组配置参数新建合并组 */
    if (group_index == HAL_GPIO_GROUP_OBJECT_MAX) {
        if (p_driver->group_count >= HAL_GPIO_GROUP_OBJECT_MAX) {
            return hal_gpio_err_param;
        }
        status = hal_gpio_bind_group(
            &p_driver->group_object[p_driver->group_count],
            &group_key,
            p_pin->pin_mask);
        if (status != hal_gpio_ok) {
            return status;
        }
        group_index = p_driver->group_count;
        p_driver->group_count++;
    }

    /* ⑤ 同键引脚并入该组掩码（后续由配置视图统一执行一次 GPIO_Init） */
    p_driver->group_object[group_index].config.pin_mask |= p_pin->pin_mask;

    /* 操作视图：单 bit 掩码，只绑定不初始化（初始化由配置视图统一完成） */
    status = hal_gpio_bind(&p_driver->pin_object[p_driver->pin_count],
                           p_pin->port,
                           &config);
    if (status != hal_gpio_ok) {
        return status;
    }
    p_driver->pin_count++;

    return hal_gpio_ok;
}

/**
 * @brief 由引脚描述表构建 GPIO 组对象（配置视图 + 操作视图）。
 * @param p_driver GPIO 组对象指针。
 * @param p_pins 引脚描述表首地址。
 * @param pin_count 表项数量。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或视图容量不足。
 */
hal_gpio_status_enum hal_gpio_build(
    hal_gpio_group_t *p_driver,
    const hal_gpio_pin_desc_t *p_pins,
    uint8_t pin_count)
{
    hal_gpio_status_enum status;
    uint8_t index;

    if ((p_driver == NULL) || (p_pins == NULL) || (pin_count == 0U)) {
        return hal_gpio_err_param;
    }

    p_driver->group_count = 0U;
    p_driver->pin_count = 0U;

    for (index = 0U; index < pin_count; index++) {
        status = hal_gpio_append_pin(p_driver, &p_pins[index]);
        if (status != hal_gpio_ok) {
            return status;
        }
    }

    /* 掩码合并完成后，每个合并组只执行一次 GPIO_Init */
    for (index = 0U; index < p_driver->group_count; index++) {
        status = hal_gpio_init(&p_driver->group_object[index]);
        if (status != hal_gpio_ok) {
            return status;
        }
    }

    return hal_gpio_ok;
}

/**
 * @brief 运行期追加一个引脚并立即完成配置。
 * @param p_driver GPIO 组对象指针。
 * @param p_pin 单引脚描述表项。
 * @retval hal_gpio_ok 操作成功。
 * @retval hal_gpio_err_param 参数无效或容量不足。
 * @retval hal_gpio_err_state 追加成功但未找到所属配置视图。
 */
hal_gpio_status_enum hal_gpio_mcu_init(
    hal_gpio_group_t *p_driver,
    const hal_gpio_pin_desc_t *p_pin)
{
    hal_gpio_group_key_t group_key;
    hal_gpio_status_enum status;
    uint8_t index;

    status = hal_gpio_append_pin(p_driver, p_pin);
    if (status != hal_gpio_ok) {
        return status;
    }

    /* 用同一组配置参数查回所属合并组并重新配置（同键掩码，重复配置无副作用） */
    hal_gpio_make_group_key(p_pin, &group_key);
    for (index = 0U; index < p_driver->group_count; index++) {
        if (hal_gpio_match_group(&p_driver->group_object[index], &group_key)) {
            return hal_gpio_init(&p_driver->group_object[index]);
        }
    }

    return hal_gpio_err_state;
}

/**
 * @brief 取操作视图中的引脚对象。
 * @param p_driver GPIO 组对象指针。
 * @param pin_index 引脚索引。
 * @return 引脚对象指针；越界或对象为空时返回 NULL。
 */
hal_gpio_t *hal_gpio_get_pin(hal_gpio_group_t *p_driver, uint8_t pin_index)
{
    if ((p_driver == NULL) || (pin_index >= p_driver->pin_count)) {
        return NULL;
    }
    return &p_driver->pin_object[pin_index];
}

/**
 * @brief 取配置视图中的合并组对象。
 * @param p_driver GPIO 组对象指针。
 * @param group_index 合并组索引。
 * @return 组对象指针；越界或对象为空时返回 NULL。
 */
hal_gpio_t *hal_gpio_get_group(hal_gpio_group_t *p_driver, uint8_t group_index)
{
    if ((p_driver == NULL) || (group_index >= p_driver->group_count)) {
        return NULL;
    }
    return &p_driver->group_object[group_index];
}

/** @brief 取操作视图已构建的引脚数。 */
uint8_t hal_gpio_get_pin_count(const hal_gpio_group_t *p_driver)
{
    return (p_driver == NULL) ? 0U : p_driver->pin_count;
}

/** @brief 取配置视图已合并的组数。 */
uint8_t hal_gpio_get_group_count(const hal_gpio_group_t *p_driver)
{
    return (p_driver == NULL) ? 0U : p_driver->group_count;
}

/* ==========================================================================
 * 3) 板级引脚表与组对象：表项顺序即引脚操作视图索引顺序（与头文件 pin_enum
 *    逐项对应）；表已对外可见，调用方直接用 hal_gpio_build() 构建对应组对象，
 *    不再提供 hal_gpio_xxx_init() 之类的二次封装。
 *    同 (端口,模式,速率,电平) 的表项会被自动合并成一次 GPIO_Init
 * ========================================================================== */

/** @brief 电机 PWM 引脚表：TIM1 六路，PA8/9/10 高端 + PB13/14/15 低端。 */
const hal_gpio_pin_desc_t hal_gpio_tim_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_8,  PLAT_GPIO_MODE_AF_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_9,  PLAT_GPIO_MODE_AF_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_10, PLAT_GPIO_MODE_AF_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_13, PLAT_GPIO_MODE_AF_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_14, PLAT_GPIO_MODE_AF_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_15, PLAT_GPIO_MODE_AF_PP, PLAT_GPIO_LEVEL_RESET)
};

/** @brief 运放引脚表：PA5/6/7 + PB0 输入，PA3/4 + PB1 模拟。 */
const hal_gpio_pin_desc_t hal_gpio_opamp_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_5, PLAT_GPIO_MODE_INPUT_FLOAT, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_6, PLAT_GPIO_MODE_INPUT_FLOAT, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_7, PLAT_GPIO_MODE_INPUT_FLOAT, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_3, PLAT_GPIO_MODE_ANALOG,      PLAT_GPIO_LEVEL_RESET),          // 母线运放采样端（后续需要做母线电流监测/单电阻采样时再进行改动）
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_4, PLAT_GPIO_MODE_ANALOG,      PLAT_GPIO_LEVEL_RESET),          // 
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_0, PLAT_GPIO_MODE_INPUT_FLOAT, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_1, PLAT_GPIO_MODE_ANALOG,      PLAT_GPIO_LEVEL_RESET)
};

/** @brief ADC 常规通道引脚表：PA0 电位器、PA1 母线电压、PA2 NTC。 */
const hal_gpio_pin_desc_t hal_gpio_adc_reguler_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_0, PLAT_GPIO_MODE_ANALOG, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_1, PLAT_GPIO_MODE_ANALOG, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_2, PLAT_GPIO_MODE_ANALOG, PLAT_GPIO_LEVEL_RESET)
};

/** @brief CAN 引脚表：PA12 TX、PA11 RX（默认引脚，无需重映射）。 */
const hal_gpio_pin_desc_t hal_gpio_can_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_12, PLAT_GPIO_MODE_AF_PP,        PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_11, PLAT_GPIO_MODE_INPUT_PULL_UP, PLAT_GPIO_LEVEL_RESET)
};

/** @brief 串口引脚表：PB10 TX、PB11 RX（USART3 默认引脚）。 */
const hal_gpio_pin_desc_t hal_gpio_usart_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_10, PLAT_GPIO_MODE_AF_PP,       PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_11, PLAT_GPIO_MODE_INPUT_FLOAT, PLAT_GPIO_LEVEL_RESET)
};

/**
 * @brief 硬件 I2C 引脚表：PB8/PB9 为 I2C1 重映射脚，空闲电平为高。
 * @note PB8/PB9 属 I2C1 重映射引脚：构建本组前需先经
 *       plat_ch32v203_gpio_ops->pf_remap_enable(PLAT_GPIO_REMAP_AFIO_RCC,
 *       PLAT_GPIO_REMAP_I2C1) 使能重映射，否则信号无法到达 I2C1 外设。
 */
const hal_gpio_pin_desc_t hal_gpio_i2c_hardware_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_8, PLAT_GPIO_MODE_AF_OD, PLAT_GPIO_LEVEL_SET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_9, PLAT_GPIO_MODE_AF_OD, PLAT_GPIO_LEVEL_SET)
};

/** @brief 硬件 SPI/SSI 引脚表：PB3/PB5 复用、PB4 浮空输入、PA15 软件片选（空闲高）。 */
const hal_gpio_pin_desc_t hal_gpio_spi_hardware_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_3,  PLAT_GPIO_MODE_AF_PP,        PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_4,  PLAT_GPIO_MODE_INPUT_FLOAT,  PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_5,  PLAT_GPIO_MODE_AF_PP,        PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOA, PLAT_RCC_APB2_GPIOA, PLAT_GPIO_PIN_15, PLAT_GPIO_MODE_OUTPUT_PP,    PLAT_GPIO_LEVEL_SET)
};

/** @brief 指示灯引脚表：PC14 绿灯、PC15 红灯，初始熄灭。 */
const hal_gpio_pin_desc_t hal_gpio_led_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOC, PLAT_RCC_APB2_GPIOC, PLAT_GPIO_PIN_14, PLAT_GPIO_MODE_OUTPUT_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOC, PLAT_RCC_APB2_GPIOC, PLAT_GPIO_PIN_15, PLAT_GPIO_MODE_OUTPUT_PP, PLAT_GPIO_LEVEL_RESET)
};

/** @brief IO 测试引脚表（PB6 验证 adc 触发位置、PB7 闲置备用），组对象 hal_gpio_io_test。 */
const hal_gpio_pin_desc_t hal_gpio_io_test_pin_table[] = {
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_6, PLAT_GPIO_MODE_OUTPUT_PP, PLAT_GPIO_LEVEL_RESET),
    HAL_GPIO_PIN_DESC(PLAT_PERIPH_GPIOB, PLAT_RCC_APB2_GPIOB, PLAT_GPIO_PIN_7, PLAT_GPIO_MODE_OUTPUT_PP, PLAT_GPIO_LEVEL_RESET)
};

hal_gpio_group_t hal_gpio_tim = {0};

hal_gpio_group_t hal_gpio_opamp = {0};

hal_gpio_group_t hal_gpio_adc_reguler = {0};

hal_gpio_group_t hal_gpio_can = {0};

hal_gpio_group_t hal_gpio_usart = {0};

hal_gpio_group_t hal_gpio_i2c_hardware = {0};

hal_gpio_group_t hal_gpio_spi_hardware = {0};

hal_gpio_group_t hal_gpio_led = {0};

hal_gpio_group_t hal_gpio_io_test = {0};


/* ==========================================================================
 * 4) HAL 层 GPIO 函数表
 * ========================================================================== */

/** @brief HAL 层 GPIO 函数表（本文件私有，绑定本层对外的三个接口）。 */
static const hal_gpio_ops_t s_hal_gpio_ops = {
    .pf_build = hal_gpio_build,
    .pf_write = hal_gpio_write,
    .pf_read = hal_gpio_read
};

/** @brief HAL 层 GPIO 函数表指针（对外外露，声明于 hal_gpio.h）。 */
const hal_gpio_ops_t *const hal_gpio_ops = &s_hal_gpio_ops;
