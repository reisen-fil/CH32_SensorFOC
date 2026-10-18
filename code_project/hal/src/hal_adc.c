/**
 * @file hal_adc.c
 * @brief ADC 对象操作 + CH32V203 板级实例（规则通道+DMA / 注入通道）+ HAL 函数表。
 * @details 硬件访问统一经平台 ADC 操作函数表指针 plat_ch32v203_adc_ops 与平台 DMA
 *          操作函数表指针 plat_ch32v203_dma_ops 完成，引脚组构建经 HAL GPIO 函数表
 *          指针 hal_gpio_ops 完成，本文件不含厂商库符号。
 *          板级配置依据参考实现 drv_adc.c：
 *          - OPA/ADC 引脚：PA5/6/7、PB0 浮空输入，PA3/4、PB1 模拟（见 hal_gpio_opamp）；
 *          - ADC1 规则：PA0(通道0)、PA1(通道1)，239.5 周期，扫描 + 连续 + DMA 循环；
 *          - ADC2 注入：PB1(通道9) 序号1、PA4(通道4) 序号2，13.5 周期，TIM1_TRGO 触发。
 */
#include "hal_adc.h"
#include "hal_gpio.h"
#include "platform_ch32v203_dma.h"
#include <stddef.h>

/* ==========================================================================
 * 板级配置：通道序列与 ADC 参数
 * ========================================================================== */

/** @brief 规则通道序列：PA0（电位器，通道 0）、PA1（母线电压，通道 1）、PA2（NTC，通道 2）。 */
static const hal_adc_channel_config_t s_adc_reguler_channels[] = {
    { PLAT_ADC_CHANNEL_0, 1U, PLAT_ADC_SAMPLE_TIME_239CYCLES5 },
    { PLAT_ADC_CHANNEL_1, 2U, PLAT_ADC_SAMPLE_TIME_239CYCLES5 },
    { PLAT_ADC_CHANNEL_2, 3U, PLAT_ADC_SAMPLE_TIME_239CYCLES5 }
};

/** @brief 注入通道序列：PB1（运放 1 输出，通道 9）、PA4（运放 2 输出，通道 4）。 */
static const hal_adc_channel_config_t s_adc_injected_channels[] = {
    { PLAT_ADC_CHANNEL_9, 3U, PLAT_ADC_SAMPLE_TIME_13CYCLES5 },
    { PLAT_ADC_CHANNEL_4, 4U, PLAT_ADC_SAMPLE_TIME_13CYCLES5 }
};

/** @brief ADC1 规则通道参数：独立模式、扫描+连续、软件触发、右对齐、3 通道。 */
static const hal_adc_config_t s_adc_reguler_config = {
    .mode = PLAT_ADC_MODE_INDEPENDENT,
    .is_scan_enabled = true,
    .is_continuous_enabled = true,
    .external_trigger = PLAT_ADC_EXTERNAL_TRIG_NONE,
    .data_align = PLAT_ADC_DATA_ALIGN_RIGHT,
    .channel_count = HAL_ADC_REGULER_CHANNEL_COUNT,
    .timeout_count = 100000U,
    .p_user_data = NULL
};

/** @brief ADC2 注入通道参数：独立模式、扫描、非连续、软件触发、右对齐、2 通道。 */
static const hal_adc_config_t s_adc_injected_config = {
    .mode = PLAT_ADC_MODE_INDEPENDENT,
    .is_scan_enabled = true,
    .is_continuous_enabled = false,
    .external_trigger = PLAT_ADC_EXTERNAL_TRIG_NONE,
    .data_align = PLAT_ADC_DATA_ALIGN_RIGHT,
    .channel_count = HAL_ADC_INJECTED_CHANNEL_COUNT,
    .timeout_count = 100000U,
    .p_user_data = NULL
};

/** @brief 注入组中断线（ADC1/ADC2 共用 PLAT_ADC_IRQ_ADC1_2）优先级：抢占 0 / 子 0。
 *  @note 本工程所用标准库 NVIC_Init() 只区分抢占优先级 0/1 两档；电流环采样控制
 *        取最高档 0，串口空闲中断取 1（见 hal_uart.c 的 HAL_UART_IDLE_IRQ_*
 *        板级常量），以保证采样中断不会被串口中断打断。 */
#define HAL_ADC_INJECTED_IRQ_PREEMPTION_PRIORITY   (0U)
#define HAL_ADC_INJECTED_IRQ_SUB_PRIORITY          (0U)

/** @brief ADC1 规则通道 DMA 采样缓存（由平台 DMA 抽象通道循环更新）。 */
uint16_t hal_adc_reguler_dma_value[HAL_ADC_REGULER_CHANNEL_COUNT] = {0};

/** @brief ADC1 规则通道实例。 */
hal_adc_t hal_adc_reguler = {0};
/** @brief ADC2 注入通道实例。 */
hal_adc_t hal_adc_injected = {0};

/* ==========================================================================
 * 对象操作
 * ========================================================================== */

/**
 * @brief 检查ADC对象是否已绑定可用。
 * @param p_object 模块对象指针。
 * @retval true 对象已绑定（实例基地址有效）。
 * @retval false 对象为空或未绑定。
 */
static bool hal_adc_is_object_valid(const hal_adc_t *p_object)
{
    return (p_object != NULL) && (p_object->base_address != 0U);
}

/**
 * @brief 绑定ADC对象：保存实例基地址与配置。
 * @param p_object 模块对象指针。
 * @param base_address 外设实例基地址。
 * @param p_config 模块初始化配置指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效或必需指针为空。
 */
hal_adc_status_enum hal_adc_bind(hal_adc_t *p_object,
                                 uintptr_t base_address,
                                 const hal_adc_config_t *p_config)
{
    if ((p_object == NULL) || (p_config == NULL) || (base_address == 0U)) {
        return hal_adc_err_param;
    }

    p_object->state = hal_adc_state_reset;
    p_object->base_address = base_address;
    p_object->config = *p_config;

    return hal_adc_ok;
}

/**
 * @brief 初始化ADC：时钟 + 配置 + 内部缓冲关闭 + 使能 + 校准。
 * @param p_object 模块对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定、实例不受支持或配置非法。
 * @retval hal_adc_err_timeout 校准等待超时。
 */
hal_adc_status_enum hal_adc_init(hal_adc_t *p_object)
{
    platform_ch32v203_adc_config_t platform_config;
    uintptr_t instance;

    if (!hal_adc_is_object_valid(p_object)) {
        return hal_adc_err_param;
    }

    instance = p_object->base_address;

    platform_config.mode = p_object->config.mode;
    platform_config.is_scan_enabled = p_object->config.is_scan_enabled;
    platform_config.is_continuous_enabled = p_object->config.is_continuous_enabled;
    platform_config.external_trigger = p_object->config.external_trigger;
    platform_config.data_align = p_object->config.data_align;
    platform_config.channel_count = p_object->config.channel_count;

    if ((!plat_ch32v203_adc_ops->pf_clock_enable(instance)) ||
        (!plat_ch32v203_adc_ops->pf_deinit(instance)) ||
        (!plat_ch32v203_adc_ops->pf_config(instance, &platform_config)) ||
        (!plat_ch32v203_adc_ops->pf_cmd(instance, true)) ||
        (!plat_ch32v203_adc_ops->pf_buffer_cmd(instance, false))) {
        p_object->state = hal_adc_state_error;
        return hal_adc_err_param;
    }

    if (!plat_ch32v203_adc_ops->pf_calibrate(instance)) {
        p_object->state = hal_adc_state_error;
        return hal_adc_err_timeout;
    }

    p_object->state = hal_adc_state_ready;
    return hal_adc_ok;
}

/**
 * @brief 关闭并复位ADC外设。
 * @param p_object 模块对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定或实例不受支持。
 */
hal_adc_status_enum hal_adc_deinit(hal_adc_t *p_object)
{
    if (!hal_adc_is_object_valid(p_object)) {
        return hal_adc_err_param;
    }

    if ((!plat_ch32v203_adc_ops->pf_cmd(p_object->base_address, false)) ||
        (!plat_ch32v203_adc_ops->pf_deinit(p_object->base_address))) {
        return hal_adc_err_param;
    }

    p_object->state = hal_adc_state_reset;
    return hal_adc_ok;
}

/**
 * @brief 使能ADC转换。
 * @param p_object 模块对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定或实例不受支持。
 */
hal_adc_status_enum hal_adc_enable(hal_adc_t *p_object)
{
    if (!hal_adc_is_object_valid(p_object)) {
        return hal_adc_err_param;
    }

    if (!plat_ch32v203_adc_ops->pf_cmd(p_object->base_address, true)) {
        return hal_adc_err_param;
    }

    p_object->state = hal_adc_state_enabled;
    return hal_adc_ok;
}

/**
 * @brief 禁止ADC转换。
 * @param p_object 模块对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定或实例不受支持。
 */
hal_adc_status_enum hal_adc_disable(hal_adc_t *p_object)
{
    if (!hal_adc_is_object_valid(p_object)) {
        return hal_adc_err_param;
    }

    if (!plat_ch32v203_adc_ops->pf_cmd(p_object->base_address, false)) {
        return hal_adc_err_param;
    }

    p_object->state = hal_adc_state_disabled;
    return hal_adc_ok;
}

/**
 * @brief 启动规则通道转换（软件触发）。
 * @param p_object 模块对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定。
 * @retval hal_adc_err_state 平台层拒绝。
 */
hal_adc_status_enum hal_adc_start(hal_adc_t *p_object)
{
    if (!hal_adc_is_object_valid(p_object)) {
        return hal_adc_err_param;
    }

    if (!plat_ch32v203_adc_ops->pf_software_start(p_object->base_address, true)) {
        return hal_adc_err_state;
    }

    p_object->state = hal_adc_state_enabled;
    return hal_adc_ok;
}

/**
 * @brief 装配循环 DMA 并启动规则通道连续转换。
 * @details DMA 通道号取自平台 DMA 层的 pf_channel_get（ADC1 规则通道），
 *          外设地址取自平台 ADC 层的 pf_get_data_register_address；
 *          通道装配（DeInit/Init/Cmd）全部经 plat_ch32v203_dma_ops 完成，
 *          本层只负责"外设侧"的 DMA 请求使能与软件启动。
 * @param p_object 模块对象指针。
 * @param p_buffer 目标缓冲区。
 * @param buffer_size 缓冲区长度（按半字计）。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 * @retval hal_adc_err_state 平台层拒绝（无 DMA 通道映射 / 外设地址不可得 / 平台调用失败）。
 */
hal_adc_status_enum hal_adc_start_dma(hal_adc_t *p_object,
                                      void *p_buffer,
                                      uint16_t buffer_size)
{
    platform_ch32v203_dma_config_t dma_config;
    uintptr_t data_register_address;
    uint8_t dma_channel;

    if ((!hal_adc_is_object_valid(p_object)) || (p_buffer == NULL) ||
        (buffer_size == 0U)) {
        return hal_adc_err_param;
    }

    /* 通道号由平台 DMA 层统一给出（本层不关心具体是哪条通道） */
    dma_channel = plat_ch32v203_dma_ops->pf_channel_get(
        (uint8_t)platform_ch32v203_dma_peripheral_adc1, false);
    if (dma_channel == (uint8_t)platform_ch32v203_dma_channel_none) {
        return hal_adc_err_state;
    }

    /* 外设数据寄存器地址由平台 ADC 层给出（本层不得书写寄存器符号） */
    data_register_address =
        plat_ch32v203_adc_ops->pf_get_data_register_address(
            p_object->base_address);
    if (data_register_address == 0U) {
        return hal_adc_err_state;
    }

    /* ADC 规则通道 → 内存：半字宽度、存储器自增、循环、中优先级（等价于改造前） */
    dma_config.peripheral_address = data_register_address;
    dma_config.memory_address = (uintptr_t)p_buffer;
    dma_config.direction = PLAT_DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_config.buffer_size = buffer_size;
    dma_config.peripheral_increment = PLAT_DMA_PERIPHERAL_INCREMENT_DISABLE;
    dma_config.memory_increment = PLAT_DMA_MEMORY_INCREMENT_ENABLE;
    dma_config.peripheral_data_size = PLAT_DMA_PERIPHERAL_DATA_SIZE_HALFWORD;
    dma_config.memory_data_size = PLAT_DMA_MEMORY_DATA_SIZE_HALFWORD;
    dma_config.mode = PLAT_DMA_MODE_CIRCULAR;
    dma_config.priority = PLAT_DMA_PRIORITY_MEDIUM;
    dma_config.memory_to_memory = PLAT_DMA_MEMORY_TO_MEMORY_DISABLE;

    if ((!plat_ch32v203_dma_ops->pf_clock_enable()) ||
        (!plat_ch32v203_dma_ops->pf_deinit(dma_channel)) ||
        (!plat_ch32v203_dma_ops->pf_config(dma_channel, &dma_config)) ||
        (!plat_ch32v203_dma_ops->pf_cmd(dma_channel, true)) ||
        (!plat_ch32v203_adc_ops->pf_dma_request_cmd(p_object->base_address,
                                                    true)) ||
        (!plat_ch32v203_adc_ops->pf_software_start(p_object->base_address,
                                                   true))) {
        return hal_adc_err_state;
    }

    p_object->state = hal_adc_state_enabled;
    return hal_adc_ok;
}

/**
 * @brief 读取规则通道数据寄存器中的最新转换值。
 * @param p_object 模块对象指针。
 * @param p_value 输出数据指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 */
hal_adc_status_enum hal_adc_get_value(hal_adc_t *p_object, uint16_t *p_value)
{
    if ((!hal_adc_is_object_valid(p_object)) || (p_value == NULL)) {
        return hal_adc_err_param;
    }

    *p_value = plat_ch32v203_adc_ops->pf_get_value(p_object->base_address);
    return hal_adc_ok;
}

/**
 * @brief 读取注入通道转换值。
 * @param p_object 模块对象指针。
 * @param rank 注入序号，从 1 开始。
 * @param p_value 输出数据指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效或序号越界。
 */
hal_adc_status_enum hal_adc_get_injected_value(hal_adc_t *p_object,
                                               uint8_t rank,
                                               uint16_t *p_value)
{
    if ((!hal_adc_is_object_valid(p_object)) || (p_value == NULL) ||
        (rank == 0U) || (rank > HAL_ADC_INJECTED_CHANNEL_COUNT)) {
        return hal_adc_err_param;
    }

    *p_value = plat_ch32v203_adc_ops->pf_get_injected_value(p_object->base_address,rank);
    return hal_adc_ok;
}

/**
 * @brief 使能/禁止注入组转换完成中断源（JEOC）。
 * @warning 使能前必须已实现 ADC1_2_IRQHandler：本工程启动文件对该向量只提供
 *          `.weak` 默认处理函数（内容为死循环），未实现就打开中断会直接跑飞。
 * @param p_object 模块对象指针。
 * @param enable true 使能 JEOC 中断源，false 关闭。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定。
 * @retval hal_adc_err_state 平台层拒绝（实例不受支持）。
 */
hal_adc_status_enum hal_adc_enable_injected_irq(hal_adc_t *p_object,
                                                bool enable)
{
    if (!hal_adc_is_object_valid(p_object)) {
        return hal_adc_err_param;
    }

    if (!plat_ch32v203_adc_ops->pf_it_config(p_object->base_address,
                                             PLAT_ADC_IT_JEOC, enable)) {
        return hal_adc_err_state;
    }

    return hal_adc_ok;
}

/**
 * @brief 查询 ADC 状态/中断标志位（JEOC / EOC）。
 * @details 封装平台层的标志查询接口（pf_get_flag），供中断服务函数判断是否本外设
 *          事件；用法见 hal_adc.h 中 hal_adc_get_flag 的说明。
 * @param p_object 模块对象指针。
 * @param flag 标志位，取 PLAT_ADC_FLAG_x（如 PLAT_ADC_FLAG_JEOC）。
 * @param p_is_set 输出：true = 置位，false = 未置位。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 对象未绑定或输出指针为空。
 */
hal_adc_status_enum hal_adc_get_flag(const hal_adc_t *p_object,
                                     uint8_t flag,
                                     bool *p_is_set)
{
    if ((!hal_adc_is_object_valid(p_object)) || (p_is_set == NULL)) {
        return hal_adc_err_param;
    }

    *p_is_set = plat_ch32v203_adc_ops->pf_get_flag(p_object->base_address,
                                                   flag);

    return hal_adc_ok;
}

/**
 * @brief 返回ADC对象当前状态。
 * @param p_object 模块对象指针。
 * @return 返回当前对象状态或接口执行结果。
 */
hal_adc_state_enum hal_adc_get_state(const hal_adc_t *p_object)
{
    if (p_object == NULL) {
        return hal_adc_state_error;
    }
    return p_object->state;
}

/* ==========================================================================
 * 板级构建
 * ========================================================================== */

/**
 * @brief 构建 ADC1 规则通道实例（引脚组 + ADCCLK 分频 + 配置 + 通道序列 + DMA 启动）。
 * @details 板级前置顺序：PA0/PA1 模拟输入引脚组 → ADCCLK 分频（ADC1/ADC2 共用，
 *          须在 ADC 使能/校准之前设置）→ 绑定/初始化 → 通道序列 → DMA 循环启动。
 * @param p_object ADC 对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 * @retval hal_adc_err_state 引脚组构建或平台层配置失败。
 */
static hal_adc_status_enum hal_adc_build_reguler(hal_adc_t *p_object)
{
    hal_adc_status_enum status;
    uint8_t index;

    /* 板级前置：PA0/PA1 模拟输入引脚组（经 HAL GPIO 函数表构建） */
    if (hal_gpio_ops->pf_build(&hal_gpio_adc_reguler,
                               hal_gpio_adc_reguler_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_adc_reguler_pin_table))
        != hal_gpio_ok) {
        return hal_adc_err_state;
    }

    /* 板级前置：ADCCLK = PCLK2/8（ADC1/ADC2 共用同一分频器，须在 ADC 使能/校准之前设置）
     * PCLK2 = 72MHz 时 ADCCLK = 9MHz，满足数据手册 fADC ≤ 14MHz 的要求。 */
    PLAT_RCC_ADC_CLOCK_CONFIG(PLAT_RCC_ADC_CLOCK_DIV8);

    status = hal_adc_bind(p_object, (uintptr_t)PLAT_PERIPH_ADC1,
                          &s_adc_reguler_config);
    if (status != hal_adc_ok) {
        return status;
    }

    status = hal_adc_init(p_object);
    if (status != hal_adc_ok) {
        return status;
    }

    for (index = 0U; index < HAL_ADC_REGULER_CHANNEL_COUNT; index++) {
        if (!plat_ch32v203_adc_ops->pf_regular_channel_config(
                p_object->base_address, s_adc_reguler_channels[index].channel,
                s_adc_reguler_channels[index].rank,
                s_adc_reguler_channels[index].sample_time)) {
            return hal_adc_err_state;
        }
    }

    return hal_adc_start_dma(p_object,
                             (void *)hal_adc_reguler_dma_value,
                             HAL_ADC_REGULER_CHANNEL_COUNT);
}

/**
 * @brief 构建 ADC2 注入通道实例（ADCCLK 分频 + 配置 + 注入序列 + TIM1_TRGO 触发 + 中断线）。
 * @details ADCCLK 分频由 ADC1/ADC2 共用，此处与规则实例构建时取同一分频值（幂等），
 *          以保证"只构建注入实例"时 ADC 时钟不会停留在复位值（PCLK2/2）。
 * @param p_object ADC 对象指针。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效。
 * @retval hal_adc_err_state 平台层配置失败。
 */
static hal_adc_status_enum hal_adc_build_injected(hal_adc_t *p_object)
{
    hal_adc_status_enum status;
    uint8_t index;

    /* 板级前置：ADCCLK = PCLK2/8（与规则实例构建取同一分频值，幂等；
     * 须在 ADC 使能/校准之前设置） */
    // PLAT_RCC_ADC_CLOCK_CONFIG(PLAT_RCC_ADC_CLOCK_DIV8);

    // 在opamp外设初始化函数中已经进行引脚配置，在BSP层初始化时会进行调用

    /* 注入通道采样运放输出（PA4/PB1），其引脚由运放引脚组配置 */
    status = hal_adc_bind(p_object, (uintptr_t)PLAT_PERIPH_ADC2,
                          &s_adc_injected_config);
    if (status != hal_adc_ok) {
        return status;
    }

    status = hal_adc_init(p_object);
    if (status != hal_adc_ok) {
        return status;
    }

    if (!plat_ch32v203_adc_ops->pf_injected_sequencer_length(
            p_object->base_address, HAL_ADC_INJECTED_CHANNEL_COUNT)) {
        return hal_adc_err_state;
    }

    for (index = 0U; index < HAL_ADC_INJECTED_CHANNEL_COUNT; index++) {
        if (!plat_ch32v203_adc_ops->pf_injected_channel_config(
                p_object->base_address, s_adc_injected_channels[index].channel,
                s_adc_injected_channels[index].rank,
                s_adc_injected_channels[index].sample_time)) {
            return hal_adc_err_state;
        }
    }

    if (!plat_ch32v203_adc_ops->pf_external_trig_injected(
            p_object->base_address, PLAT_ADC_EXTERNAL_TRIG_INJEC_T1_TRGO,
            true)) {
        return hal_adc_err_state;
    }

    /* 注入组中断分两层处理：
     * - 中断线（ADC1/ADC2 共用 PLAT_ADC_IRQ_ADC1_2）：构建时即配置并使能，
     *   优先级取本文件板级常量；此时中断源未打开，不会触发中断。
     * - 中断源（JEOC，注入转换结束）：默认不使能，业务层实现 ADC1_2_IRQHandler
     *   之后经 hal_adc_ops->pf_enable_injected_irq() 按需打开
     *   （本工程启动文件中 ADC1_2 等向量均为 .weak 的 `1: j 1b` 死循环）。 */
    if (!plat_ch32v203_adc_ops->pf_irq_enable(
            PLAT_ADC_IRQ_ADC1_2, HAL_ADC_INJECTED_IRQ_PREEMPTION_PRIORITY,
            HAL_ADC_INJECTED_IRQ_SUB_PRIORITY)) {
        return hal_adc_err_state;
    }

    return hal_adc_ok;
}

/**
 * @brief 构建板级 ADC 实例。
 * @param p_object ADC 对象指针。
 * @param role 板级用途，取 hal_adc_role_x。
 * @retval hal_adc_ok 操作成功。
 * @retval hal_adc_err_param 参数无效或用途不支持。
 * @retval hal_adc_err_state 引脚组构建或平台层配置失败。
 */
hal_adc_status_enum hal_adc_build(hal_adc_t *p_object, hal_adc_role_enum role)
{
    if (p_object == NULL) {
        return hal_adc_err_param;
    }

    if (role == hal_adc_role_reguler) {
        return hal_adc_build_reguler(p_object);
    }
    if (role == hal_adc_role_injected) {
        return hal_adc_build_injected(p_object);
    }

    return hal_adc_err_param;
}

/* ==========================================================================
 * HAL 层 ADC 函数表
 * ========================================================================== */

/** @brief HAL 层 ADC 函数表（本文件私有，绑定本层对外接口）。 */
static const hal_adc_ops_t s_hal_adc_ops = {
    .pf_build = hal_adc_build,
    .pf_get_injected_value = hal_adc_get_injected_value,
    .pf_enable_injected_irq = hal_adc_enable_injected_irq,
    .pf_get_flag = hal_adc_get_flag,
    /* 备用接口：与同名 HAL 函数一一对应，供外部按需接入 */
    .pf_get_value = hal_adc_get_value,  // 规则组非连续模式单次读取会用到    
    .pf_start = hal_adc_start,
    .pf_deinit = hal_adc_deinit,
    .pf_enable = hal_adc_enable,
    .pf_disable = hal_adc_disable,
    .pf_get_state = hal_adc_get_state    
};

/** @brief HAL 层 ADC 函数表指针（对外外露，声明于 hal_adc.h）。 */
const hal_adc_ops_t *const hal_adc_ops = &s_hal_adc_ops;
