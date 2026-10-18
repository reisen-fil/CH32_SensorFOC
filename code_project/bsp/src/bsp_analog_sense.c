/**
 * @file bsp_analog_sense.c
 * @brief 板级模拟量感知 BSP 实现（母线电压 / 控制电位器 / NTC 温度）。
 * @details 结构与本层 bsp_encoder.c / bsp_motor_ctrl.c 同构：
 *          - bsp_analog_sense_init() 经 HAL ADC 函数表 hal_adc_ops->pf_build 构建
 *            ADC1 规则组（PA0 电位器、PA1 母线电压、PA2 NTC + 循环 DMA）；
 *          - bsp_analog_sense_update() 读取 DMA 采样快照，在整数域换算三路物理量
 *            （NTC 走查表 + 线性插值，不引 libm，可安全用于任意上下文）。
 *          本模块独占 hal_adc_reguler：三路模拟量来自同一次扫描与同一条 DMA，
 *          由本模块统一对外提供，其它模块不得重复构建该实例。
 */

/* 
    待测项：
    1、NTC查表读数实测（由于分压电路参考电源给得有问题暂时搁置）    
    2、ADC规则组循环模式读取实测（完成）
    3、母线电压采样与电位器读取验证（完成）
*/
 
#include "bsp_analog_sense.h"
#include <stddef.h>

/* ==========================================================================
 * 板级常量与配置
 * ========================================================================== */

/**
 * @brief 母线分压比（母线端 : ADC 端）。
 * @note 示例按 1:1（未分压）填写；请按实际电阻分压填写，例如 11:1 表示母线端
 *       11 V 对应 ADC 端 1 V（1:1 时读到的就是 ADC 端电压）。
 */
#define BSP_ANALOG_SENSE_BUS_DIVIDER_NUM   (57U)
#define BSP_ANALOG_SENSE_BUS_DIVIDER_DEN   (10U)

/** @brief ADC 参考电压（mV）与满量程计数（12 位）。 */
#define BSP_ANALOG_SENSE_ADC_VREF_MV       (3300U)
#define BSP_ANALOG_SENSE_ADC_FULL_SCALE    (4095U)

/** @brief 电位器行程满量程（千分比）。 */
#define BSP_ANALOG_SENSE_PERMILLE_FULL     (1000U)

/**
 * @brief NTC 温度—原始值对照表（温度升序、原始值降序）。
 * @note 表内数据按"典型 10 kΩ / B=3950 NTC + 10 kΩ 上拉至 3.3 V、NTC 接地、12 位 ADC"生成；
 *       请按实际 NTC 参数与上拉电阻重新生成后再使用。
 *       重新生成公式：R(T)=R25·exp(B·(1/T−1/298.15))，raw = 4095·R/(R+R_pull)。
 */
static const struct {
    int16_t temperature_dc;   /**< 温度（0.1 ℃） */
    uint16_t raw_value;       /**< 对应 ADC 原始计数 */
} s_ntc_table[] = {
    {  -200, 3740 },   /* 105.4 kΩ */
    {  -100, 3495 },   /*  58.2 kΩ */
    {     0, 3156 },   /*  33.6 kΩ */
    {   100, 2738 },   /*  20.2 kΩ */
    {   200, 2278 },   /*  12.5 kΩ */
    {   300, 1825 },   /*   8.0 kΩ */
    {   400, 1419 },   /*   5.3 kΩ */
    {   500, 1081 },   /*   3.6 kΩ */
    {   600,  815 },   /*   2.5 kΩ */
    {   700,  613 },   /*   1.8 kΩ */
    {   800,  462 },   /*   1.3 kΩ */
    {   900,  350 },   /*   0.9 kΩ */
    {  1000,  267 },   /*   0.7 kΩ */
    {  1100,  206 },   /*   0.5 kΩ */
    {  1200,  160 }    /*   0.4 kΩ */
};

/** @brief 板级模拟量换算配置。 */
static const bsp_analog_sense_config_t s_analog_sense_config = {
    .adc_reference_mv = BSP_ANALOG_SENSE_ADC_VREF_MV,
    .adc_full_scale = BSP_ANALOG_SENSE_ADC_FULL_SCALE,
    .bus_voltage_divider_num = BSP_ANALOG_SENSE_BUS_DIVIDER_NUM,
    .bus_voltage_divider_den = BSP_ANALOG_SENSE_BUS_DIVIDER_DEN
};

/** @brief 板级模拟量感知对象（单实例，只保存配置与最近一次刷新的结果）。 */
bsp_analog_sense_t bsp_analog_sense_board = {0};

/**
 * @brief 编译期校验：通道枚举顺序必须与 ADC 规则组的 DMA 缓冲长度一致。
 * @note 规则组通道顺序（PA0/PA1/PA2）由 hal_adc.c 的通道表决定，本层按下标取值。
 */
typedef char bsp_analog_sense_channel_check
    [(bsp_analog_sense_channel_count == HAL_ADC_REGULER_CHANNEL_COUNT) ? 1 : -1];

/* ==========================================================================
 * 内部换算（整数域，不引 libm）
 * ========================================================================== */

/**
 * @brief 原始计数 → ADC 端电压（mV）。
 * @param p_config 板级配置。
 * @param raw ADC 原始计数。
 * @return ADC 端电压（mV）。
 */
static uint32_t bsp_analog_sense_raw_to_mv(
    const bsp_analog_sense_config_t *p_config,
    uint16_t raw)
{
    return ((uint32_t)raw * (uint32_t)p_config->adc_reference_mv) /
           (uint32_t)p_config->adc_full_scale;
}

/**
 * @brief NTC 原始计数 → 温度（0.1 ℃），查表 + 线性插值，超范围时夹到端点。
 * @param raw NTC 通道原始计数。
 * @return 温度（0.1 ℃）。
 */
static int16_t bsp_analog_sense_ntc_to_temperature(uint16_t raw)
{
    uint32_t index;
    uint32_t table_size = (uint32_t)(sizeof(s_ntc_table) / sizeof(s_ntc_table[0]));

    for (index = 0U; (index + 1U) < table_size; index++) {
        uint16_t raw_low_temp = s_ntc_table[index].raw_value;
        uint16_t raw_high_temp = s_ntc_table[index + 1U].raw_value;

        /* 表中温度升序、原始值降序：找到跨过 raw 的相邻两点 */
        if ((raw <= raw_low_temp) && (raw >= raw_high_temp)) {
            int32_t temp_span = (int32_t)s_ntc_table[index + 1U].temperature_dc -
                                (int32_t)s_ntc_table[index].temperature_dc;
            int32_t raw_span = (int32_t)raw_low_temp - (int32_t)raw_high_temp;
            int32_t raw_offset = (int32_t)raw_low_temp - (int32_t)raw;

            return (int16_t)((int32_t)s_ntc_table[index].temperature_dc +
                             (temp_span * raw_offset) / raw_span);
        }
    }

    /* 超出表范围：低于最低温或高于最高温，夹到端点 */
    if (raw > s_ntc_table[0].raw_value) {
        return s_ntc_table[0].temperature_dc;
    }

    return s_ntc_table[table_size - 1U].temperature_dc;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

/**
 * @brief 初始化模拟量感知：构建板级 ADC 规则组。
 * @param p_object 模拟量感知对象指针。
 * @retval bsp_analog_sense_ok 操作成功。
 * @retval bsp_analog_sense_err_param 参数无效。
 * @retval bsp_analog_sense_err_state ADC 构建失败。
 */
bsp_analog_sense_status_enum bsp_analog_sense_init(bsp_analog_sense_t *p_object)
{
    uint32_t index;

    if (p_object == NULL) {
        return bsp_analog_sense_err_param;
    }

    /* 板级 ADC1 规则组：PA0/PA1/PA2 引脚组 + 通道序列 + 循环 DMA——经 HAL ADC 函数表构建 */
    if (hal_adc_ops->pf_build(&hal_adc_reguler, hal_adc_role_reguler) != hal_adc_ok) {
        p_object->state = bsp_analog_sense_state_error;
        return bsp_analog_sense_err_state;
    }

    p_object->config = s_analog_sense_config;
    for (index = 0U; index < (uint32_t)bsp_analog_sense_channel_count; index++) {
        p_object->raw_value[index] = 0U;
    }
    p_object->bus_voltage_mv = 0U;
    p_object->potentiometer_permille = 0U;
    p_object->ntc_temperature_dc = 0;
    p_object->state = bsp_analog_sense_state_ready;

    return bsp_analog_sense_ok;
}

/**
 * @brief 刷新一次：读取 DMA 采样快照并换算三路物理量。
 * @param p_object 模拟量感知对象指针。
 * @retval bsp_analog_sense_ok 操作成功。
 * @retval bsp_analog_sense_err_param 参数无效或对象未就绪。
 */
bsp_analog_sense_status_enum bsp_analog_sense_update(bsp_analog_sense_t *p_object)
{
    uint32_t index;
    uint32_t adc_pin_mv;
    uint16_t raw_ntc;

    /* 就绪判断直接内联在此处，不再单独封装 */
    if ((p_object == NULL) ||
        ((p_object->state != bsp_analog_sense_state_ready) &&
         (p_object->state != bsp_analog_sense_state_running))) {
        return bsp_analog_sense_err_param;
    }

    /* 取 DMA 采样快照（下标顺序与通道枚举一致，已由编译期断言校验） */
    for (index = 0U; index < (uint32_t)bsp_analog_sense_channel_count; index++) {
        p_object->raw_value[index] = hal_adc_reguler_dma_value[index];
    }

    /* 电位器行程：千分比 */
    p_object->potentiometer_permille = (uint16_t)(
        ((uint32_t)p_object->raw_value[bsp_analog_sense_channel_potentiometer] *
         BSP_ANALOG_SENSE_PERMILLE_FULL) /
        (uint32_t)p_object->config.adc_full_scale);

    /* 母线电压：ADC 端电压 × 分压比（整数域，先除满量程再乘分压比避免溢出） */
    adc_pin_mv = bsp_analog_sense_raw_to_mv(
        &p_object->config, p_object->raw_value[bsp_analog_sense_channel_bus_voltage]);
    p_object->bus_voltage_mv =
        (adc_pin_mv * (uint32_t)p_object->config.bus_voltage_divider_num) /
        (uint32_t)p_object->config.bus_voltage_divider_den;

    /* NTC 温度：查表 + 线性插值 */
    // raw_ntc = p_object->raw_value[bsp_analog_sense_channel_ntc];
    // p_object->ntc_temperature_dc = bsp_analog_sense_ntc_to_temperature(raw_ntc);

    p_object->state = bsp_analog_sense_state_running;
    return bsp_analog_sense_ok;
}

/**
 * @brief 取某一路的 ADC 原始计数。
 * @param p_object 模拟量感知对象指针。
 * @param channel 通道号，取 bsp_analog_sense_channel_x。
 * @return 原始计数；参数非法时返回 0。
 */
uint16_t bsp_analog_sense_get_raw(const bsp_analog_sense_t *p_object,
                                  uint8_t channel)
{
    if ((p_object == NULL) || (channel >= (uint8_t)bsp_analog_sense_channel_count)) {
        return 0U;
    }

    return p_object->raw_value[channel];
}

/**
 * @brief 取母线电压（mV）。
 * @param p_object 模拟量感知对象指针。
 * @return 母线电压（mV）；参数为空时返回 0。
 */
uint32_t bsp_analog_sense_get_bus_voltage(const bsp_analog_sense_t *p_object)
{
    if (p_object == NULL) {
        return 0U;
    }

    return p_object->bus_voltage_mv;
}

/**
 * @brief 取电位器行程（千分比 0…1000）。
 * @param p_object 模拟量感知对象指针。
 * @return 行程千分比；参数为空时返回 0。
 */
uint16_t bsp_analog_sense_get_potentiometer(const bsp_analog_sense_t *p_object)
{
    if (p_object == NULL) {
        return 0U;
    }

    return p_object->potentiometer_permille;
}

/**
 * @brief 取 NTC 温度（0.1 ℃）。
 * @param p_object 模拟量感知对象指针。
 * @return 温度（0.1 ℃）；参数为空时返回 0。
 */
int16_t bsp_analog_sense_get_temperature(const bsp_analog_sense_t *p_object)
{
    if (p_object == NULL) {
        return 0;
    }

    return p_object->ntc_temperature_dc;
}

/**
 * @brief 查询模拟量感知对象当前状态。
 * @param p_object 模拟量感知对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_analog_sense_state_error。
 */
bsp_analog_sense_state_enum bsp_analog_sense_get_state(
    const bsp_analog_sense_t *p_object)
{
    if (p_object == NULL) {
        return bsp_analog_sense_state_error;
    }

    return p_object->state;
}

/* ==========================================================================
 * BSP 模拟量感知函数表
 * ========================================================================== */

/** @brief BSP 模拟量感知函数表（本文件私有，绑定板级接口）。 */
static const bsp_analog_sense_ops_t s_bsp_analog_sense_ops = {
    .pf_init = bsp_analog_sense_init,
    .pf_update = bsp_analog_sense_update,
    .pf_get_bus_voltage = bsp_analog_sense_get_bus_voltage,
    .pf_get_potentiometer = bsp_analog_sense_get_potentiometer,
    .pf_get_temperature = bsp_analog_sense_get_temperature
};

/** @brief BSP 模拟量感知函数表指针（对外外露，声明于 bsp_analog_sense.h）。 */
const bsp_analog_sense_ops_t *const bsp_analog_sense_ops = &s_bsp_analog_sense_ops;
