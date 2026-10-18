/**
 * @file bsp_analog_sense.h
 * @brief 板级模拟量感知 BSP 接口（母线电压 / 控制电位器 / NTC 温度）。
 * @details 结构与本层 bsp_encoder / bsp_motor_ctrl 同构：对象只保存状态，接口 + 一张 BSP 函数表：
 *          - bsp_analog_sense_init()：构建板级 ADC 规则组（PA0/PA1/PA2 + 循环 DMA）；
 *          - bsp_analog_sense_update()：一次刷新，从 DMA 采样快照换算三路物理量；
 *          - get_raw / get_bus_voltage / get_potentiometer / get_temperature：读取结果。
 *          三路模拟量共用**同一次 ADC1 规则组扫描 + 同一条 DMA**，因此必须由本模块独占
 *          hal_adc_reguler 并对三路提供统一视图（避免多个模块重复构建同一外设）。
 *          换算一律在**整数域**完成（NTC 走查表插值，不引 libm），可在任意上下文调用。
 *          硬件访问一律经 HAL ADC 函数表指针 hal_adc_ops。
 *          接口经 bsp_analog_sense_ops 函数表对外外露（状态查询为普通访问函数）。
 */
#ifndef __BSP_ANALOG_SENSE_H__
#define __BSP_ANALOG_SENSE_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 模拟量感知对象的生命周期状态。 */
typedef enum {
    bsp_analog_sense_state_reset = 0,
    bsp_analog_sense_state_ready,   /**< 已构建 ADC，尚未刷新 */
    bsp_analog_sense_state_running, /**< 已在正常刷新 */
    bsp_analog_sense_state_error
} bsp_analog_sense_state_enum;

/** @brief 模拟量感知接口调用返回的状态码。 */
typedef enum {
    bsp_analog_sense_ok = 0,
    bsp_analog_sense_err_param = -1,
    bsp_analog_sense_err_state = -2,
    bsp_analog_sense_err_unsupported = -3,
    bsp_analog_sense_err_timeout = -4
} bsp_analog_sense_status_enum;

/**
 * @brief 模拟量通道（顺序与 ADC 规则组的 DMA 缓冲一一对应）。
 * @note 通道序号即 hal_adc_reguler_dma_value[] 的下标，由本模块内部保证一致
 *       （编译期断言校验），上层无需知道 DMA 细节。
 */
typedef enum {
    bsp_analog_sense_channel_potentiometer = 0, /**< PA0：控制电位器 */
    bsp_analog_sense_channel_bus_voltage,      /**< PA1：母线电压 */
    bsp_analog_sense_channel_ntc,               /**< PA2：NTC 温度 */
    bsp_analog_sense_channel_count
} bsp_analog_sense_channel_enum;

/**
 * @brief 板级模拟量换算配置。
 * @details 只放"板级电气事实"，不含控制策略：
 *          - adc_reference_mv：ADC 参考电压（mV）；
 *          - adc_full_scale：ADC 满量程计数（12 位为 4095）；
 *          - bus_voltage_divider_num/den：母线分压比（母线端 : ADC 端 = num : den）。
 */
typedef struct {
    uint16_t adc_reference_mv;          /**< 参考电压（mV），如 3300 */
    uint16_t adc_full_scale;            /**< 满量程计数，如 4095 */
    uint16_t bus_voltage_divider_num;   /**< 分压比分子（母线端） */
    uint16_t bus_voltage_divider_den;   /**< 分压比分母（ADC 端） */
} bsp_analog_sense_config_t;

/**
 * @brief 模拟量感知状态对象：生命周期状态 + 配置 + 最近一次刷新的原始值与物理量。
 */
typedef struct {
    bsp_analog_sense_state_enum state;   /**< 对象生命周期状态 */
    bsp_analog_sense_config_t config;    /**< 板级换算配置 */

    uint16_t raw_value[bsp_analog_sense_channel_count]; /**< 各路 ADC 原始计数 */
    uint16_t potentiometer_permille;     /**< 电位器行程（千分比 0…1000） */
    uint32_t bus_voltage_mv;             /**< 母线电压（mV） */    
    int16_t ntc_temperature_dc;          /**< NTC 温度（0.1 ℃） */
} bsp_analog_sense_t;

/** @brief 板级模拟量感知对象（单实例）。 */
extern bsp_analog_sense_t bsp_analog_sense_board;

/**
 * @brief 初始化模拟量感知：构建板级 ADC 规则组（引脚组 + 通道序列 + 循环 DMA 启动）。
 * @param p_object 模拟量感知对象指针。
 * @retval bsp_analog_sense_ok 操作成功。
 * @retval bsp_analog_sense_err_param 参数无效。
 * @retval bsp_analog_sense_err_state ADC 构建失败。
 */
bsp_analog_sense_status_enum bsp_analog_sense_init(bsp_analog_sense_t *p_object);

/**
 * @brief 刷新一次：读取 DMA 采样快照并换算三路物理量。
 * @details 纯内存读取 + 整数换算，无阻塞、可在控制周期或中断中调用。
 * @param p_object 模拟量感知对象指针。
 * @retval bsp_analog_sense_ok 操作成功。
 * @retval bsp_analog_sense_err_param 参数无效或对象未就绪。
 */
bsp_analog_sense_status_enum bsp_analog_sense_update(bsp_analog_sense_t *p_object);

/**
 * @brief 取某一路的 ADC 原始计数。
 * @param p_object 模拟量感知对象指针。
 * @param channel 通道号，取 bsp_analog_sense_channel_x。
 * @return 原始计数；参数非法时返回 0。
 */
uint16_t bsp_analog_sense_get_raw(const bsp_analog_sense_t *p_object,
                                  uint8_t channel);

/**
 * @brief 取母线电压（mV）。
 * @param p_object 模拟量感知对象指针。
 * @return 母线电压（mV）；参数为空时返回 0。
 */
uint32_t bsp_analog_sense_get_bus_voltage(const bsp_analog_sense_t *p_object);

/**
 * @brief 取电位器行程（千分比 0…1000）。
 * @note 行程→目标转速/转矩的映射属控制策略，由 service/app 负责。
 * @param p_object 模拟量感知对象指针。
 * @return 行程千分比；参数为空时返回 0。
 */
uint16_t bsp_analog_sense_get_potentiometer(const bsp_analog_sense_t *p_object);

/**
 * @brief 取 NTC 温度（0.1 ℃）。
 * @note 过温保护策略由 service/app 负责，本层只给温度值。
 * @param p_object 模拟量感知对象指针。
 * @return 温度（0.1 ℃）；参数为空时返回 0。
 */
int16_t bsp_analog_sense_get_temperature(const bsp_analog_sense_t *p_object);

/**
 * @brief 查询模拟量感知对象当前状态。
 * @param p_object 模拟量感知对象指针。
 * @return 当前对象状态；参数为空时返回 bsp_analog_sense_state_error。
 */
bsp_analog_sense_state_enum bsp_analog_sense_get_state(
    const bsp_analog_sense_t *p_object);

/**
 * @brief BSP 模拟量感知函数表：初始化 / 刷新 / 三路物理量读取。
 * @details 表中的函数指针在 bsp_analog_sense.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 bsp_analog_sense_ops 指针统一调用。
 */
typedef struct {
    /** @brief 初始化（见 bsp_analog_sense_init）。 */
    bsp_analog_sense_status_enum (*pf_init)(bsp_analog_sense_t *p_object);
    /** @brief 刷新一次（见 bsp_analog_sense_update）。 */
    bsp_analog_sense_status_enum (*pf_update)(bsp_analog_sense_t *p_object);
    /** @brief 取母线电压 mV（见 bsp_analog_sense_get_bus_voltage）。 */
    uint32_t (*pf_get_bus_voltage)(const bsp_analog_sense_t *p_object);
    /** @brief 取电位器行程千分比（见 bsp_analog_sense_get_potentiometer）。 */
    uint16_t (*pf_get_potentiometer)(const bsp_analog_sense_t *p_object);
    /** @brief 取 NTC 温度 0.1 ℃（见 bsp_analog_sense_get_temperature）。 */
    int16_t (*pf_get_temperature)(const bsp_analog_sense_t *p_object);
} bsp_analog_sense_ops_t;

/** @brief BSP 模拟量感知函数表指针（在 bsp_analog_sense.c 中定义并绑定）。 */
extern const bsp_analog_sense_ops_t *const bsp_analog_sense_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ANALOG_SENSE_H__ */
