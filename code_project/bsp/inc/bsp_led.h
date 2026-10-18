/**
 * @file bsp_led.h
 * @brief LED 状态显示 BSP 接口。
 * @details 本层只做三个简单封装：
 *          - bsp_led_init()：构建板级 LED 引脚组并默认按"状态正常"显示；
 *          - bsp_set_green_led() / bsp_set_red_led()：状态正常 / 异常显示。
 *          三个接口内部一律经 HAL GPIO 函数表指针 hal_gpio_ops
 *          （pf_build / pf_write）访问硬件；
 *          对象只保存当前 LED 指示状态，不携带操作表、配置等额外信息；
 *          三个接口经 bsp_led_ops 函数表对外外露。
 */
#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief LED接口调用返回的状态码。 */
typedef enum {
    bsp_led_ok = 0,
    bsp_led_err_param = -1,
    bsp_led_err_state = -2,
    bsp_led_err_unsupported = -3,
    bsp_led_err_timeout = -4
} bsp_led_status_enum;

/** @brief LED 指示状态：状态正常 / 状态异常。 */
typedef enum {
    bsp_led_indicator_idle = 0,
    bsp_led_indicator_normal,   /**< 状态正常：绿灯亮、红灯灭 */
    bsp_led_indicator_abnormal      /**< 状态异常：红灯亮、绿灯灭 */
} bsp_led_indicator_enum;

/** @brief LED 状态对象：只保存当前指示状态。 */
typedef struct {
    bsp_led_indicator_enum indicator;   /**< 当前指示状态 */
} bsp_led_display_t;

/** @brief LED 状态显示对象（单实例，保存当前显示状态）。 */
extern bsp_led_display_t bsp_led_display;

/**
 * @brief 初始化 LED 状态显示：构建 LED 引脚组（PC14 绿 / PC15 红）并默认按"状态正常"显示。
 * @param p_object LED 状态显示对象指针。
 * @retval bsp_led_ok 操作成功。
 * @retval bsp_led_err_param 参数无效。
 * @retval bsp_led_err_state 引脚组构建或引脚写失败。
 */
bsp_led_status_enum bsp_led_init(bsp_led_display_t *p_object);

/**
 * @brief 状态正常显示：绿灯亮、红灯灭，并把 indicator 置为 normal。
 * @param p_object LED 状态显示对象指针。
 * @retval bsp_led_ok 操作成功。
 * @retval bsp_led_err_param 参数无效。
 * @retval bsp_led_err_state 引脚写失败。
 */
bsp_led_status_enum bsp_set_green_led(bsp_led_display_t *p_object);

/**
 * @brief 状态异常显示：红灯亮、绿灯灭，并把 indicator 置为 abnormal。
 * @param p_object LED 状态显示对象指针。
 * @retval bsp_led_ok 操作成功。
 * @retval bsp_led_err_param 参数无效。
 * @retval bsp_led_err_state 引脚写失败。
 */
bsp_led_status_enum bsp_set_red_led(bsp_led_display_t *p_object);

/**
 * @brief BSP LED 函数表：初始化 / 状态正常显示 / 状态异常显示。
 * @details 表中的函数指针在 bsp_led.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 bsp_led_ops 指针统一调用。
 */
typedef struct {
    /** @brief 初始化 LED 状态显示（见 bsp_led_init）。 */
    bsp_led_status_enum (*pf_init)(bsp_led_display_t *p_object);
    /** @brief 状态正常显示（见 bsp_set_green_led）。 */
    bsp_led_status_enum (*pf_set_green)(bsp_led_display_t *p_object);
    /** @brief 状态异常显示（见 bsp_set_red_led）。 */
    bsp_led_status_enum (*pf_set_red)(bsp_led_display_t *p_object);
} bsp_led_ops_t;

/** @brief BSP LED 函数表指针（在 bsp_led.c 中定义并绑定）。 */
extern const bsp_led_ops_t *const bsp_led_ops;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LED_H__ */
