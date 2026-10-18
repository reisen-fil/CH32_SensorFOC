/**
 * @file bsp_led.c
 * @brief LED 状态显示 BSP 实现（三个接口对 HAL GPIO 函数表的简单封装）。
 * @details bsp_led_init() 经 HAL GPIO 函数表指针 hal_gpio_ops 的 pf_build 构建
 *          板级 LED 引脚组；bsp_set_green_led() / bsp_set_red_led() 经 pf_write
 *          逐脚写 PC14（绿）/ PC15（红）。对象只保存当前指示状态。
 */

/*
    待测项：
    1、BSP层点灯测试（完成）

*/

#include "bsp_led.h"
#include <stddef.h>

/** @brief LED 状态显示对象（单实例，只保存当前指示状态）。 */
bsp_led_display_t bsp_led_display = {0};

/**
 * @brief 初始化 LED 状态显示：构建 LED 引脚组并默认按"状态正常"显示。
 * @param p_object LED 状态显示对象指针。
 * @retval bsp_led_ok 操作成功。
 * @retval bsp_led_err_param 参数无效。
 * @retval bsp_led_err_state 引脚组构建或引脚写失败。
 */
bsp_led_status_enum bsp_led_init(bsp_led_display_t *p_object)
{
    if (p_object == NULL) {
        return bsp_led_err_param;
    }

    /* 板级 LED 引脚组：PC14（绿）/ PC15（红）——经 HAL GPIO 函数表构建 */
    if (hal_gpio_ops->pf_build(&hal_gpio_led,
                               hal_gpio_led_pin_table,
                               HAL_GPIO_TABLE_COUNT(hal_gpio_led_pin_table))
        != hal_gpio_ok) {
        return bsp_led_err_state;
    }

    p_object->indicator = bsp_led_indicator_idle;

    /* 默认按"状态正常"显示 */
    // return bsp_set_green_led(p_object);
    return bsp_led_ok;
}

/**
 * @brief 状态正常显示：绿灯亮、红灯灭。
 * @param p_object LED 状态显示对象指针。
 * @retval bsp_led_ok 操作成功。
 * @retval bsp_led_err_param 参数无效。
 * @retval bsp_led_err_state 引脚写失败。
 */
bsp_led_status_enum bsp_set_green_led(bsp_led_display_t *p_object)
{
    if (p_object == NULL) {
        return bsp_led_err_param;
    }

    if ((hal_gpio_ops->pf_write(&hal_gpio_led, hal_gpio_led_pin_green, true)
         != hal_gpio_ok) ||
        (hal_gpio_ops->pf_write(&hal_gpio_led, hal_gpio_led_pin_red, false)
         != hal_gpio_ok)) {
        return bsp_led_err_state;
    }

    p_object->indicator = bsp_led_indicator_normal;
    return bsp_led_ok;
}

/**
 * @brief 状态异常显示：红灯亮、绿灯灭。
 * @param p_object LED 状态显示对象指针。
 * @retval bsp_led_ok 操作成功。
 * @retval bsp_led_err_param 参数无效。
 * @retval bsp_led_err_state 引脚写失败。
 */
bsp_led_status_enum bsp_set_red_led(bsp_led_display_t *p_object)
{
    if (p_object == NULL) {
        return bsp_led_err_param;
    }

    if ((hal_gpio_ops->pf_write(&hal_gpio_led, hal_gpio_led_pin_red, true)
         != hal_gpio_ok) ||
        (hal_gpio_ops->pf_write(&hal_gpio_led, hal_gpio_led_pin_green, false)
         != hal_gpio_ok)) {
        return bsp_led_err_state;
    }

    p_object->indicator = bsp_led_indicator_abnormal;
    return bsp_led_ok;
}

/* ==========================================================================
 * BSP LED 函数表
 * ========================================================================== */

/** @brief BSP LED 函数表（本文件私有，绑定板级三个接口）。 */
static const bsp_led_ops_t s_bsp_led_ops = {
    .pf_init = bsp_led_init,
    .pf_set_green = bsp_set_green_led,
    .pf_set_red = bsp_set_red_led
};

/** @brief BSP LED 函数表指针（对外外露，声明于 bsp_led.h）。 */
const bsp_led_ops_t *const bsp_led_ops = &s_bsp_led_ops;
