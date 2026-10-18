/**
 * @file app_foc_control.c
 * @brief 应用层FOC控制调度实现。
 * @details 该模块通过对象状态和操作表组织FOC功能；具体硬件访问由下层适配实现。
 */
#include "app_foc_control.h"
#include "bsp_current_sense.h"
#include "hal_gpio.h"

#include <stddef.h>

// static bool app_foc_control_is_object_valid(
//     const app_foc_control_t *p_object)
// {
//     return (p_object != NULL) && (p_object->p_vtable != NULL);
// }

// app_foc_control_status_enum app_foc_control_bind(
//     app_foc_control_t *p_object,
//     const app_foc_control_vtable_t *p_vtable,
//     const app_foc_control_config_t *p_config,
//     void *p_user_data)
// {
//     if ((p_object == NULL) || (p_vtable == NULL) || (p_config == NULL)) {
//         return app_foc_control_err_param;
//     }
//     p_object->p_vtable = p_vtable;
//     p_object->state = app_foc_control_state_reset;
//     p_object->config = *p_config;
//     p_object->p_foc_service = NULL;
//     p_object->p_state_machine_service = NULL;
//     p_object->p_communication_service = NULL;
//     p_object->p_user_data = p_user_data;
//     return app_foc_control_ok;
// }

// app_foc_control_status_enum app_foc_control_bind_dependencies(
//     app_foc_control_t *p_object,
//     svc_foc_t *p_foc_service,
//     svc_state_machine_t *p_state_machine_service,
//     svc_communication_t *p_communication_service)
// {
//     if (p_object == NULL) {
//         return app_foc_control_err_param;
//     }
//     p_object->p_foc_service = p_foc_service;
//     p_object->p_state_machine_service = p_state_machine_service;
//     p_object->p_communication_service = p_communication_service;
//     return app_foc_control_ok;
// }

// app_foc_control_status_enum app_foc_control_init(
//     app_foc_control_t *p_object)
// {
//     app_foc_control_status_enum status;
//     if ((!app_foc_control_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_init == NULL)) {
//         return app_foc_control_err_param;
//     }
//     status = p_object->p_vtable->pf_init(p_object);
//     p_object->state = (status == app_foc_control_ok) ?
//                       app_foc_control_state_ready :
//                       app_foc_control_state_error;
//     return status;
// }

// app_foc_control_status_enum app_foc_control_deinit(
//     app_foc_control_t *p_object)
// {
//     if ((!app_foc_control_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_deinit == NULL)) {
//         return app_foc_control_err_param;
//     }
//     p_object->state = app_foc_control_state_reset;
//     return p_object->p_vtable->pf_deinit(p_object);
// }

// app_foc_control_status_enum app_foc_control_run(
//     app_foc_control_t *p_object,
//     uint32_t elapsed_us)
// {
//     app_foc_control_status_enum status;
//     if ((!app_foc_control_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_run == NULL)) {
//         return app_foc_control_err_param;
//     }
//     status = p_object->p_vtable->pf_run(p_object, elapsed_us);
//     if (status == app_foc_control_ok) {
//         p_object->state = app_foc_control_state_running;
//     } else {
//         p_object->state = app_foc_control_state_error;
//     }
//     return status;
// }

// app_foc_control_status_enum app_foc_control_set_enable(
//     app_foc_control_t *p_object,
//     bool is_enabled)
// {
//     if ((!app_foc_control_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_set_enable == NULL)) {
//         return app_foc_control_err_param;
//     }
//     return p_object->p_vtable->pf_set_enable(p_object, is_enabled);
// }

// app_foc_control_status_enum app_foc_control_set_speed_reference(
//     app_foc_control_t *p_object,
//     float speed_reference_rpm)
// {
//     if ((!app_foc_control_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_set_speed_reference == NULL)) {
//         return app_foc_control_err_param;
//     }
//     p_object->config.speed_reference_rpm = speed_reference_rpm;
//     return p_object->p_vtable->pf_set_speed_reference(
//         p_object, speed_reference_rpm);
// }

// app_foc_control_status_enum app_foc_control_set_torque_reference(
//     app_foc_control_t *p_object,
//     float torque_reference)
// {
//     if ((!app_foc_control_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_set_torque_reference == NULL)) {
//         return app_foc_control_err_param;
//     }
//     p_object->config.torque_reference = torque_reference;
//     return p_object->p_vtable->pf_set_torque_reference(
//         p_object, torque_reference);
// }

// app_foc_control_state_enum app_foc_control_get_state(
//     const app_foc_control_t *p_object)
// {
//     if (p_object == NULL) {
//         return app_foc_control_state_error;
//     }
//     return p_object->state;
// }



// ADC注入组中断线测试
// void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
// void ADC1_2_IRQHandler(void)
// {
//     PLAT_PERIPH_ADC2->STATR = 
// }

bool irq_count = 0;

PLAT_ISR_DEFINE(PLAT_ISR_ADC1_2)
{
    bool is_set;
    (void)hal_adc_ops->pf_get_flag(&hal_adc_injected, PLAT_ADC_FLAG_JEOC, &is_set);

    if(is_set)
    {
        PLAT_PERIPH_ADC2->STATR &= ~(0x04);

        if(!irq_count)
        {
            irq_count = 1;
            hal_gpio_ops->pf_write(&hal_gpio_io_test,hal_gpio_test_pin_1,1);
        }
        else
        {
            irq_count = 0;
            hal_gpio_ops->pf_write(&hal_gpio_io_test,hal_gpio_test_pin_1,0);
        }

        bsp_current_sense_calc(&bsp_current_sense_three_phase);
    }
}






