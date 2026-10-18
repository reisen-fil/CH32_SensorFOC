/**
 * @file app_system.c
 * @brief 应用系统生命周期与周期调度实现。
 * @details 该模块通过对象状态和操作表组织应用系统功能；具体硬件访问由下层适配实现。
 */
#include "app_system.h"
#include <stddef.h>

// static bool app_system_is_object_valid(const app_system_t *p_object)
// {
//     return (p_object != NULL) && (p_object->p_vtable != NULL);
// }

// static app_system_status_enum app_system_default_init(app_system_t *p_object)
// {
//     (void)p_object;
//     /* TODO: Initialize Service objects, App objects, clocks, and board-independent resources. */
//     return app_system_ok;
// }

// static app_system_status_enum app_system_default_deinit(app_system_t *p_object)
// {
//     (void)p_object;
//     /* TODO: Deinitialize application and Service objects in dependency order. */
//     return app_system_ok;
// }

// static app_system_status_enum app_system_default_process(
//     app_system_t *p_object,
//     uint32_t elapsed_us)
// {
//     (void)p_object;
//     (void)elapsed_us;
//     /* TODO: Dispatch state machine, FOC control, protection, and telemetry processing. */
//     return app_system_ok;
// }

// static app_system_status_enum app_system_default_idle(app_system_t *p_object)
// {
//     (void)p_object;
//     /* TODO: Add low-priority application work and background diagnostics. */
//     return app_system_ok;
// }

// static const app_system_vtable_t s_app_system_default_vtable = {
//     .pf_init = app_system_default_init,
//     .pf_process = app_system_default_process,
//     .pf_idle = app_system_default_idle
// };

// app_system_status_enum app_system_bind(
//     app_system_t *p_object,
//     const app_system_vtable_t *p_vtable,
//     const app_system_config_t *p_config,
//     void *p_user_data)
// {
//     if ((p_object == NULL) || (p_vtable == NULL) || (p_config == NULL)) {
//         return app_system_err_param;
//     }
//     p_object->p_vtable = p_vtable;
//     p_object->state = app_system_state_reset;
//     p_object->config = *p_config;
//     p_object->p_foc_control = NULL;
//     p_object->p_state_machine_service = NULL;
//     p_object->p_communication_service = NULL;
//     p_object->p_user_data = p_user_data;
//     return app_system_ok;
// }

// app_system_status_enum app_system_create(
//     app_system_t *p_object,
//     const app_system_config_t *p_config)
// {
//     return app_system_bind(
//         p_object, &s_app_system_default_vtable, p_config, NULL);
// }

// app_system_status_enum app_system_bind_dependencies(
//     app_system_t *p_object,
//     app_foc_control_t *p_foc_control,
//     svc_state_machine_t *p_state_machine_service,
//     svc_communication_t *p_communication_service)
// {
//     if (p_object == NULL) {
//         return app_system_err_param;
//     }
//     p_object->p_foc_control = p_foc_control;
//     p_object->p_state_machine_service = p_state_machine_service;
//     p_object->p_communication_service = p_communication_service;
//     return app_system_ok;
// }

// app_system_status_enum app_system_init(app_system_t *p_object)
// {
//     app_system_status_enum status;
//     if ((!app_system_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_init == NULL)) {
//         return app_system_err_param;
//     }
//     status = p_object->p_vtable->pf_init(p_object);
//     p_object->state = (status == app_system_ok) ?
//                       app_system_state_ready : app_system_state_fault;
//     return status;
// }

// app_system_status_enum app_system_process(
//     app_system_t *p_object,
//     uint32_t elapsed_us)
// {
//     if ((!app_system_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_process == NULL)) {
//         return app_system_err_param;
//     }
//     return p_object->p_vtable->pf_process(p_object, elapsed_us);
// }

// app_system_status_enum app_system_idle(app_system_t *p_object)
// {
//     if ((!app_system_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_idle == NULL)) {
//         return app_system_err_param;
//     }
//     return p_object->p_vtable->pf_idle(p_object);
// }

// app_system_state_enum app_system_get_state(const app_system_t *p_object)
// {
//     if (p_object == NULL) {
//         return app_system_state_fault;
//     }
//     return p_object->state;
// }


/* USART3 中断线触发+空闲中断接收测试 */

PLAT_ISR_DEFINE(PLAT_ISR_USART3)
{
    // bool is_set;
          
    hal_uart_ops->pf_rx_idle_handler(&hal_uart_comm);
}








