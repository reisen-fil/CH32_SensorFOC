/**
 * @file svc_fsm.c
 * @brief 状态机配置。
 * @details 该模块通过对象状态和操作表组织状态机功能；具体硬件访问由下层适配实现。
 */
#include "svc_fsm.h"
#include <stddef.h>

// static bool svc_state_machine_is_object_valid(
//     const svc_state_machine_t *p_object)
// {
//     return (p_object != NULL) && (p_object->p_vtable != NULL);
// }

// svc_state_machine_status_enum svc_state_machine_bind(
//     svc_state_machine_t *p_object,
//     const svc_state_machine_vtable_t *p_vtable,
//     const svc_state_machine_config_t *p_config,
//     void *p_user_data)
// {
//     if ((p_object == NULL) || (p_vtable == NULL) || (p_config == NULL)) {
//         return svc_state_machine_err_param;
//     }
//     p_object->p_vtable = p_vtable;
//     p_object->state = svc_state_machine_state_reset;
//     p_object->config = *p_config;
//     p_object->current_state_id = p_config->initial_state;
//     p_object->previous_state_id = p_config->initial_state;
//     p_object->p_dependency = NULL;
//     p_object->p_user_data = p_user_data;
//     return svc_state_machine_ok;
// }

// svc_state_machine_status_enum svc_state_machine_bind_dependency(
//     svc_state_machine_t *p_object,
//     void *p_dependency)
// {
//     if (p_object == NULL) {
//         return svc_state_machine_err_param;
//     }
//     p_object->p_dependency = p_dependency;
//     return svc_state_machine_ok;
// }

// svc_state_machine_status_enum svc_state_machine_init(
//     svc_state_machine_t *p_object)
// {
//     svc_state_machine_status_enum status;
//     if ((!svc_state_machine_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_init == NULL)) {
//         return svc_state_machine_err_param;
//     }
//     status = p_object->p_vtable->pf_init(p_object);
//     p_object->state = (status == svc_state_machine_ok) ?
//                       svc_state_machine_state_ready :
//                       svc_state_machine_state_error;
//     return status;
// }

// svc_state_machine_status_enum svc_state_machine_deinit(
//     svc_state_machine_t *p_object)
// {
//     if ((!svc_state_machine_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_deinit == NULL)) {
//         return svc_state_machine_err_param;
//     }
//     p_object->state = svc_state_machine_state_reset;
//     return p_object->p_vtable->pf_deinit(p_object);
// }

// svc_state_machine_status_enum svc_state_machine_enter(
//     svc_state_machine_t *p_object,
//     uint16_t state_id)
// {
//     svc_state_machine_status_enum status;
//     if ((!svc_state_machine_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_enter_state == NULL)) {
//         return svc_state_machine_err_param;
//     }
//     status = p_object->p_vtable->pf_enter_state(p_object, state_id);
//     if (status == svc_state_machine_ok) {
//         p_object->previous_state_id = p_object->current_state_id;
//         p_object->current_state_id = state_id;
//     }
//     return status;
// }

// svc_state_machine_status_enum svc_state_machine_run(
//     svc_state_machine_t *p_object,
//     uint32_t elapsed_us)
// {
//     if ((!svc_state_machine_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_run == NULL)) {
//         return svc_state_machine_err_param;
//     }
//     return p_object->p_vtable->pf_run(p_object, elapsed_us);
// }

// svc_state_machine_status_enum svc_state_machine_handle_event(
//     svc_state_machine_t *p_object,
//     uint32_t event_id)
// {
//     if ((!svc_state_machine_is_object_valid(p_object)) ||
//         (p_object->p_vtable->pf_handle_event == NULL)) {
//         return svc_state_machine_err_param;
//     }
//     return p_object->p_vtable->pf_handle_event(p_object, event_id);
// }

// svc_state_machine_state_enum svc_state_machine_get_state(
//     const svc_state_machine_t *p_object)
// {
//     if (p_object == NULL) {
//         return svc_state_machine_state_error;
//     }
//     return p_object->state;
// }










