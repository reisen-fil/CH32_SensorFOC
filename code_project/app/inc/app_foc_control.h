/**
 * @file app_foc_control.h
 * @brief 应用层FOC控制调度接口。
 * @details 该模块通过对象状态和操作表组织FOC功能；具体硬件访问由下层适配实现。
 */
#ifndef __APP_FOC_CONTROL_H__
#define __APP_FOC_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include "svc_foc.h"
#include "svc_fsm.h"
#include "svc_comm_mgr.h"

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct app_foc_control app_foc_control_t;

// /** @brief 应用 FOC 控制对象的生命周期状态。 */
// typedef enum {
//     app_foc_control_state_reset = 0,
//     app_foc_control_state_ready,
//     app_foc_control_state_running,
//     app_foc_control_state_stopped,
//     app_foc_control_state_error
// } app_foc_control_state_enum;

// /** @brief 应用 FOC 控制接口执行结果。 */
// typedef enum {
//     app_foc_control_ok = 0,
//     app_foc_control_err_param = -1,
//     app_foc_control_err_state = -2,
//     app_foc_control_err_unsupported = -3,
//     app_foc_control_err_timeout = -4
// } app_foc_control_status_enum;

// /** @brief 应用系统控制周期和依赖对象配置。 */
// typedef struct {
//     svc_foc_format_enum arithmetic_format;
//     svc_foc_modulation_enum modulation;
//     uint32_t control_period_us;
//     float speed_reference_rpm;
//     float torque_reference;
//     void *p_user_data;
// } app_foc_control_config_t;

// /** @brief 应用FOC初始化、运行和参考值设置操作表。 */
// typedef struct {
//     app_foc_control_status_enum (*pf_init)(app_foc_control_t *p_object);
//     app_foc_control_status_enum (*pf_deinit)(app_foc_control_t *p_object);
//     app_foc_control_status_enum (*pf_run)(app_foc_control_t *p_object, uint32_t elapsed_us);
//     app_foc_control_status_enum (*pf_set_enable)(app_foc_control_t *p_object, bool is_enabled);
//     app_foc_control_status_enum (*pf_set_speed_reference)(app_foc_control_t *p_object, float speed_reference_rpm);
//     app_foc_control_status_enum (*pf_set_torque_reference)(app_foc_control_t *p_object, float torque_reference);
// } app_foc_control_vtable_t;

// /**
//  * @brief 应用 FOC 控制对象。
//  * @details 保存控制状态、周期配置以及 FOC、状态机和通信服务依赖。
//  */
// struct app_foc_control {
//     const app_foc_control_vtable_t *p_vtable;
//     app_foc_control_state_enum state;
//     app_foc_control_config_t config;
//     svc_foc_t *p_foc_service;
//     svc_state_machine_t *p_state_machine_service;
//     svc_communication_t *p_communication_service;
//     void *p_user_data;
// };

// /** @brief 将应用FOC操作表、配置和用户上下文保存到对象。 */
// app_foc_control_status_enum app_foc_control_bind(
//     app_foc_control_t *p_object,
//     const app_foc_control_vtable_t *p_vtable,
//     const app_foc_control_config_t *p_config,
//     void *p_user_data);

// /** @brief 绑定FOC、状态机和通信服务依赖。 */
// app_foc_control_status_enum app_foc_control_bind_dependencies(
//     app_foc_control_t *p_object,
//     svc_foc_t *p_foc_service,
//     svc_state_machine_t *p_state_machine_service,
//     svc_communication_t *p_communication_service);

// /** @brief 初始化应用 FOC 控制对象。 */
// app_foc_control_status_enum app_foc_control_init(app_foc_control_t *p_object);

// /** @brief 反初始化应用 FOC 控制对象。 */
// app_foc_control_status_enum app_foc_control_deinit(app_foc_control_t *p_object);

// /** @brief 按经过时间运行一次应用FOC控制流程。 */
// app_foc_control_status_enum app_foc_control_run(
//     app_foc_control_t *p_object,
//     uint32_t elapsed_us);

// /** @brief 使能或禁用应用 FOC 控制环。 */
// app_foc_control_status_enum app_foc_control_set_enable(
//     app_foc_control_t *p_object,
//     bool is_enabled);

// /** @brief 设置速度环目标转速。 */
// app_foc_control_status_enum app_foc_control_set_speed_reference(
//     app_foc_control_t *p_object,
//     float speed_reference_rpm);

// /** @brief 设置转矩控制目标值。 */
// app_foc_control_status_enum app_foc_control_set_torque_reference(
//     app_foc_control_t *p_object,
//     float torque_reference);

// /** @brief 获取应用 FOC 控制对象当前状态。 */
// app_foc_control_state_enum app_foc_control_get_state(
//     const app_foc_control_t *p_object);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FOC_CONTROL_H__ */










