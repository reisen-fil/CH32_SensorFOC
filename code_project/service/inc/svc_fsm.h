/**
 * @file svc_fsm.h
 * @brief 状态机服务接口。
 * @details 提供状态进入、周期运行和事件处理操作，并保存当前及上一个状态编号。
 */
#ifndef __SVC_FSM_H__
#define __SVC_FSM_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct svc_state_machine svc_state_machine_t;

// /** @brief 状态机服务对象的生命周期状态。 */
// typedef enum {
//     svc_state_machine_state_reset = 0,
//     svc_state_machine_state_ready,
//     svc_state_machine_state_running,
//     svc_state_machine_state_error
// } svc_state_machine_state_enum;

// /** @brief 状态机服务接口执行结果。 */
// typedef enum {
//     svc_state_machine_ok = 0,
//     svc_state_machine_err_param = -1,
//     svc_state_machine_err_state = -2,
//     svc_state_machine_err_unsupported = -3,
//     svc_state_machine_err_timeout = -4
// } svc_state_machine_status_enum;

// /** @brief 状态机状态数量和初始状态配置。 */
// typedef struct {
//     uint16_t state_count;
//     uint16_t initial_state;
//     void *p_user_data;
// } svc_state_machine_config_t;

// /** @brief 状态机初始化、状态切换、运行和事件处理操作表。 */
// typedef struct {
//     svc_state_machine_status_enum (*pf_init)(svc_state_machine_t *p_object);
//     svc_state_machine_status_enum (*pf_deinit)(svc_state_machine_t *p_object);
//     svc_state_machine_status_enum (*pf_enter_state)(svc_state_machine_t *p_object, uint16_t state_id);
//     svc_state_machine_status_enum (*pf_run)(svc_state_machine_t *p_object, uint32_t elapsed_us);
//     svc_state_machine_status_enum (*pf_handle_event)(svc_state_machine_t *p_object, uint32_t event_id);
// } svc_state_machine_vtable_t;

// /**
//  * @brief 状态机服务对象。
//  * @details 保存操作表、运行状态、状态编号以及外部依赖和用户上下文。
//  */
// struct svc_state_machine {
//     const svc_state_machine_vtable_t *p_vtable;
//     svc_state_machine_state_enum state;
//     svc_state_machine_config_t config;
//     uint16_t current_state_id;
//     uint16_t previous_state_id;
//     void *p_dependency;
//     void *p_user_data;
// };

// /** @brief 将状态机操作表、配置和用户上下文保存到对象。 */
// svc_state_machine_status_enum svc_state_machine_bind(
//     svc_state_machine_t *p_object,
//     const svc_state_machine_vtable_t *p_vtable,
//     const svc_state_machine_config_t *p_config,
//     void *p_user_data);

// /** @brief 绑定状态机依赖的外部上下文。 */
// svc_state_machine_status_enum svc_state_machine_bind_dependency(
//     svc_state_machine_t *p_object,
//     void *p_dependency);

// /** @brief 初始化状态机并进入初始状态。 */
// svc_state_machine_status_enum svc_state_machine_init(svc_state_machine_t *p_object);

// /** @brief 反初始化状态机服务对象。 */
// svc_state_machine_status_enum svc_state_machine_deinit(svc_state_machine_t *p_object);

// /** @brief 请求状态机进入指定状态。 */
// svc_state_machine_status_enum svc_state_machine_enter(
//     svc_state_machine_t *p_object,
//     uint16_t state_id);

// /** @brief 按经过时间运行一次当前状态。 */
// svc_state_machine_status_enum svc_state_machine_run(
//     svc_state_machine_t *p_object,
//     uint32_t elapsed_us);

// /** @brief 向状态机分发一个事件编号。 */
// svc_state_machine_status_enum svc_state_machine_handle_event(
//     svc_state_machine_t *p_object,
//     uint32_t event_id);

// /** @brief 获取状态机服务当前运行状态。 */
// svc_state_machine_state_enum svc_state_machine_get_state(
//     const svc_state_machine_t *p_object);

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_SERVICE_H__ */









