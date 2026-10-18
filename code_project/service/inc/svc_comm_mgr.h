/**
 * @file svc_comm_mgr.h
 * @brief 通信服务接口。
 * @details 提供发送、接收和周期处理操作，并区分调试通道与远程控制通道。
 */
#ifndef __SVC_COMM_MGR_H__
#define __SVC_COMM_MGR_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct svc_communication svc_communication_t;

// /** @brief 通信服务对象的生命周期状态。 */
// typedef enum {
//     svc_communication_state_reset = 0,
//     svc_communication_state_ready,
//     svc_communication_state_running,
//     svc_communication_state_error
// } svc_communication_state_enum;

// /** @brief 通信服务接口执行结果。 */
// typedef enum {
//     svc_communication_ok = 0,
//     svc_communication_err_param = -1,
//     svc_communication_err_state = -2,
//     svc_communication_err_unsupported = -3,
//     svc_communication_err_timeout = -4
// } svc_communication_status_enum;

// /** @brief 通信服务使用角色。 */
// typedef enum {
//     svc_communication_role_debug = 0,
//     svc_communication_role_remote
// } svc_communication_role_enum;

// /** @brief 通信帧长度和处理周期配置。 */
// typedef struct {
//     svc_communication_role_enum role;
//     uint16_t maximum_frame_length;
//     uint32_t process_period_us;
//     void *p_user_data;
// } svc_communication_config_t;

// /** @brief 通信初始化、收发和周期处理操作表。 */
// typedef struct {
//     svc_communication_status_enum (*pf_init)(svc_communication_t *p_object);
//     svc_communication_status_enum (*pf_deinit)(svc_communication_t *p_object);
//     svc_communication_status_enum (*pf_send)(svc_communication_t *p_object, const uint8_t *p_tx_buf, uint16_t data_length);
//     svc_communication_status_enum (*pf_receive)(svc_communication_t *p_object, uint8_t *p_rx_buf, uint16_t buffer_length, uint16_t *p_received_length);
//     svc_communication_status_enum (*pf_process)(svc_communication_t *p_object, uint32_t elapsed_us);
// } svc_communication_vtable_t;

// /**
//  * @brief 通信服务对象。
//  * @details 保存操作表、运行状态、通信配置、传输依赖和用户上下文。
//  */
// struct svc_communication {
//     const svc_communication_vtable_t *p_vtable;
//     svc_communication_state_enum state;
//     svc_communication_config_t config;
//     void *p_transport_dependency;
//     void *p_user_data;
// };

// /** @brief 将通信服务操作表、配置和用户上下文保存到对象。 */
// svc_communication_status_enum svc_communication_bind(
//     svc_communication_t *p_object,
//     const svc_communication_vtable_t *p_vtable,
//     const svc_communication_config_t *p_config,
//     void *p_user_data);

// /** @brief 绑定UART、CAN等底层传输依赖。 */
// svc_communication_status_enum svc_communication_bind_dependency(
//     svc_communication_t *p_object,
//     void *p_transport_dependency);

// /** @brief 初始化通信服务对象。 */
// svc_communication_status_enum svc_communication_init(svc_communication_t *p_object);

// /** @brief 反初始化通信服务对象。 */
// svc_communication_status_enum svc_communication_deinit(svc_communication_t *p_object);

// /** @brief 发送一段通信数据。 */
// svc_communication_status_enum svc_communication_send(
//     svc_communication_t *p_object,
//     const uint8_t *p_tx_buf,
//     uint16_t data_length);

// /** @brief 接收一段通信数据并返回实际长度。 */
// svc_communication_status_enum svc_communication_receive(
//     svc_communication_t *p_object,
//     uint8_t *p_rx_buf,
//     uint16_t buffer_length,
//     uint16_t *p_received_length);

// /** @brief 按经过时间执行一次通信协议处理。 */
// svc_communication_status_enum svc_communication_process(
//     svc_communication_t *p_object,
//     uint32_t elapsed_us);

// /** @brief 获取通信服务当前状态。 */
// svc_communication_state_enum svc_communication_get_state(
//     const svc_communication_t *p_object);

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_SERVICE_H__ */









