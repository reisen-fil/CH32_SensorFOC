/**
 * @file svc_foc.h
 * @brief FOC 算法服务接口。
 * @details 定义浮点和 Q16 定点坐标变换、PWM 调制，以及服务对象的依赖绑定接口。
 */
#ifndef __SVC_FOC_H__
#define __SVC_FOC_H__

#include <stdint.h>
#include <stdbool.h>

#include "platform_ch32v203_gpio.h"
#include "hal_gpio.h"
#include "bsp_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct svc_foc svc_foc_t;
// typedef int32_t svc_foc_q16_t;

// /** @brief FOC服务对象的生命周期状态。 */
// typedef enum {
//     svc_foc_state_reset = 0,
//     svc_foc_state_ready,
//     svc_foc_state_running,
//     svc_foc_state_error
// } svc_foc_state_enum;

// /** @brief FOC服务接口调用返回的状态码。 */
// typedef enum {
//     svc_foc_ok = 0,
//     svc_foc_err_param = -1,
//     svc_foc_err_state = -2,
//     svc_foc_err_unsupported = -3,
//     svc_foc_err_timeout = -4
// } svc_foc_status_enum;

// /** @brief FOC计算使用的浮点或Q16定点格式。 */
// typedef enum {
//     svc_foc_format_float = 0,
//     svc_foc_format_q16
// } svc_foc_format_enum;

// /** @brief FOC PWM 调制方式枚举。 */
// typedef enum {
//     svc_foc_modulation_svpwm_sector = 0,
//     svc_foc_modulation_zero_sequence_spwm,
//     svc_foc_modulation_spwm
// } svc_foc_modulation_enum;

// /** @brief 浮点三相坐标量，包含 U、V、W 分量。 */
// typedef struct {
//     float phase_u;
//     float phase_v;
//     float phase_w;
// } svc_foc_float_abc_t;

// /** @brief 浮点静止坐标量，包含 Alpha、Beta 分量。 */
// typedef struct {
//     float alpha;
//     float beta;
// } svc_foc_float_alpha_beta_t;

// /** @brief 浮点旋转坐标量，包含 D、Q 分量。 */
// typedef struct {
//     float d;
//     float q;
// } svc_foc_float_dq_t;

// /** @brief Q16 定点三相坐标量。 */
// typedef struct {
//     svc_foc_q16_t phase_u;
//     svc_foc_q16_t phase_v;
//     svc_foc_q16_t phase_w;
// } svc_foc_q16_abc_t;

// /** @brief Q16 定点静止坐标量。 */
// typedef struct {
//     svc_foc_q16_t alpha;
//     svc_foc_q16_t beta;
// } svc_foc_q16_alpha_beta_t;

// /** @brief Q16 定点旋转坐标量。 */
// typedef struct {
//     svc_foc_q16_t d;
//     svc_foc_q16_t q;
// } svc_foc_q16_dq_t;

// /** @brief FOC PWM 调制输出，包含三相占空比和扇区号。 */
// typedef struct {
//     float duty_u;
//     float duty_v;
//     float duty_w;
//     uint8_t sector;
// } svc_foc_float_pwm_output_t;

// /** @brief FOC PWM 调制输出，包含三相占空比和扇区号。 */
// typedef struct {
//     svc_foc_q16_t duty_u;
//     svc_foc_q16_t duty_v;
//     svc_foc_q16_t duty_w;
//     uint8_t sector;
// } svc_foc_q16_pwm_output_t;

// /** @brief FOC控制周期、极对数、母线电压和调制方式配置。 */
// typedef struct {
//     svc_foc_format_enum arithmetic_format;
//     svc_foc_modulation_enum modulation;
//     uint32_t control_frequency_hz;
//     uint8_t motor_pole_pairs;
//     float dc_bus_voltage;
//     svc_foc_q16_t dc_bus_voltage_q16;
//     void *p_user_data;
// } svc_foc_config_t;

// /** @brief FOC坐标变换和PWM调制操作表。 */
// typedef struct {
//     svc_foc_status_enum (*pf_init)(svc_foc_t *p_object);
//     svc_foc_status_enum (*pf_deinit)(svc_foc_t *p_object);
//     svc_foc_status_enum (*pf_float_clarke)(svc_foc_t *p_object, const svc_foc_float_abc_t *p_abc, svc_foc_float_alpha_beta_t *p_alpha_beta);
//     svc_foc_status_enum (*pf_float_park)(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, float electrical_angle, svc_foc_float_dq_t *p_dq);
//     svc_foc_status_enum (*pf_float_inverse_park)(svc_foc_t *p_object, const svc_foc_float_dq_t *p_dq, float electrical_angle, svc_foc_float_alpha_beta_t *p_alpha_beta);
//     svc_foc_status_enum (*pf_q16_clarke)(svc_foc_t *p_object, const svc_foc_q16_abc_t *p_abc, svc_foc_q16_alpha_beta_t *p_alpha_beta);
//     svc_foc_status_enum (*pf_q16_park)(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_t sine_value, svc_foc_q16_t cosine_value, svc_foc_q16_dq_t *p_dq);
//     svc_foc_status_enum (*pf_q16_inverse_park)(svc_foc_t *p_object, const svc_foc_q16_dq_t *p_dq, svc_foc_q16_t sine_value, svc_foc_q16_t cosine_value, svc_foc_q16_alpha_beta_t *p_alpha_beta);
//     svc_foc_status_enum (*pf_float_modulate_svpwm_sector)(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, svc_foc_float_pwm_output_t *p_output);
//     svc_foc_status_enum (*pf_float_modulate_zero_sequence_spwm)(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, svc_foc_float_pwm_output_t *p_output);
//     svc_foc_status_enum (*pf_float_modulate_spwm)(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, svc_foc_float_pwm_output_t *p_output);
//     svc_foc_status_enum (*pf_q16_modulate_svpwm_sector)(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_pwm_output_t *p_output);
//     svc_foc_status_enum (*pf_q16_modulate_zero_sequence_spwm)(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_pwm_output_t *p_output);
//     svc_foc_status_enum (*pf_q16_modulate_spwm)(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_pwm_output_t *p_output);
// } svc_foc_vtable_t;

// /**
//  * @brief FOC 服务对象。
//  * @details 保存算法操作表、运行状态、配置及电机、编码器和电流采样依赖。
//  */
// struct svc_foc {
//     const svc_foc_vtable_t *p_vtable;
//     svc_foc_state_enum state;
//     svc_foc_config_t config;
//     void *p_motor_dependency;
//     void *p_encoder_dependency;
//     void *p_current_sense_dependency;
//     void *p_user_data;
// };

// /** @brief 将FOC操作表、算法配置和用户上下文保存到对象。 */
// svc_foc_status_enum svc_foc_bind(svc_foc_t *p_object, const svc_foc_vtable_t *p_vtable, const svc_foc_config_t *p_config, void *p_user_data);

// /** @brief 绑定电机、编码器和电流采样依赖。 */
// svc_foc_status_enum svc_foc_bind_dependencies(svc_foc_t *p_object, void *p_motor_dependency, void *p_encoder_dependency, void *p_current_sense_dependency);

// /** @brief 初始化FOC算法服务。 */
// svc_foc_status_enum svc_foc_init(svc_foc_t *p_object);

// /** @brief 关闭FOC算法服务并释放资源。 */
// svc_foc_status_enum svc_foc_deinit(svc_foc_t *p_object);

// /** @brief 执行浮点 Clarke 变换，将三相量转换为 Alpha-Beta 量。 */
// svc_foc_status_enum svc_foc_float_clarke(svc_foc_t *p_object, const svc_foc_float_abc_t *p_abc, svc_foc_float_alpha_beta_t *p_alpha_beta);

// /** @brief 执行浮点 Park 变换，将 Alpha-Beta 量转换为 D-Q 量。 */
// svc_foc_status_enum svc_foc_float_park(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, float electrical_angle, svc_foc_float_dq_t *p_dq);

// /** @brief 执行浮点逆 Park 变换，将 D-Q 量转换为 Alpha-Beta 量。 */
// svc_foc_status_enum svc_foc_float_inverse_park(svc_foc_t *p_object, const svc_foc_float_dq_t *p_dq, float electrical_angle, svc_foc_float_alpha_beta_t *p_alpha_beta);

// /** @brief 执行 Q16 定点 Clarke 变换。 */
// svc_foc_status_enum svc_foc_q16_clarke(svc_foc_t *p_object, const svc_foc_q16_abc_t *p_abc, svc_foc_q16_alpha_beta_t *p_alpha_beta);

// /** @brief 执行 Q16 定点 Park 变换。 */
// svc_foc_status_enum svc_foc_q16_park(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_t sine_value, svc_foc_q16_t cosine_value, svc_foc_q16_dq_t *p_dq);

// /** @brief 执行 Q16 定点逆 Park 变换。 */
// svc_foc_status_enum svc_foc_q16_inverse_park(svc_foc_t *p_object, const svc_foc_q16_dq_t *p_dq, svc_foc_q16_t sine_value, svc_foc_q16_t cosine_value, svc_foc_q16_alpha_beta_t *p_alpha_beta);

// /** @brief 使用扇区 SVPWM 计算三相 PWM 占空比。 */
// svc_foc_status_enum svc_foc_float_modulate_svpwm_sector(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, svc_foc_float_pwm_output_t *p_output);

// /** @brief 使用零序注入 SPWM 计算三相 PWM 占空比。 */
// svc_foc_status_enum svc_foc_float_modulate_zero_sequence_spwm(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, svc_foc_float_pwm_output_t *p_output);

// /** @brief 使用 SPWM 计算三相 PWM 占空比。 */
// svc_foc_status_enum svc_foc_float_modulate_spwm(svc_foc_t *p_object, const svc_foc_float_alpha_beta_t *p_alpha_beta, svc_foc_float_pwm_output_t *p_output);

// /** @brief 使用扇区 SVPWM 计算三相 PWM 占空比。 */
// svc_foc_status_enum svc_foc_q16_modulate_svpwm_sector(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_pwm_output_t *p_output);

// /** @brief 使用零序注入 SPWM 计算三相 PWM 占空比。 */
// svc_foc_status_enum svc_foc_q16_modulate_zero_sequence_spwm(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_pwm_output_t *p_output);

// /** @brief 使用 SPWM 计算三相 PWM 占空比。 */
// svc_foc_status_enum svc_foc_q16_modulate_spwm(svc_foc_t *p_object, const svc_foc_q16_alpha_beta_t *p_alpha_beta, svc_foc_q16_pwm_output_t *p_output);

// /** @brief 查询FOC服务当前状态。 */
// svc_foc_state_enum svc_foc_get_state(const svc_foc_t *p_object);

#ifdef __cplusplus
}
#endif

#endif /* __FOC_SERVICE_H__ */









