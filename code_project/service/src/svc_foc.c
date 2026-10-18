/**
 * @file svc_foc.c
 * @brief FOC 服务层实现（电流获取 + dq 电压 → 三相 PWM 输出）。
 * @details 对外只有四个接口（init / get_state / output_dq_pwm / get_current），内部实现：
 *            dq(V) --逆Park--> αβ(V) --÷母线电压--> αβ(标幺) --SVPWM--> 占空比(标幺)
 *                  --×pwm_period--> 比较值计数 --> bsp_motor_ctrl_ops->pf_set_duty()
 *            三相电流(A) --Clarke--> αβ(A) --Park--> dq(A)（观测用，供上层上报/调试）
 *          Clarke/Park/SVPWM/三角运算全部是本文件内的 static 函数，不对外暴露。
 *          运算类型由 config.arithmetic_type 分派：Q15 定点路径全程 PLAT_IQ_*（三角运算用
 *          PLAT_IQ_SIN_PU / PLAT_IQ_COS_PU），浮点路径走单精度运算与 sinf / cosf。
 *          本文件不做任何 BSP 外设初始化，只在运行时经 BSP 函数表访问电流采样与三相 PWM。
 */
#include "svc_foc.h"

#include "bsp_current_sense.h"
#include "bsp_motor_ctrl.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

/** @brief √3 相关的标幺域常量（编译期换算成 Q15 常量，不产生运行期浮点运算）。 */
#define SVC_FOC_SQRT3              (1.7320508f)
#define SVC_FOC_HALF_SQRT3         (0.8660254f)
#define SVC_FOC_ONE_POINT_FIVE     (1.5f)
#define SVC_FOC_INV_SQRT3          (0.5773503f)

/** @brief 2π：浮点路径把标幺电角度换算成弧度。 */
#define SVC_FOC_TWO_PI             (6.2831853f)

/**
 * @brief FOC 服务层板级配置（Q15 定点 + 扇区法）。
 * @details pwm_period 取 HAL_TIM_PWM_PERIOD（12 kHz 中心对齐，144 MHz/(12000×2) = 6000）；
 *          dc_bus_voltage_v 是母线电压，SVPWM 用它把伏特归一化成占空比，接不同母线只需改这里。
 */
static const svc_foc_config_t s_svc_foc_config = {
    .arithmetic_type = svc_foc_arithmetic_q15,
    .modulation = svc_foc_modulation_svpwm_sector,
    .pwm_period = (uint16_t)HAL_TIM_PWM_PERIOD,
    .dc_bus_voltage_v = 12.0f,
    .p_user_data = NULL
};

/** @brief 板级 FOC 服务对象（单实例）。 */
svc_foc_t svc_foc_motor = {0};

/**
 * @brief 内部用：标幺占空比（0 ~ 1，按运算类型取并集成员）+ 扇区号。
 * @details 调制实现产出标幺值，再由 svc_foc_duty_to_counts() 统一换算成 BSP 需要的计数，
 *          避免在四个调制分支里重复做计数换算。
 */
typedef struct {
    svc_foc_value_t duty_u;   /**< U 相标幺占空比 */
    svc_foc_value_t duty_v;   /**< V 相标幺占空比 */
    svc_foc_value_t duty_w;   /**< W 相标幺占空比 */
    uint8_t sector;           /**< 扇区号 1 ~ 6；三次谐波注入法固定为 0 */
} svc_foc_duty_pu_t;

/**
 * @brief 校验服务层对象是否可用。
 * @param p_object FOC 服务对象指针。
 * @return svc_foc_ok 表示可用；否则返回对应错误码。
 */
static svc_foc_status_enum svc_foc_validate(const svc_foc_t *p_object)
{
    if (p_object == NULL) {
        return svc_foc_err_param;
    }
    if (p_object->state != svc_foc_state_ready) {
        return svc_foc_err_state;
    }
    return svc_foc_ok;
}

/**
 * @brief 当前配置是否使用 Q15 定点格式。
 * @param p_object FOC 服务对象指针。
 * @return true 表示并集值取 q_value 成员。
 */
static bool svc_foc_is_q15(const svc_foc_t *p_object)
{
    return (p_object->config.arithmetic_type == svc_foc_arithmetic_q15);
}

/**
 * @brief 把 Q15 占空比限幅到 [0, 1]。
 * @param value Q15 占空比。
 * @return 限幅后的值。
 */
static PLAT_IQ_TYPE svc_foc_clamp_q15(PLAT_IQ_TYPE value)
{
    if (value < 0) {
        return 0;
    }
    if (value > PLAT_IQ(1.0f)) {
        return PLAT_IQ(1.0f);
    }
    return value;
}

/**
 * @brief 把浮点占空比限幅到 [0, 1]。
 * @param value 浮点占空比。
 * @return 限幅后的值。
 */
static float svc_foc_clamp_float(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

/**
 * @brief 求标幺电角度的正弦/余弦（按运算类型分派）。
 * @param p_object FOC 服务对象指针。
 * @param angle_pu 标幺电角度。
 * @param p_sin 输出正弦值。
 * @param p_cos 输出余弦值。
 */
static void svc_foc_sin_cos_calc(const svc_foc_t *p_object,
                                 svc_foc_value_t angle_pu,
                                 svc_foc_value_t *p_sin,
                                 svc_foc_value_t *p_cos)
{
    if (svc_foc_is_q15(p_object)) {
        /* Q15：直接用标幺输入（1.0 = 2π）的查表函数 */
        p_sin->q_value = PLAT_IQ_SIN_PU(angle_pu.q_value);
        p_cos->q_value = PLAT_IQ_COS_PU(angle_pu.q_value);
    } else {
        float radian = SVC_FOC_TWO_PI * angle_pu.float_value;

        p_sin->float_value = sinf(radian);
        p_cos->float_value = cosf(radian);
    }
}

/**
 * @brief Clarke 正变换：三相 → 静止两相，幅值不变（α = u，β = (u + 2v)/√3）。
 * @param p_object FOC 服务对象指针。
 * @param p_abc 输入三相量。
 * @param p_alpha_beta 输出静止两相量。
 */
static void svc_foc_clarke_calc(const svc_foc_t *p_object,
                                const svc_foc_abc_t *p_abc,
                                svc_foc_alpha_beta_t *p_alpha_beta)
{
    if (svc_foc_is_q15(p_object)) {
        PLAT_IQ_TYPE sum = PLAT_IQ_TO_IQ((int32_t)p_abc->phase_u.q_value
                                         + (2 * (int32_t)p_abc->phase_v.q_value));

        p_alpha_beta->alpha.q_value = p_abc->phase_u.q_value;
        p_alpha_beta->beta.q_value = PLAT_IQ_MPY(sum, PLAT_IQ(SVC_FOC_INV_SQRT3));
    } else {
        p_alpha_beta->alpha.float_value = p_abc->phase_u.float_value;
        p_alpha_beta->beta.float_value =
            (p_abc->phase_u.float_value + (2.0f * p_abc->phase_v.float_value)) *
            SVC_FOC_INV_SQRT3;
    }
}

/**
 * @brief Clarke 逆变换：静止两相 → 三相，幅值不变。
 * @details u = α，v = -α/2 + (√3/2)β，w = -α/2 - (√3/2)β。
 * @param p_object FOC 服务对象指针。
 * @param p_alpha_beta 输入静止两相量。
 * @param p_abc 输出三相量。
 */
static void svc_foc_inverse_clarke_calc(const svc_foc_t *p_object,
                                        const svc_foc_alpha_beta_t *p_alpha_beta,
                                        svc_foc_abc_t *p_abc)
{
    if (svc_foc_is_q15(p_object)) {
        PLAT_IQ_TYPE half_alpha = PLAT_IQ_MPY(PLAT_IQ(0.5f), p_alpha_beta->alpha.q_value);
        PLAT_IQ_TYPE half_sqrt3_beta =
            PLAT_IQ_MPY(PLAT_IQ(SVC_FOC_HALF_SQRT3), p_alpha_beta->beta.q_value);

        p_abc->phase_u.q_value = p_alpha_beta->alpha.q_value;
        p_abc->phase_v.q_value = half_sqrt3_beta - half_alpha;
        p_abc->phase_w.q_value = -half_sqrt3_beta - half_alpha;
    } else {
        float half_alpha = 0.5f * p_alpha_beta->alpha.float_value;
        float half_sqrt3_beta = SVC_FOC_HALF_SQRT3 * p_alpha_beta->beta.float_value;

        p_abc->phase_u.float_value = p_alpha_beta->alpha.float_value;
        p_abc->phase_v.float_value = half_sqrt3_beta - half_alpha;
        p_abc->phase_w.float_value = -half_sqrt3_beta - half_alpha;
    }
}

/**
 * @brief Park 正变换：静止两相 → 旋转两相。
 * @details d = α·cosθ + β·sinθ，q = β·cosθ - α·sinθ。
 * @param p_object FOC 服务对象指针。
 * @param p_alpha_beta 输入静止两相量。
 * @param angle_pu 标幺电角度。
 * @param p_dq 输出 dq 量。
 */
static void svc_foc_park_calc(const svc_foc_t *p_object,
                              const svc_foc_alpha_beta_t *p_alpha_beta,
                              svc_foc_value_t angle_pu,
                              svc_foc_dq_t *p_dq)
{
    svc_foc_value_t sin_value;
    svc_foc_value_t cos_value;

    svc_foc_sin_cos_calc(p_object, angle_pu, &sin_value, &cos_value);

    if (svc_foc_is_q15(p_object)) {
        p_dq->d.q_value = PLAT_IQ_MPY(p_alpha_beta->alpha.q_value, cos_value.q_value)
                          + PLAT_IQ_MPY(p_alpha_beta->beta.q_value, sin_value.q_value);
        p_dq->q.q_value = PLAT_IQ_MPY(p_alpha_beta->beta.q_value, cos_value.q_value)
                          - PLAT_IQ_MPY(p_alpha_beta->alpha.q_value, sin_value.q_value);
    } else {
        p_dq->d.float_value = (p_alpha_beta->alpha.float_value * cos_value.float_value)
                              + (p_alpha_beta->beta.float_value * sin_value.float_value);
        p_dq->q.float_value = (p_alpha_beta->beta.float_value * cos_value.float_value)
                              - (p_alpha_beta->alpha.float_value * sin_value.float_value);
    }
}

/**
 * @brief Park 逆变换：旋转两相 → 静止两相。
 * @details α = d·cosθ - q·sinθ，β = d·sinθ + q·cosθ。
 * @param p_object FOC 服务对象指针。
 * @param p_dq 输入 dq 量（电压时量纲为伏特）。
 * @param angle_pu 标幺电角度。
 * @param p_alpha_beta 输出静止两相量（伏特）。
 */
static void svc_foc_inverse_park_calc(const svc_foc_t *p_object,
                                      const svc_foc_dq_t *p_dq,
                                      svc_foc_value_t angle_pu,
                                      svc_foc_alpha_beta_t *p_alpha_beta)
{
    svc_foc_value_t sin_value;
    svc_foc_value_t cos_value;

    svc_foc_sin_cos_calc(p_object, angle_pu, &sin_value, &cos_value);

    if (svc_foc_is_q15(p_object)) {
        p_alpha_beta->alpha.q_value = PLAT_IQ_MPY(p_dq->d.q_value, cos_value.q_value)
                                      - PLAT_IQ_MPY(p_dq->q.q_value, sin_value.q_value);
        p_alpha_beta->beta.q_value = PLAT_IQ_MPY(p_dq->d.q_value, sin_value.q_value)
                                     + PLAT_IQ_MPY(p_dq->q.q_value, cos_value.q_value);
    } else {
        p_alpha_beta->alpha.float_value = (p_dq->d.float_value * cos_value.float_value)
                                          - (p_dq->q.float_value * sin_value.float_value);
        p_alpha_beta->beta.float_value = (p_dq->d.float_value * sin_value.float_value)
                                         + (p_dq->q.float_value * cos_value.float_value);
    }
}

/**
 * @brief 扇区法 SVPWM（Q15 定点路径）：标幺 αβ → 标幺占空比。
 * @details 先由 αβ 判扇区（A = β、B = (√3α - β)/2、C = (-√3α - β)/2 的符号位组合成
 *          N = 1 ~ 6），再由 X = √3β、Y = 1.5α + (√3/2)β、Z = -1.5α + (√3/2)β 按扇区
 *          取 T1/T2，最低相切换点 Ta = (1 - T1 - T2)/2，其余两相依次加 T1、T2。
 * @param p_object FOC 服务对象指针。
 * @param p_alpha_beta_pu 输入 αβ 电压（标幺母线电压，1.0 = Vdc）。
 * @param p_duty_pu 输出三相标幺占空比与扇区号。
 */
static void svc_foc_svpwm_sector_q15(const svc_foc_t *p_object,
                                     const svc_foc_alpha_beta_t *p_alpha_beta_pu,
                                     svc_foc_duty_pu_t *p_duty_pu)
{
    PLAT_IQ_TYPE v_alpha = p_alpha_beta_pu->alpha.q_value;
    PLAT_IQ_TYPE v_beta = p_alpha_beta_pu->beta.q_value;
    PLAT_IQ_TYPE x;
    PLAT_IQ_TYPE y;
    PLAT_IQ_TYPE z;
    PLAT_IQ_TYPE t1;
    PLAT_IQ_TYPE t2;
    PLAT_IQ_TYPE ta;
    PLAT_IQ_TYPE duty_u;
    PLAT_IQ_TYPE duty_v;
    PLAT_IQ_TYPE duty_w;
    uint8_t sector = 0U;

    (void)p_object;

    x = PLAT_IQ_MPY(v_beta, PLAT_IQ(SVC_FOC_SQRT3));
    y = PLAT_IQ_MPY(v_alpha, PLAT_IQ(SVC_FOC_ONE_POINT_FIVE))
        + PLAT_IQ_MPY(v_beta, PLAT_IQ(SVC_FOC_HALF_SQRT3));
    z = -PLAT_IQ_MPY(v_alpha, PLAT_IQ(SVC_FOC_ONE_POINT_FIVE))
        + PLAT_IQ_MPY(v_beta, PLAT_IQ(SVC_FOC_HALF_SQRT3));

    if (v_beta > 0) {
        sector = (uint8_t)(sector | 0x01U);
    }
    if ((PLAT_IQ_MPY(v_alpha, PLAT_IQ(SVC_FOC_SQRT3)) - v_beta) > 0) {
        sector = (uint8_t)(sector | 0x02U);
    }
    if ((-PLAT_IQ_MPY(v_alpha, PLAT_IQ(SVC_FOC_SQRT3)) - v_beta) > 0) {
        sector = (uint8_t)(sector | 0x04U);
    }

    switch (sector) {
    case 1U: t1 = -z; t2 = x;  break;
    case 2U: t1 = z;  t2 = y;  break;
    case 3U: t1 = x;  t2 = -y; break;
    case 4U: t1 = -x; t2 = z;  break;
    case 5U: t1 = -y; t2 = -z; break;
    case 6U: t1 = y;  t2 = -x; break;
    default: t1 = 0;  t2 = 0;  break;
    }

    ta = (PLAT_IQ(1.0f) - t1 - t2) / 2;

    switch (sector) {
    case 1U: duty_u = ta + t1 + t2; duty_v = ta + t2; duty_w = ta; break;
    case 2U: duty_v = ta + t1 + t2; duty_u = ta + t2; duty_w = ta; break;
    case 3U: duty_v = ta + t1 + t2; duty_w = ta + t2; duty_u = ta; break;
    case 4U: duty_w = ta + t1 + t2; duty_v = ta + t2; duty_u = ta; break;
    case 5U: duty_w = ta + t1 + t2; duty_u = ta + t2; duty_v = ta; break;
    case 6U: duty_u = ta + t1 + t2; duty_w = ta + t2; duty_v = ta; break;
    default: duty_u = ta; duty_v = ta; duty_w = ta; break;
    }

    p_duty_pu->duty_u.q_value = svc_foc_clamp_q15(duty_u);
    p_duty_pu->duty_v.q_value = svc_foc_clamp_q15(duty_v);
    p_duty_pu->duty_w.q_value = svc_foc_clamp_q15(duty_w);
    p_duty_pu->sector = sector;
}

/**
 * @brief 扇区法 SVPWM（浮点路径）：算法与 Q15 版本一一对应。
 * @param p_object FOC 服务对象指针。
 * @param p_alpha_beta_pu 输入 αβ 电压（标幺母线电压）。
 * @param p_duty_pu 输出三相标幺占空比与扇区号。
 */
static void svc_foc_svpwm_sector_float(const svc_foc_t *p_object,
                                       const svc_foc_alpha_beta_t *p_alpha_beta_pu,
                                       svc_foc_duty_pu_t *p_duty_pu)
{
    float v_alpha = p_alpha_beta_pu->alpha.float_value;
    float v_beta = p_alpha_beta_pu->beta.float_value;
    float x = SVC_FOC_SQRT3 * v_beta;
    float y = (SVC_FOC_ONE_POINT_FIVE * v_alpha) + (SVC_FOC_HALF_SQRT3 * v_beta);
    float z = (-SVC_FOC_ONE_POINT_FIVE * v_alpha) + (SVC_FOC_HALF_SQRT3 * v_beta);
    float t1 = 0.0f;
    float t2 = 0.0f;
    float ta;
    float duty_u;
    float duty_v;
    float duty_w;
    uint8_t sector = 0U;

    (void)p_object;

    if (v_beta > 0.0f) {
        sector = (uint8_t)(sector | 0x01U);
    }
    if (((SVC_FOC_SQRT3 * v_alpha) - v_beta) > 0.0f) {
        sector = (uint8_t)(sector | 0x02U);
    }
    if (((-SVC_FOC_SQRT3 * v_alpha) - v_beta) > 0.0f) {
        sector = (uint8_t)(sector | 0x04U);
    }

    switch (sector) {
    case 1U: t1 = -z; t2 = x;  break;
    case 2U: t1 = z;  t2 = y;  break;
    case 3U: t1 = x;  t2 = -y; break;
    case 4U: t1 = -x; t2 = z;  break;
    case 5U: t1 = -y; t2 = -z; break;
    case 6U: t1 = y;  t2 = -x; break;
    default: t1 = 0.0f; t2 = 0.0f; break;
    }

    ta = (1.0f - t1 - t2) * 0.5f;

    switch (sector) {
    case 1U: duty_u = ta + t1 + t2; duty_v = ta + t2; duty_w = ta; break;
    case 2U: duty_v = ta + t1 + t2; duty_u = ta + t2; duty_w = ta; break;
    case 3U: duty_v = ta + t1 + t2; duty_w = ta + t2; duty_u = ta; break;
    case 4U: duty_w = ta + t1 + t2; duty_v = ta + t2; duty_u = ta; break;
    case 5U: duty_w = ta + t1 + t2; duty_u = ta + t2; duty_v = ta; break;
    case 6U: duty_u = ta + t1 + t2; duty_w = ta + t2; duty_v = ta; break;
    default: duty_u = ta; duty_v = ta; duty_w = ta; break;
    }

    p_duty_pu->duty_u.float_value = svc_foc_clamp_float(duty_u);
    p_duty_pu->duty_v.float_value = svc_foc_clamp_float(duty_v);
    p_duty_pu->duty_w.float_value = svc_foc_clamp_float(duty_w);
    p_duty_pu->sector = sector;
}

/**
 * @brief 三次谐波注入法 SVPWM（Q15 定点路径）：标幺 αβ → 标幺占空比。
 * @details 由 αβ 逆 Clarke 求三相正弦，叠加零序分量 -(max+min)/2，再整体抬到以 0.5 为
 *          中心；与扇区法输出的线电压等价。
 * @param p_object FOC 服务对象指针。
 * @param p_alpha_beta_pu 输入 αβ 电压（标幺母线电压）。
 * @param p_duty_pu 输出三相标幺占空比（扇区号固定为 0）。
 */
static void svc_foc_svpwm_third_harmonic_q15(const svc_foc_t *p_object,
                                             const svc_foc_alpha_beta_t *p_alpha_beta_pu,
                                             svc_foc_duty_pu_t *p_duty_pu)
{
    svc_foc_abc_t phase;
    PLAT_IQ_TYPE phase_max;
    PLAT_IQ_TYPE phase_min;
    PLAT_IQ_TYPE offset;

    svc_foc_inverse_clarke_calc(p_object, p_alpha_beta_pu, &phase);

    phase_max = phase.phase_u.q_value;
    phase_min = phase.phase_u.q_value;
    if (phase.phase_v.q_value > phase_max) {
        phase_max = phase.phase_v.q_value;
    }
    if (phase.phase_v.q_value < phase_min) {
        phase_min = phase.phase_v.q_value;
    }
    if (phase.phase_w.q_value > phase_max) {
        phase_max = phase.phase_w.q_value;
    }
    if (phase.phase_w.q_value < phase_min) {
        phase_min = phase.phase_w.q_value;
    }

    offset = -((phase_max + phase_min) / 2);

    p_duty_pu->duty_u.q_value = svc_foc_clamp_q15(PLAT_IQ(0.5f)
                                                  + phase.phase_u.q_value + offset);
    p_duty_pu->duty_v.q_value = svc_foc_clamp_q15(PLAT_IQ(0.5f)
                                                  + phase.phase_v.q_value + offset);
    p_duty_pu->duty_w.q_value = svc_foc_clamp_q15(PLAT_IQ(0.5f)
                                                  + phase.phase_w.q_value + offset);
    p_duty_pu->sector = 0U;
}

/**
 * @brief 三次谐波注入法 SVPWM（浮点路径）：算法与 Q15 版本一一对应。
 * @param p_object FOC 服务对象指针。
 * @param p_alpha_beta_pu 输入 αβ 电压（标幺母线电压）。
 * @param p_duty_pu 输出三相标幺占空比（扇区号固定为 0）。
 */
static void svc_foc_svpwm_third_harmonic_float(const svc_foc_t *p_object,
                                               const svc_foc_alpha_beta_t *p_alpha_beta_pu,
                                               svc_foc_duty_pu_t *p_duty_pu)
{
    svc_foc_abc_t phase;
    float phase_max;
    float phase_min;
    float offset;

    svc_foc_inverse_clarke_calc(p_object, p_alpha_beta_pu, &phase);

    phase_max = phase.phase_u.float_value;
    phase_min = phase.phase_u.float_value;
    if (phase.phase_v.float_value > phase_max) {
        phase_max = phase.phase_v.float_value;
    }
    if (phase.phase_v.float_value < phase_min) {
        phase_min = phase.phase_v.float_value;
    }
    if (phase.phase_w.float_value > phase_max) {
        phase_max = phase.phase_w.float_value;
    }
    if (phase.phase_w.float_value < phase_min) {
        phase_min = phase.phase_w.float_value;
    }

    offset = -((phase_max + phase_min) * 0.5f);

    p_duty_pu->duty_u.float_value =
        svc_foc_clamp_float(0.5f + phase.phase_u.float_value + offset);
    p_duty_pu->duty_v.float_value =
        svc_foc_clamp_float(0.5f + phase.phase_v.float_value + offset);
    p_duty_pu->duty_w.float_value =
        svc_foc_clamp_float(0.5f + phase.phase_w.float_value + offset);
    p_duty_pu->sector = 0U;
}

/**
 * @brief SVPWM 计算（含母线电压归一化）：αβ 电压（V）→ 三相标幺占空比。
 * @details 先把伏特除以母线电压归一化成标幺母线电压（1.0 = Vdc），再按调制方式与运算
 *          类型分派到四个具体实现；母线电压非正时返回参数错误，避免除零。
 * @param p_object FOC 服务对象指针。
 * @param p_voltage_v 输入 αβ 电压（V）。
 * @param p_duty_pu 输出三相标幺占空比与扇区号。
 * @retval svc_foc_ok 操作成功。
 * @retval svc_foc_err_param 母线电压为零。
 * @retval svc_foc_err_unsupported 运算类型或调制方式不受支持。
 */
static svc_foc_status_enum svc_foc_svpwm_calc(const svc_foc_t *p_object,
                                              const svc_foc_alpha_beta_t *p_voltage_v,
                                              svc_foc_duty_pu_t *p_duty_pu)
{
    svc_foc_alpha_beta_t voltage_pu;

    if ((p_object->config.arithmetic_type != svc_foc_arithmetic_q15) &&
        (p_object->config.arithmetic_type != svc_foc_arithmetic_float)) {
        return svc_foc_err_unsupported;
    }

    /* 母线电压归一化：伏特 → 标幺母线电压 */
    if (svc_foc_is_q15(p_object)) {
        if (p_object->dc_bus_q <= 0) {
            return svc_foc_err_param;
        }
        voltage_pu.alpha.q_value = PLAT_IQ_DIV(p_voltage_v->alpha.q_value,
                                              p_object->dc_bus_q);
        voltage_pu.beta.q_value = PLAT_IQ_DIV(p_voltage_v->beta.q_value,
                                             p_object->dc_bus_q);
    } else {
        if (p_object->config.dc_bus_voltage_v <= 0.0f) {
            return svc_foc_err_param;
        }
        voltage_pu.alpha.float_value =
            p_voltage_v->alpha.float_value / p_object->config.dc_bus_voltage_v;
        voltage_pu.beta.float_value =
            p_voltage_v->beta.float_value / p_object->config.dc_bus_voltage_v;
    }

    switch (p_object->config.modulation) {
    case svc_foc_modulation_svpwm_sector:
        if (svc_foc_is_q15(p_object)) {
            svc_foc_svpwm_sector_q15(p_object, &voltage_pu, p_duty_pu);
        } else {
            svc_foc_svpwm_sector_float(p_object, &voltage_pu, p_duty_pu);
        }
        break;

    case svc_foc_modulation_svpwm_third_harmonic:
        if (svc_foc_is_q15(p_object)) {
            svc_foc_svpwm_third_harmonic_q15(p_object, &voltage_pu, p_duty_pu);
        } else {
            svc_foc_svpwm_third_harmonic_float(p_object, &voltage_pu, p_duty_pu);
        }
        break;

    default:
        return svc_foc_err_unsupported;
    }

    return svc_foc_ok;
}

/**
 * @brief 标幺占空比 → 比较值计数（0 ~ pwm_period），写入对象输出缓冲。
 * @param p_object FOC 服务对象指针。
 * @param p_duty_pu 输入三相标幺占空比与扇区号。
 * @param p_pwm 输出三相比较值计数与扇区号。
 */
static void svc_foc_duty_to_counts(const svc_foc_t *p_object,
                                   const svc_foc_duty_pu_t *p_duty_pu,
                                   svc_foc_pwm_t *p_pwm)
{
    if (svc_foc_is_q15(p_object)) {
        p_pwm->duty_u = (uint16_t)PLAT_IQ_MPY_I32_INT(p_duty_pu->duty_u.q_value,
                                                     (int32_t)p_object->pwm_period);
        p_pwm->duty_v = (uint16_t)PLAT_IQ_MPY_I32_INT(p_duty_pu->duty_v.q_value,
                                                     (int32_t)p_object->pwm_period);
        p_pwm->duty_w = (uint16_t)PLAT_IQ_MPY_I32_INT(p_duty_pu->duty_w.q_value,
                                                     (int32_t)p_object->pwm_period);
    } else {
        p_pwm->duty_u = (uint16_t)((p_duty_pu->duty_u.float_value
                                    * (float)p_object->pwm_period) + 0.5f);
        p_pwm->duty_v = (uint16_t)((p_duty_pu->duty_v.float_value
                                    * (float)p_object->pwm_period) + 0.5f);
        p_pwm->duty_w = (uint16_t)((p_duty_pu->duty_w.float_value
                                    * (float)p_object->pwm_period) + 0.5f);
    }

    p_pwm->sector = p_duty_pu->sector;
}

/**
 * @brief 初始化服务层：注入板级配置表、校验配置、缓存板级常量。
 * @param p_object FOC 服务对象指针。
 * @retval svc_foc_ok 操作成功。
 * @retval svc_foc_err_param 参数无效（PWM 满量程为 0 或母线电压非正）。
 * @retval svc_foc_err_unsupported 运算类型或调制方式未实现。
 */
svc_foc_status_enum svc_foc_init(svc_foc_t *p_object)
{
    if (p_object == NULL) {
        return svc_foc_err_param;
    }

    (void)memset(p_object, 0, sizeof(*p_object));
    p_object->config = s_svc_foc_config;

    if ((p_object->config.arithmetic_type != svc_foc_arithmetic_q15) &&
        (p_object->config.arithmetic_type != svc_foc_arithmetic_float)) {
        return svc_foc_err_unsupported;
    }
    if ((p_object->config.modulation != svc_foc_modulation_svpwm_sector) &&
        (p_object->config.modulation != svc_foc_modulation_svpwm_third_harmonic)) {
        return svc_foc_err_unsupported;
    }
    if ((p_object->config.pwm_period == 0U) ||
        (p_object->config.dc_bus_voltage_v <= 0.0f)) {
        return svc_foc_err_param;
    }

    /* 板级常量缓存：PWM 满量程与母线电压的 Q15 形式 */
    p_object->pwm_period = p_object->config.pwm_period;
    p_object->dc_bus_q = PLAT_IQ(p_object->config.dc_bus_voltage_v);

    p_object->state = svc_foc_state_ready;

    return svc_foc_ok;
}

/**
 * @brief 查询服务层状态。
 * @param p_object FOC 服务对象指针。
 * @return 当前状态；参数为空时返回 svc_foc_state_error。
 */
svc_foc_state_enum svc_foc_get_state(const svc_foc_t *p_object)
{
    if (p_object == NULL) {
        return svc_foc_state_error;
    }
    return p_object->state;
}

/**
 * @brief dq 轴电压（伏特）→ 三相 PWM 输出。
 * @details dq 入参是定点/浮点兼容的并集类型：调用方按 config.arithmetic_type 填好对应
 *          成员（Q15 填 q_value，1.0 = 1 V；浮点填 float_value，单位 V），本函数直接透传，
 *          不再做类型换算。内部链路：逆 Park(伏特) → 母线电压归一化 → SVPWM → 占空比
 *          × pwm_period → 比较值计数 → 经 BSP 下发三相 PWM。
 * @param p_object FOC 服务对象指针。
 * @param d_axis_v d 轴电压（V，按运算类型取并集对应成员）。
 * @param q_axis_v q 轴电压（V，按运算类型取并集对应成员）。
 * @param angle_pu 标幺电角度。
 * @retval svc_foc_ok 操作成功。
 * @retval svc_foc_err_param 参数无效或母线电压为 0。
 * @retval svc_foc_err_state 服务层未初始化或 BSP 三相 PWM 拒绝。
 * @retval svc_foc_err_unsupported 运算类型或调制方式不受支持。
 */
svc_foc_status_enum svc_foc_output_dq_pwm(svc_foc_t *p_object,
                                          svc_foc_value_t d_axis_v,
                                          svc_foc_value_t q_axis_v,
                                          svc_foc_value_t angle_pu)
{
    svc_foc_status_enum status;
    svc_foc_dq_t voltage_dq_v;
    svc_foc_duty_pu_t duty_pu;

    status = svc_foc_validate(p_object);
    if (status != svc_foc_ok) {
        return status;
    }

    /* dq 电压：调用方已按运算类型填入并集成员，这里直接透传 */
    voltage_dq_v.d = d_axis_v;
    voltage_dq_v.q = q_axis_v;
    p_object->voltage_dq = voltage_dq_v;

    /* 逆 Park：dq(V) → αβ(V) */
    svc_foc_inverse_park_calc(p_object, &voltage_dq_v, angle_pu,
                              &p_object->voltage_alpha_beta);

    /* SVPWM：αβ(V) → 母线电压归一化 → 三相标幺占空比 */
    status = svc_foc_svpwm_calc(p_object, &p_object->voltage_alpha_beta, &duty_pu);
    if (status != svc_foc_ok) {
        return status;
    }

    /* 标幺占空比 → 比较值计数，直接下发三相 PWM */
    svc_foc_duty_to_counts(p_object, &duty_pu, &p_object->pwm_output);

    if (bsp_motor_ctrl_ops->pf_set_duty(&bsp_motor_ctrl_three_phase,
                                       p_object->pwm_output.duty_u,
                                       p_object->pwm_output.duty_v,
                                       p_object->pwm_output.duty_w) != bsp_motor_ctrl_ok) {
        return svc_foc_err_state;
    }

    return svc_foc_ok;
}

/**
 * @brief 采样三相电流并更新 Clarke/Park 观测值。
 * @param p_object FOC 服务对象指针。
 * @param angle_pu 用于 Park 变换的标幺电角度。
 * @retval svc_foc_ok 操作成功。
 * @retval svc_foc_err_param 参数无效。
 * @retval svc_foc_err_state 服务层未初始化、电流采样不可用或采样失败。
 */
svc_foc_status_enum svc_foc_get_current(svc_foc_t *p_object, svc_foc_value_t angle_pu)
{
    svc_foc_status_enum status;

    status = svc_foc_validate(p_object);
    if (status != svc_foc_ok) {
        return status;
    }

    /* 电流采样对象未初始化或不支持三相重构时，本接口只报告状态，不改动观测值 */
    if ((bsp_current_sense_three_phase.state != bsp_current_sense_state_ready) &&
        (bsp_current_sense_three_phase.state != bsp_current_sense_state_running)) {
        return svc_foc_err_state;
    }
    if (bsp_current_sense_ops->pf_calc(&bsp_current_sense_three_phase)
        != bsp_current_sense_ok) {
        return svc_foc_err_state;
    }

    if (svc_foc_is_q15(p_object)) {
        p_object->current_abc.phase_u.q_value = bsp_current_sense_three_phase.current_q_u;
        p_object->current_abc.phase_v.q_value = bsp_current_sense_three_phase.current_q_v;
        p_object->current_abc.phase_w.q_value = bsp_current_sense_three_phase.current_q_w;
    } else {
        p_object->current_abc.phase_u.float_value =
            bsp_current_sense_three_phase.current_a_u;
        p_object->current_abc.phase_v.float_value =
            bsp_current_sense_three_phase.current_a_v;
        p_object->current_abc.phase_w.float_value =
            bsp_current_sense_three_phase.current_a_w;
    }

    /* 三相电流 → αβ → dq（纯观测，供上层上报/调试） */
    svc_foc_clarke_calc(p_object, &p_object->current_abc,
                        &p_object->current_alpha_beta);
    svc_foc_park_calc(p_object, &p_object->current_alpha_beta, angle_pu,
                      &p_object->current_dq);

    return svc_foc_ok;
}

/** @brief FOC 服务层函数表（本文件私有，只包含对外暴露的四个接口）。 */
static const svc_foc_ops_t s_svc_foc_ops = {
    .pf_init = svc_foc_init,
    .pf_get_state = svc_foc_get_state,
    .pf_output_dq_pwm = svc_foc_output_dq_pwm,
    .pf_get_current = svc_foc_get_current
};

/** @brief FOC 服务层函数表指针（绑定到本文件的私有函数表）。 */
const svc_foc_ops_t *const svc_foc_ops = &s_svc_foc_ops;
