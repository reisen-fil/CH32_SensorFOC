/**
 * @file svc_foc.h
 * @brief FOC 服务层接口（电流获取 + dq 电压 → 三相 PWM 输出）。
 * @details 本模块承担 FOC 链路的"运算 + 硬件下发"，对外只暴露四个接口：
 *          - svc_foc_init()：注入板级配置表（运算类型、调制方式、PWM 满量程、母线电压）；
 *          - svc_foc_get_state()：查询服务层状态；
 *          - svc_foc_output_dq_pwm()：dq 轴电压（**伏特**，定点/浮点兼容的并集入参）→ 逆 Park
 *            → SVPWM（含母线电压归一化）→ 三相占空比**计数**（0 ~ pwm_period）→ 经 BSP
 *            直接下发三相 PWM；
 *          - svc_foc_get_current()：采样三相电流 → Clarke → Park（按电角度）→ 更新对象中
 *            的 current_abc / current_alpha_beta / current_dq 供上层观测。
 *          Clarke/Park 正逆变换、SVPWM 调制、三角函数等全部是本文件内的 static 实现，
 *          不出现在头文件里；上层（app_foc_control）只调用上述四个接口。
 *
 *          量纲约定 —— **整条 FOC 链路只有电角度用标幺形式**：
 *          - 电角度：标幺值（0 ~ 1 对应 0 ~ 2π），Q15 时 1.0 = 32768；
 *          - dq/αβ 电压：**伏特**（V），SVPWM 内部除以 config.dc_bus_voltage_v 归一化；
 *          - 电流：**安培**（A，Q15 时 1.0 = 1 A，与 bsp_current_sense 一致）；
 *          - PWM 占空比：**计数**（0 ~ config.pwm_period），直接交给 bsp_motor_ctrl，
 *            中间不再经过百分比/标幺量。
 *
 *          运算类型（config.arithmetic_type）：
 *          - q15：全程 Q15 定点（PLAT_IQ_* / PLAT_IQ_SIN_PU / PLAT_IQ_COS_PU），无浮点累加；
 *          - float：全程单精度浮点，三角运算用标准库 sinf / cosf（链接需 -lm）。
 *          本模块不做任何 BSP 外设初始化，外设初始化由 app_system 集中调用。
 */
#ifndef __SVC_FOC_H__
#define __SVC_FOC_H__

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief FOC 服务层状态。 */
typedef enum {
    svc_foc_state_reset = 0,   /**< 未初始化（未注入配置） */
    svc_foc_state_ready,       /**< 已注入配置，接口可用 */
    svc_foc_state_error        /**< 配置非法或出错 */
} svc_foc_state_enum;

/** @brief FOC 服务接口调用返回的状态码。 */
typedef enum {
    svc_foc_ok = 0,                /**< 操作成功 */
    svc_foc_err_param = -1,        /**< 参数无效（含母线电压为 0） */
    svc_foc_err_state = -2,        /**< 服务层未初始化 / 电流采样不可用 / BSP 拒绝 */
    svc_foc_err_unsupported = -3   /**< 运算类型或调制方式不受支持 */
} svc_foc_status_enum;

/**
 * @brief FOC 运算类型。
 * @details 决定并集字段按哪一侧解释，以及三角运算走哪条路径：
 *          - q15：全程 Q15 定点（PLAT_IQ_* / PLAT_IQ_SIN_PU / PLAT_IQ_COS_PU）；
 *          - float：全程单精度浮点，三角运算走标准库 sinf / cosf。
 */
typedef enum {
    svc_foc_arithmetic_q15 = 0,  /**< Q15 定点运算 */
    svc_foc_arithmetic_float     /**< 单精度浮点运算 */
} svc_foc_arithmetic_enum;

/**
 * @brief SVPWM 计算方式。
 * @details 两者输出的线电压一致，区别只在实现路径：
 *          - sector：先由 αβ 判断扇区，再算 T1/T2 与三相切换点；
 *          - third_harmonic：由 αβ 求三相正弦，叠加 -(max+min)/2 的零序分量。
 */
typedef enum {
    svc_foc_modulation_svpwm_sector = 0,      /**< 扇区法 */
    svc_foc_modulation_svpwm_third_harmonic   /**< 三次谐波注入法（min-max 零序注入） */
} svc_foc_modulation_enum;

/**
 * @brief 标幺值：同一存储按 config.arithmetic_type 解释为 Q15 或 float。
 * @note 有效成员由运算类型决定；写反了不会报错，只会得到错误的数值。
 */
typedef union {
    PLAT_IQ_TYPE q_value;   /**< Q15 定点值（1.0 = 32768） */
    float float_value;      /**< 单精度浮点值 */
} svc_foc_value_t;

/** @brief 三相量（U/V/W）：电流为安培，电压为伏特。 */
typedef struct {
    svc_foc_value_t phase_u;   /**< U 相 */
    svc_foc_value_t phase_v;   /**< V 相 */
    svc_foc_value_t phase_w;   /**< W 相 */
} svc_foc_abc_t;

/** @brief 静止两相量（Alpha/Beta）。 */
typedef struct {
    svc_foc_value_t alpha;   /**< α 轴分量 */
    svc_foc_value_t beta;    /**< β 轴分量 */
} svc_foc_alpha_beta_t;

/** @brief 旋转两相量（d/q）。 */
typedef struct {
    svc_foc_value_t d;   /**< d 轴分量（励磁） */
    svc_foc_value_t q;   /**< q 轴分量（转矩） */
} svc_foc_dq_t;

/**
 * @brief 三相 PWM 输出：**占空比计数**（0 ~ pwm_period）+ 扇区号。
 * @details 计数直接来自 SVPWM 计算结果，不再经过百分比换算；bsp_motor_ctrl 的
 *          pf_set_duty() 按同一计数口径接收。
 */
typedef struct {
    uint16_t duty_u;   /**< U 相比较值（计数，0 ~ pwm_period） */
    uint16_t duty_v;   /**< V 相比较值（计数，0 ~ pwm_period） */
    uint16_t duty_w;   /**< W 相比较值（计数，0 ~ pwm_period） */
    uint8_t sector;    /**< 扇区号 1 ~ 6；三次谐波注入法固定为 0 */
} svc_foc_pwm_t;

/**
 * @brief FOC 服务层配置表（由本文件内的板级常量 s_svc_foc_config 提供）。
 * @details - arithmetic_type：运算类型（本板取 Q15 定点）；
 *          - modulation：SVPWM 计算方式（本板取扇区法）；
 *          - pwm_period：PWM 满量程计数（= HAL_TIM_PWM_PERIOD，与 bsp_motor_ctrl 一致），
 *            标幺占空比按它换算成比较值；
 *          - dc_bus_voltage_v：母线电压（V），SVPWM 用它把伏特归一化成标幺占空比。
 */
typedef struct {
    svc_foc_arithmetic_enum arithmetic_type;   /**< 运算类型：Q15 / 浮点 */
    svc_foc_modulation_enum modulation;        /**< SVPWM 计算方式：扇区法 / 三次谐波 */

    uint16_t pwm_period;                       /**< PWM 满量程计数（与 BSP 三相 PWM 一致） */
    float dc_bus_voltage_v;                    /**< 母线电压（V），用于电压归一化 */

    void *p_user_data;                         /**< 上层私有数据 */
} svc_foc_config_t;

/**
 * @brief FOC 服务层对象：状态 + 配置 + 观测量。
 * @details 不保存下层对象指针——电流采样与三相 PWM 板级单例
 *          （bsp_current_sense_three_phase / bsp_motor_ctrl_three_phase）由本文件直接引用。
 *          观测字段（current_* / voltage_* / pwm_output）每次调用相应接口后刷新，便于调试
 *          与上层上报；并集值的有效成员由 config.arithmetic_type 决定。
 */
typedef struct {
    svc_foc_state_enum state;    /**< 服务层状态 */
    svc_foc_config_t config;     /**< 板级配置（init 时从配置表注入） */

    uint16_t pwm_period;         /**< PWM 满量程计数（init 时取自 config） */
    PLAT_IQ_TYPE dc_bus_q;       /**< 母线电压的 Q15 缓存（避免运行期浮点换算） */

    svc_foc_abc_t current_abc;               /**< 最近一次三相电流（A） */
    svc_foc_alpha_beta_t current_alpha_beta; /**< 最近一次 Clarke 结果（A） */
    svc_foc_dq_t current_dq;                 /**< 最近一次 Park 结果（A） */
    svc_foc_dq_t voltage_dq;                 /**< 最近一次下发的 dq 电压（V） */
    svc_foc_alpha_beta_t voltage_alpha_beta; /**< 最近一次逆 Park 结果（V） */
    svc_foc_pwm_t pwm_output;                /**< 最近一次下发的三相占空比计数 + 扇区 */
} svc_foc_t;

/** @brief 板级 FOC 服务对象（单实例）。 */
extern svc_foc_t svc_foc_motor;

/**
 * @brief 初始化服务层：注入板级配置表、校验配置、缓存板级常量。
 * @note 纯配置接口，不访问也不初始化任何 BSP 外设（由 app_system 集中初始化）。
 * @param p_object FOC 服务对象指针。
 * @retval svc_foc_ok 操作成功。
 * @retval svc_foc_err_param 参数无效（PWM 满量程为 0 或母线电压非正）。
 * @retval svc_foc_err_unsupported 运算类型或调制方式未实现。
 */
svc_foc_status_enum svc_foc_init(svc_foc_t *p_object);

/**
 * @brief 查询服务层状态。
 * @param p_object FOC 服务对象指针。
 * @return 当前状态；参数为空时返回 svc_foc_state_error。
 */
svc_foc_state_enum svc_foc_get_state(const svc_foc_t *p_object);

/**
 * @brief dq 轴电压（伏特）→ 三相 PWM 输出。
 * @details 计算链路：
 *          1) 由 dq 电压做逆 Park 得 αβ 电压（V）；
 *          2) SVPWM：αβ 除以母线电压归一化 → 按 config.modulation 算三相占空比
 *             （扇区法或三次谐波注入法，均限幅到 [0, 1]）；
 *          3) 占空比 × pwm_period 得比较值计数，经 bsp_motor_ctrl_ops->pf_set_duty() 下发。
 *          中间结果（voltage_dq / voltage_alpha_beta / pwm_output）同步存进对象便于观测。
 *          入参用 svc_foc_value_t，兼容定点与浮点：调用方按 config.arithmetic_type 填
 *          对应成员（Q15 填 q_value，单位 1.0 = 1 V；浮点填 float_value，单位 V）。
 * @param p_object FOC 服务对象指针。
 * @param d_axis_v d 轴电压（V，按运算类型取并集对应成员）。
 * @param q_axis_v q 轴电压（V，按运算类型取并集对应成员）。
 * @param angle_pu 标幺电角度（0 ~ 1，按运算类型取并集的对应成员）。
 * @retval svc_foc_ok 操作成功，已下发新的三相占空比。
 * @retval svc_foc_err_param 参数无效或母线电压为 0。
 * @retval svc_foc_err_state 服务层未初始化或 BSP 三相 PWM 拒绝。
 * @retval svc_foc_err_unsupported 运算类型或调制方式不受支持。
 */
svc_foc_status_enum svc_foc_output_dq_pwm(svc_foc_t *p_object,
                                          svc_foc_value_t d_axis_v,
                                          svc_foc_value_t q_axis_v,
                                          svc_foc_value_t angle_pu);

/**
 * @brief 采样三相电流并更新 Clarke/Park 观测值。
 * @details 电流采样对象未初始化（状态非 ready/running）或采样失败时返回
 *          svc_foc_err_state，此时对象内的 current_* 字段保持上一次的值——本接口只做观测，
 *          失败不应影响上层的输出决策。
 * @param p_object FOC 服务对象指针。
 * @param angle_pu 用于 Park 变换的标幺电角度（0 ~ 1）。
 * @retval svc_foc_ok 操作成功，current_abc / current_alpha_beta / current_dq 已刷新。
 * @retval svc_foc_err_param 参数无效。
 * @retval svc_foc_err_state 服务层未初始化、电流采样不可用或采样失败。
 */
svc_foc_status_enum svc_foc_get_current(svc_foc_t *p_object, svc_foc_value_t angle_pu);

/**
 * @brief FOC 服务层函数表：初始化 / 状态获取 / dq→PWM 输出 / 电流获取。
 * @details 表中的函数指针在 svc_foc.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 svc_foc_ops 统一调用。
 */
typedef struct {
    /** @brief 初始化（见 svc_foc_init）。 */
    svc_foc_status_enum (*pf_init)(svc_foc_t *p_object);
    /** @brief 状态查询（见 svc_foc_get_state）。 */
    svc_foc_state_enum (*pf_get_state)(const svc_foc_t *p_object);
    /** @brief dq 电压 → 三相 PWM 输出（见 svc_foc_output_dq_pwm）。 */
    svc_foc_status_enum (*pf_output_dq_pwm)(svc_foc_t *p_object,
                                            svc_foc_value_t d_axis_v,
                                            svc_foc_value_t q_axis_v,
                                            svc_foc_value_t angle_pu);
    /** @brief 三相电流获取（见 svc_foc_get_current）。 */
    svc_foc_status_enum (*pf_get_current)(svc_foc_t *p_object, svc_foc_value_t angle_pu);
} svc_foc_ops_t;

/** @brief FOC 服务层函数表指针（在 svc_foc.c 中定义并绑定）。 */
extern const svc_foc_ops_t *const svc_foc_ops;

#ifdef __cplusplus
}
#endif

#endif /* __SVC_FOC_H__ */
