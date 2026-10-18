/**
 * @file app_foc_control.h
 * @brief 应用层 FOC 控制接口（零点标定 + 开环 Uq 控制）。
 * @details 本模块只做**控制业务**，FOC 运算与 PWM 下发全部经服务层函数表 svc_foc_ops：
 *          - 零点标定（整合为单一入口）：首次调用自启动并施加 d 轴矢量（电角度 0）→
 *            保持 zero_align_time_us → 按固定间隔连续采集 zero_align_sample_count 次
 *            编码器机械角并求平均 → 反算并写回零点偏置 → 撤掉矢量；全程非阻塞，由上层
 *            按 elapsed_us 周期调用同一函数推进，无需单独的启动接口；内部由通用状态机
 *            服务 svc_fsm 驱动（energizing → sampling → done 三个状态，状态函数见 .c）；
 *          - 开环 Uq 控制：电角度自积分 → 调 svc_foc_ops->pf_get_current() 观测电流 →
 *            调 svc_foc_ops->pf_output_dq_pwm() 输出电压并下发三相 PWM；
 *          - 对外接口：初始化、零点标定、开环控制（运行 + 参数设置）、状态查询；
 *            **无单独失能接口**，需要停止输出时由上层调用
 *            app_foc_control_set_open_loop(obj, 0, freq) 或直接停 BSP 三相 PWM。
 *          硬件访问只保留编码器读取（bsp_encoder_mt6701_ops）与零点偏置写回；电流采样、
 *          SVPWM、占空比下发都在服务层完成。本模块**不初始化任何 BSP 外设**，外设初始化
 *          由 app_system 集中调用。
 *
 *          量纲约定（与服务层一致）：dq 电压用**伏特**，电角度用**标幺值**（0 ~ 1），
 *          电流用**安培**；观测值统一从 svc_foc_motor 的观测字段读取。
 */
#ifndef __APP_FOC_CONTROL_H__
#define __APP_FOC_CONTROL_H__

#include "svc_fsm.h"
#include "svc_foc.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 应用 FOC 控制对象的生命周期状态。 */
typedef enum {
    app_foc_control_state_reset = 0,   /**< 未初始化 */
    app_foc_control_state_ready,       /**< 已初始化，待运行 */
    app_foc_control_state_running,     /**< 正在输出（零点标定或开环控制运行中） */
    app_foc_control_state_error        /**< 出错 */
} app_foc_control_state_enum;

/** @brief 应用 FOC 控制接口执行结果。 */
typedef enum {
    app_foc_control_ok = 0,                /**< 操作成功 */
    app_foc_control_err_param = -1,        /**< 参数无效 */
    app_foc_control_err_state = -2,        /**< 对象状态不允许该操作 / 服务层或 BSP 拒绝 */
    app_foc_control_err_unsupported = -3,  /**< 服务层运算类型或调制方式不受支持 */
    app_foc_control_err_not_aligned = -4,  /**< 尚未完成零点标定，开环控制不可用 */
    app_foc_control_err_timeout = -5       /**< 预留：标定超时 */
} app_foc_control_status_enum;

/** @brief 零点标定状态机的阶段。 */
typedef enum {
    app_foc_control_align_idle = 0,   /**< 尚未开始（下次调用自动启动） */
    app_foc_control_align_energizing, /**< 已施加对齐矢量，正在保持计时 */
    app_foc_control_align_sampling,   /**< 保持结束，正在按间隔多次采样机械角 */
    app_foc_control_align_done,       /**< 标定完成，零点偏置已写回编码器对象 */
    app_foc_control_align_error       /**< 标定过程出错 */
} app_foc_control_align_state_enum;

/**
 * @brief 应用 FOC 控制板级配置表（由本文件内的板级常量 s_app_foc_control_config 提供）。
 * @details 只放控制业务参数；PWM 满量程、母线电压、运算类型、调制方式在服务层配置表里
 *          （svc_foc.c 的 s_svc_foc_config），编码器极对数/方向等硬件事实在 bsp_encoder 里。
 *          - encoder_resolution_bits：编码器单圈位数（MT6701 为 14 位），零点偏置按它取模；
 *          - zero_align_voltage_v / zero_align_time_us：零点标定施加的 d 轴电压（V）与
 *            保持时间；
 *          - zero_align_sample_count / zero_align_sample_interval_us：保持结束后连续采样
 *            机械角的次数与间隔（采样窗口 = 次数 × 间隔），均值即零点位置；
 *          - open_loop_uq_v / open_loop_frequency_millihz：开环 Uq（V）与电频率（mHz，
 *            带符号表示转向），可经 app_foc_control_set_open_loop() 运行中修改。
 */
typedef struct {
    uint8_t encoder_resolution_bits;  /**< 编码器单圈分辨率位数（MT6701 为 14） */

    float zero_align_voltage_v;       /**< 零点标定施加的 d 轴电压（V） */
    uint32_t zero_align_time_us;      /**< 对齐矢量保持时间（µs） */
    uint16_t zero_align_sample_count; /**< 求均值的采样次数（1 ~ 64） */
    uint32_t zero_align_sample_interval_us; /**< 采样间隔（µs） */

    float open_loop_uq_v;             /**< 开环 Uq（V） */
    int32_t open_loop_frequency_millihz; /**< 开环电频率（mHz，带符号 = 转向） */

    void *p_user_data;                /**< 上层私有数据 */
} app_foc_control_config_t;

/**
 * @brief 应用 FOC 控制对象：状态 + 配置 + 标定/开环运行量。
 * @details 不保存下层对象指针——编码器单例 bsp_encoder_mt6701 和 FOC 服务对象
 *          svc_foc_motor 由本文件直接引用；电流、电压、占空比等观测值存放在
 *          svc_foc_motor 的观测字段中，本对象只保留控制过程自身的运行量。
 */
typedef struct {
    app_foc_control_state_enum state;   /**< 对象生命周期状态 */
    app_foc_control_config_t config;    /**< 板级配置（init 时从配置表注入） */

    uint16_t encoder_resolution;        /**< 编码器单圈计数（init 时由 config 位数算出） */

    app_foc_control_align_state_enum zero_align_state; /**< 零点标定阶段（由状态机同步） */
    svc_fsm_t zero_align_fsm;                /**< 零点标定状态机（svc_fsm 框架实例） */
    uint32_t zero_align_sample_timer_us;     /**< 距上次采样累积的时间（µs） */
    uint16_t zero_align_sample_index;        /**< 已完成的采样次数 */
    uint8_t zero_align_retry_count;          /**< 连续采样失败次数（超限则判错） */
    uint32_t zero_align_reference_counts;    /**< 首次采样的机械角（求均值参考点） */
    int32_t zero_align_delta_sum;            /**< 各次采样相对参考点的最短弧差累加和 */
    uint32_t zero_offset_counts;             /**< 标定得到的零点偏置（编码器计数） */
    bool is_zero_aligned;                    /**< 零点标定完成标志 */

    PLAT_IQ_TYPE open_loop_angle_pu_q;   /**< 开环电角度积分值（Q15 标幺，0 ~ 1） */
} app_foc_control_t;

/** @brief 板级应用 FOC 控制对象（单实例）。 */
extern app_foc_control_t app_foc_control_motor;

/**
 * @brief 初始化应用 FOC 控制对象：注入配置、缓存编码器分辨率、初始化服务层。
 * @details 不初始化任何 BSP 外设（由 app_system 集中初始化），因此不检查 BSP 状态。
 * @param p_object 应用 FOC 控制对象指针。
 * @retval app_foc_control_ok 操作成功。
 * @retval app_foc_control_err_param 参数无效或配置非法。
 * @retval app_foc_control_err_unsupported 服务层运算类型或调制方式不受支持。
 */
app_foc_control_status_enum app_foc_control_init(app_foc_control_t *p_object);

/**
 * @brief 零点标定（整合入口，非阻塞）：首次调用自启动并施加对齐矢量，之后每次调用推进
 *        一步，直到阶段变为 done，期间的错误由返回值上报。
 * @details 内部由 svc_fsm 状态机驱动，三个阶段各对应一个状态函数（见 .c 的状态表）：
 *          1) energizing（enter 施加对齐矢量：电角度 0、Ud = zero_align_voltage_v、Uq = 0；
 *             run 用 svc_fsm_get_state_elapsed_us() 判断保持时间到 → 请求 sampling）；
 *          2) sampling（enter 复位采样统计；run 每 zero_align_sample_interval_us 采一次
 *             编码器机械角，以首次采样为参考点按**最短弧差**累加，避免 0/单圈计数 边界
 *             跨零导致均值失真；采满 zero_align_sample_count 次 → 请求 done）；
 *          3) done（enter 求平均得到零点位置，按 BSP 公式 zero_aligned = (raw + zero_offset)
 *             & mask 反算 zero_offset = (单圈计数 - 平均机械角) mod 单圈计数，写回
 *             bsp_encoder_mt6701.config 并置 is_zero_aligned，刷新电角度，最后以
 *             dq = (0, 0) 撤掉对齐矢量，三相占空比回到 50%）。
 * @note 编码器单次 CRC 失败不致命：本次采样作废，下次间隔重试；连续失败超过上限则判错。
 * @param p_object 应用 FOC 控制对象指针。
 * @param elapsed_us 距上次调用的时间（µs），用于保持计时与采样间隔计时。
 * @retval app_foc_control_ok 操作成功（含"已完成"的空操作）。
 * @retval app_foc_control_err_param 参数无效。
 * @retval app_foc_control_err_state 对象未就绪、编码器采样失败或服务层/PWM 下发失败。
 * @retval app_foc_control_err_unsupported 服务层运算类型或调制方式不受支持。
 */
app_foc_control_status_enum app_foc_control_zero_calibrate(app_foc_control_t *p_object,
                                                           uint32_t elapsed_us);

/**
 * @brief 开环 Uq 控制：电角度按 config.open_loop_frequency_millihz 自积分，
 *        dq = (0, open_loop_uq) → 服务层逆 Park + SVPWM → 下发三相 PWM。
 * @details 角度积分在 Q15 定点域完成（Δ = f × Δt × 32768 / 1e9，四舍五入），起点为 0，
 *          即零点标定得到的电角度零点，故本接口必须在标定完成后调用。
 *          同时调用服务层电流获取接口刷新 current_* 观测值（失败不影响输出）。
 * @param p_object 应用 FOC 控制对象指针。
 * @param elapsed_us 距上次调用的时间（µs），用于角度积分。
 * @retval app_foc_control_ok 操作成功，已下发新的三相占空比。
 * @retval app_foc_control_err_param 参数无效。
 * @retval app_foc_control_err_state 对象未就绪或服务层/PWM 拒绝。
 * @retval app_foc_control_err_not_aligned 尚未完成零点标定。
 */
app_foc_control_status_enum app_foc_control_open_loop_uq(app_foc_control_t *p_object,
                                                         uint32_t elapsed_us);

/**
 * @brief 运行中修改开环参数。
 * @param p_object 应用 FOC 控制对象指针。
 * @param uq_v 开环 Uq（V）。
 * @param frequency_millihz 开环电频率（mHz，带符号表示转向）。
 * @retval app_foc_control_ok 操作成功。
 * @retval app_foc_control_err_param 参数无效。
 */
app_foc_control_status_enum app_foc_control_set_open_loop(app_foc_control_t *p_object,
                                                          float uq_v,
                                                          int32_t frequency_millihz);

/**
 * @brief 查询应用 FOC 控制对象当前状态。
 * @param p_object 应用 FOC 控制对象指针。
 * @return 当前状态；参数为空时返回 app_foc_control_state_error。
 */
app_foc_control_state_enum app_foc_control_get_state(const app_foc_control_t *p_object);

/**
 * @brief 应用 FOC 控制函数表：初始化 / 零点标定 / 开环控制 / 状态查询。
 * @details 表中的函数指针在 app_foc_control.c 中绑定，函数表本体对该文件私有；
 *          调用方既可直接调用同名函数，也可经由 app_foc_control_ops 统一调用。
 */
typedef struct {
    /** @brief 初始化（见 app_foc_control_init）。 */
    app_foc_control_status_enum (*pf_init)(app_foc_control_t *p_object);
    /** @brief 零点标定（见 app_foc_control_zero_calibrate）。 */
    app_foc_control_status_enum (*pf_zero_calibrate)(app_foc_control_t *p_object,
                                                     uint32_t elapsed_us);
    /** @brief 开环 Uq 控制（见 app_foc_control_open_loop_uq）。 */
    app_foc_control_status_enum (*pf_open_loop_uq)(app_foc_control_t *p_object,
                                                   uint32_t elapsed_us);
    /** @brief 修改开环参数（见 app_foc_control_set_open_loop）。 */
    app_foc_control_status_enum (*pf_set_open_loop)(app_foc_control_t *p_object,
                                                    float uq_v,
                                                    int32_t frequency_millihz);
    /** @brief 查询对象状态（见 app_foc_control_get_state）。 */
    app_foc_control_state_enum (*pf_get_state)(const app_foc_control_t *p_object);
} app_foc_control_ops_t;

/** @brief 应用 FOC 控制函数表指针（在 app_foc_control.c 中定义并绑定）。 */
extern const app_foc_control_ops_t *const app_foc_control_ops;

#ifdef __cplusplus
}
#endif

#endif /* __APP_FOC_CONTROL_H__ */
