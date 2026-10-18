/**
 * @file svc_fsm.h
 * @brief 有限状态机服务接口（通用、与硬件无关）。
 * @details 表驱动的状态机框架：
 *          - 状态表是 static const 的 svc_fsm_state_t 数组，**状态号即数组下标**；
 *          - 每个状态可选提供 enter / run / exit 三个回调，其中 run 是周期推进函数，
 *            传给它的是"本次调用周期时长"，状态累计时间由框架记录并可查询；
 *          - 状态切换统一走 svc_fsm_request()（请求式）：框架在本拍 run 返回后立即执行
 *            exit → enter，并清零该状态计时；enter 内也可以继续请求切换；
 *          - 框架自身只有 reset / ready / running / error 四个生命周期状态，业务阶段
 *            由状态号表达；
 *          - 回调返回负值即视为该状态出错，框架置 error 并把错误码回传给调用方。
 *          本模块是**纯逻辑层**：不访问任何硬件、不依赖 platform/hal/bsp，可被任意上层
 *          （app/service/bsp）直接把对象嵌进自己的结构体里注入使用。
 *
 *          典型用法：
 *          @code
 *          static svc_fsm_status_enum xxx_run(svc_fsm_t *p_fsm, void *p_context,
 *                                            uint32_t elapsed_us);
 *          static const svc_fsm_state_t s_xxx_states[] = {
 *              { .p_name = "a", .pf_enter = ..., .pf_run = xxx_run, .pf_exit = NULL },
 *              { .p_name = "b", .pf_run = NULL },            // 终态：无周期动作
 *          };
 *          static const svc_fsm_config_t s_xxx_fsm_config = {
 *              .p_states = s_xxx_states,
 *              .state_count = 2U,
 *              .initial_state_index = 0U,
 *              .p_context = NULL,                            // init 时按实例绑定
 *          };
 *          @endcode
 */
#ifndef __SVC_FSM_H__
#define __SVC_FSM_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 无效状态号（表示"无切换请求"或"未进入任何状态"）。 */
#define SVC_FSM_STATE_NONE                 (0xFFFFU)

/** @brief 状态机框架自身的生命周期状态。 */
typedef enum {
    svc_fsm_state_reset = 0,   /**< 未初始化 */
    svc_fsm_state_ready,       /**< 已初始化并进入初始状态，尚未推进 */
    svc_fsm_state_running,     /**< 已被推进过 */
    svc_fsm_state_error        /**< 某个状态回调返回错误 */
} svc_fsm_state_enum;

/** @brief 状态机服务接口执行结果。 */
typedef enum {
    svc_fsm_ok = 0,                /**< 操作成功 */
    svc_fsm_err_param = -1,        /**< 参数无效 */
    svc_fsm_err_state = -2,        /**< 生命周期状态不允许该操作 */
    svc_fsm_err_unsupported = -3,  /**< 状态表缺少必要信息 */
    svc_fsm_err_timeout = -4       /**< 预留：超时 */
} svc_fsm_status_enum;

typedef struct svc_fsm svc_fsm_t;

/**
 * @brief 状态回调原型。
 * @param p_fsm 状态机对象指针（回调内可调用 svc_fsm_request 请求切换）。
 * @param p_context 状态机绑定的上层上下文（通常是宿主对象）。
 * @param elapsed_us 本次调用的周期时长（µs）；current 状态累计时间用
 *                   svc_fsm_get_state_elapsed_us() 查。
 * @return svc_fsm_ok 表示正常；负值表示该状态出错，框架将转入 error。
 */
typedef svc_fsm_status_enum (*svc_fsm_action_t)(svc_fsm_t *p_fsm,
                                                void *p_context,
                                                uint32_t elapsed_us);

/**
 * @brief 状态描述：状态号即它在状态表中的下标。
 * @details pf_enter / pf_run / pf_exit 均可为 NULL：pf_run 为 NULL 表示该状态没有周期
 *          动作（常用于终态），框架仍会正常计时。
 */
typedef struct {
    const char *p_name;      /**< 状态名（调试用，可为 NULL） */
    svc_fsm_action_t pf_enter; /**< 进入状态时调用（可为 NULL） */
    svc_fsm_action_t pf_run;   /**< 周期推进时调用（可为 NULL） */
    svc_fsm_action_t pf_exit;  /**< 离开状态时调用（可为 NULL） */
} svc_fsm_state_t;

/** @brief 状态机配置：状态表 + 初始状态 + 上层上下文。 */
typedef struct {
    const svc_fsm_state_t *p_states;   /**< 状态表首地址（须为静态存储期） */
    uint16_t state_count;              /**< 状态个数 */
    uint16_t initial_state_index;      /**< 初始状态号（0 ~ state_count-1） */
    void *p_context;                   /**< 传给各状态回调的上层上下文 */
} svc_fsm_config_t;

/**
 * @brief 状态机服务对象。
 * @details 可嵌进上层结构体按值持有（本框架不用堆、不用单例）。
 */
struct svc_fsm {
    svc_fsm_state_enum state;          /**< 框架生命周期状态 */
    svc_fsm_config_t config;           /**< 状态表与上下文（init 时注入） */

    uint16_t current_state_index;      /**< 当前状态号 */
    uint16_t previous_state_index;     /**< 上一个状态号（调试用） */
    uint16_t pending_state_index;      /**< 待生效的切换请求，SVC_FSM_STATE_NONE 表示无 */
    uint32_t state_elapsed_us;         /**< 当前状态已运行时间（µs，切换时清零） */
    uint32_t transition_count;         /**< 累计切换次数（调试用） */
    svc_fsm_status_enum last_status;   /**< 最近一次状态回调返回值 */
};

/**
 * @brief 初始化状态机：注入配置并进入初始状态（调用其 pf_enter）。
 * @param p_object 状态机对象指针。
 * @param p_config 状态机配置（会拷贝进对象）。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效或状态表/初始状态非法。
 * @retval svc_fsm_err_state 初始状态进入失败（对象已置 error）。
 */
svc_fsm_status_enum svc_fsm_init(svc_fsm_t *p_object, const svc_fsm_config_t *p_config);

/**
 * @brief 请求切换到指定状态。
 * @details 请求不会立即打断当前回调：框架在当前状态的 pf_run 返回后（或下一拍 run 前）
 *          执行 exit → enter。请求与当前状态相同则视为空操作。
 * @param p_object 状态机对象指针。
 * @param state_index 目标状态号。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效或状态号越界。
 * @retval svc_fsm_err_state 状态机未初始化或已出错。
 */
svc_fsm_status_enum svc_fsm_request(svc_fsm_t *p_object, uint16_t state_index);

/**
 * @brief 按周期时长推进一次：执行当前状态的 pf_run，并处理切换请求。
 * @param p_object 状态机对象指针。
 * @param elapsed_us 本次调用距上次调用的时长（µs）。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效。
 * @retval svc_fsm_err_state 状态机未初始化或已出错。
 * @retval 其他负值：当前状态回调返回的错误码（对象已置 error）。
 */
svc_fsm_status_enum svc_fsm_run(svc_fsm_t *p_object, uint32_t elapsed_us);

/**
 * @brief 复位状态机：退出当前状态并重新进入初始状态（便于反复使用同一对象）。
 * @param p_object 状态机对象指针。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效。
 * @retval svc_fsm_err_state 初始状态进入失败（对象已置 error）。
 */
svc_fsm_status_enum svc_fsm_reset(svc_fsm_t *p_object);

/**
 * @brief 查询框架生命周期状态。
 * @param p_object 状态机对象指针。
 * @return 生命周期状态；参数为空时返回 svc_fsm_state_error。
 */
svc_fsm_state_enum svc_fsm_get_state(const svc_fsm_t *p_object);

/**
 * @brief 查询当前状态号。
 * @param p_object 状态机对象指针。
 * @return 当前状态号；参数为空或未初始化时返回 SVC_FSM_STATE_NONE。
 */
uint16_t svc_fsm_get_state_index(const svc_fsm_t *p_object);

/**
 * @brief 查询当前状态已运行时间（自进入该状态起累计）。
 * @param p_object 状态机对象指针。
 * @return 累计时间（µs）；参数为空时返回 0。
 */
uint32_t svc_fsm_get_state_elapsed_us(const svc_fsm_t *p_object);

/**
 * @brief 判断当前是否处于指定状态。
 * @param p_object 状态机对象指针。
 * @param state_index 待比较的状态号。
 * @return true 表示当前状态号与之一致。
 */
bool svc_fsm_is_state(const svc_fsm_t *p_object, uint16_t state_index);

/**
 * @brief 状态机服务函数表：初始化 / 请求切换 / 周期推进 / 复位 / 查询。
 * @details 函数表本体在 svc_fsm.c 中私有，调用方既可直接调用同名函数，也可经
 *          svc_fsm_ops 统一调用。
 */
typedef struct {
    /** @brief 初始化（见 svc_fsm_init）。 */
    svc_fsm_status_enum (*pf_init)(svc_fsm_t *p_object, const svc_fsm_config_t *p_config);
    /** @brief 请求切换状态（见 svc_fsm_request）。 */
    svc_fsm_status_enum (*pf_request)(svc_fsm_t *p_object, uint16_t state_index);
    /** @brief 周期推进（见 svc_fsm_run）。 */
    svc_fsm_status_enum (*pf_run)(svc_fsm_t *p_object, uint32_t elapsed_us);
    /** @brief 复位到初始状态（见 svc_fsm_reset）。 */
    svc_fsm_status_enum (*pf_reset)(svc_fsm_t *p_object);
    /** @brief 查询生命周期状态（见 svc_fsm_get_state）。 */
    svc_fsm_state_enum (*pf_get_state)(const svc_fsm_t *p_object);
    /** @brief 查询当前状态号（见 svc_fsm_get_state_index）。 */
    uint16_t (*pf_get_state_index)(const svc_fsm_t *p_object);
    /** @brief 查询当前状态已运行时间（见 svc_fsm_get_state_elapsed_us）。 */
    uint32_t (*pf_get_state_elapsed_us)(const svc_fsm_t *p_object);
    /** @brief 判断是否处于指定状态（见 svc_fsm_is_state）。 */
    bool (*pf_is_state)(const svc_fsm_t *p_object, uint16_t state_index);
} svc_fsm_ops_t;

/** @brief 状态机服务函数表指针（在 svc_fsm.c 中定义并绑定）。 */
extern const svc_fsm_ops_t *const svc_fsm_ops;

#ifdef __cplusplus
}
#endif

#endif /* __SVC_FSM_H__ */
