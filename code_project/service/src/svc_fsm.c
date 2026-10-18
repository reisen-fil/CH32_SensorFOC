/**
 * @file svc_fsm.c
 * @brief 有限状态机服务实现（通用、与硬件无关）。
 * @details 框架只做三件事：维护"当前状态号 + 状态累计时间"、按表调用 enter / run / exit
 *          回调、把回调里的切换请求统一收敛成 exit → enter 一次原子切换。所有周期推进与
 *          业务判断都在调用方提供的状态回调里，本文件不含任何具体业务与硬件访问。
 *          切换时机：request() 只登记请求，真正的 exit → enter 在当拍 run 返回后（或下一拍
 *          run 开头）执行，因此回调不会在自身执行过程中被重入。
 */
#include "svc_fsm.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief 校验状态机对象是否可用。
 * @param p_object 状态机对象指针。
 * @return true 表示状态表、状态个数与当前状态号都合法。
 */
static bool svc_fsm_is_object_valid(const svc_fsm_t *p_object)
{
    if ((p_object == NULL) || (p_object->config.p_states == NULL)) {
        return false;
    }
    if (p_object->config.state_count == 0U) {
        return false;
    }
    return (p_object->current_state_index < p_object->config.state_count);
}

/**
 * @brief 进入指定状态：清零状态计时并调用其 pf_enter。
 * @param p_object 状态机对象指针。
 * @param state_index 目标状态号（调用方保证不越界）。
 * @return 状态回调的返回值；非 ok 时把对象置为 error。
 */
static svc_fsm_status_enum svc_fsm_enter_state(svc_fsm_t *p_object, uint16_t state_index)
{
    const svc_fsm_state_t *p_state = &p_object->config.p_states[state_index];
    svc_fsm_status_enum status = svc_fsm_ok;

    p_object->current_state_index = state_index;
    p_object->state_elapsed_us = 0U;

    if (p_state->pf_enter != NULL) {
        status = p_state->pf_enter(p_object, p_object->config.p_context, 0U);
        p_object->last_status = status;
    }

    if (status != svc_fsm_ok) {
        p_object->state = svc_fsm_state_error;
    }

    return status;
}

/**
 * @brief 执行一次状态切换：exit 旧状态 → 记录切换 → enter 新状态。
 * @param p_object 状态机对象指针（pending_state_index 必须为合法状态号）。
 * @return 状态回调的返回值；非 ok 时把对象置为 error。
 */
static svc_fsm_status_enum svc_fsm_apply_transition(svc_fsm_t *p_object)
{
    const svc_fsm_state_t *p_current =
        &p_object->config.p_states[p_object->current_state_index];
    svc_fsm_status_enum status = svc_fsm_ok;
    uint16_t next_state_index = p_object->pending_state_index;

    p_object->pending_state_index = SVC_FSM_STATE_NONE;

    /* 1) 退出当前状态 */
    if (p_current->pf_exit != NULL) {
        status = p_current->pf_exit(p_object, p_object->config.p_context,
                                    p_object->state_elapsed_us);
        p_object->last_status = status;
        if (status != svc_fsm_ok) {
            p_object->state = svc_fsm_state_error;
            return status;
        }
    }

    /* 2) 记录切换并进入新状态 */
    p_object->previous_state_index = p_object->current_state_index;
    p_object->transition_count++;

    return svc_fsm_enter_state(p_object, next_state_index);
}

/**
 * @brief 初始化状态机：注入配置并进入初始状态。
 * @param p_object 状态机对象指针。
 * @param p_config 状态机配置。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效或状态表/初始状态非法。
 * @retval svc_fsm_err_state 初始状态进入失败（对象已置 error）。
 */
svc_fsm_status_enum svc_fsm_init(svc_fsm_t *p_object, const svc_fsm_config_t *p_config)
{
    svc_fsm_status_enum status;

    if ((p_object == NULL) || (p_config == NULL) ||
        (p_config->p_states == NULL) || (p_config->state_count == 0U) ||
        (p_config->initial_state_index >= p_config->state_count)) {
        return svc_fsm_err_param;
    }

    (void)memset(p_object, 0, sizeof(*p_object));
    p_object->config = *p_config;

    p_object->current_state_index = p_config->initial_state_index;
    p_object->previous_state_index = p_config->initial_state_index;
    p_object->pending_state_index = SVC_FSM_STATE_NONE;
    p_object->state_elapsed_us = 0U;
    p_object->transition_count = 0U;
    p_object->last_status = svc_fsm_ok;
    p_object->state = svc_fsm_state_ready;

    /* 进入初始状态（若其 enter 失败，enter_state 会把对象置 error） */
    status = svc_fsm_enter_state(p_object, p_config->initial_state_index);

    return status;
}

/**
 * @brief 请求切换到指定状态（不会立即打断当前回调）。
 * @param p_object 状态机对象指针。
 * @param state_index 目标状态号。
 * @retval svc_fsm_ok 操作成功（含"请求与当前状态相同"的空操作）。
 * @retval svc_fsm_err_param 参数无效或状态号越界。
 * @retval svc_fsm_err_state 状态机未初始化或已出错。
 */
svc_fsm_status_enum svc_fsm_request(svc_fsm_t *p_object, uint16_t state_index)
{
    if ((p_object == NULL) || (state_index >= p_object->config.state_count) ||
        (p_object->config.p_states == NULL)) {
        return svc_fsm_err_param;
    }
    if ((p_object->state != svc_fsm_state_ready) &&
        (p_object->state != svc_fsm_state_running)) {
        return svc_fsm_err_state;
    }
    if (state_index == p_object->current_state_index) {
        return svc_fsm_ok;
    }

    p_object->pending_state_index = state_index;

    return svc_fsm_ok;
}

/**
 * @brief 按周期时长推进一次：处理遗留切换请求 → 执行当前状态 pf_run → 处理本拍请求。
 * @param p_object 状态机对象指针。
 * @param elapsed_us 本次调用距上次调用的时长（µs）。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效。
 * @retval svc_fsm_err_state 状态机未初始化或已出错。
 * @retval 其他负值：当前状态回调返回的错误码（对象已置 error）。
 */
svc_fsm_status_enum svc_fsm_run(svc_fsm_t *p_object, uint32_t elapsed_us)
{
    const svc_fsm_state_t *p_current;
    svc_fsm_status_enum status;

    if (!svc_fsm_is_object_valid(p_object)) {
        return svc_fsm_err_param;
    }
    if ((p_object->state != svc_fsm_state_ready) &&
        (p_object->state != svc_fsm_state_running)) {
        return svc_fsm_err_state;
    }

    /* 1) 先消化上一拍遗留的切换请求 */
    if (p_object->pending_state_index != SVC_FSM_STATE_NONE) {
        status = svc_fsm_apply_transition(p_object);
        if (status != svc_fsm_ok) {
            return status;
        }
    }

    p_object->state = svc_fsm_state_running;
    p_object->state_elapsed_us += elapsed_us;

    /* 2) 执行当前状态的周期动作（回调内可调用 svc_fsm_request 请求切换） */
    p_current = &p_object->config.p_states[p_object->current_state_index];
    if (p_current->pf_run != NULL) {
        status = p_current->pf_run(p_object, p_object->config.p_context, elapsed_us);
        p_object->last_status = status;
        if (status != svc_fsm_ok) {
            p_object->state = svc_fsm_state_error;
            return status;
        }
    }

    /* 3) 本拍内请求的切换立即生效 */
    if (p_object->pending_state_index != SVC_FSM_STATE_NONE) {
        status = svc_fsm_apply_transition(p_object);
        if (status != svc_fsm_ok) {
            return status;
        }
    }

    return svc_fsm_ok;
}

/**
 * @brief 复位状态机：退出当前状态并重新进入初始状态。
 * @param p_object 状态机对象指针。
 * @retval svc_fsm_ok 操作成功。
 * @retval svc_fsm_err_param 参数无效。
 * @retval svc_fsm_err_state 退出/进入回调失败（对象已置 error）。
 */
svc_fsm_status_enum svc_fsm_reset(svc_fsm_t *p_object)
{
    const svc_fsm_state_t *p_current;
    svc_fsm_status_enum status = svc_fsm_ok;

    if (!svc_fsm_is_object_valid(p_object)) {
        return svc_fsm_err_param;
    }

    /* 1) 退出当前状态 */
    p_current = &p_object->config.p_states[p_object->current_state_index];
    if (p_current->pf_exit != NULL) {
        status = p_current->pf_exit(p_object, p_object->config.p_context,
                                    p_object->state_elapsed_us);
        p_object->last_status = status;
        if (status != svc_fsm_ok) {
            p_object->state = svc_fsm_state_error;
            return status;
        }
    }

    /* 2) 清空运行痕迹并重新进入初始状态 */
    p_object->pending_state_index = SVC_FSM_STATE_NONE;
    p_object->previous_state_index = p_object->config.initial_state_index;
    p_object->transition_count = 0U;
    p_object->state = svc_fsm_state_ready;

    return svc_fsm_enter_state(p_object, p_object->config.initial_state_index);
}

/**
 * @brief 查询框架生命周期状态。
 * @param p_object 状态机对象指针。
 * @return 生命周期状态；参数为空时返回 svc_fsm_state_error。
 */
svc_fsm_state_enum svc_fsm_get_state(const svc_fsm_t *p_object)
{
    if (p_object == NULL) {
        return svc_fsm_state_error;
    }
    return p_object->state;
}

/**
 * @brief 查询当前状态号。
 * @param p_object 状态机对象指针。
 * @return 当前状态号；参数为空时返回 SVC_FSM_STATE_NONE。
 */
uint16_t svc_fsm_get_state_index(const svc_fsm_t *p_object)
{
    if (p_object == NULL) {
        return SVC_FSM_STATE_NONE;
    }
    return p_object->current_state_index;
}

/**
 * @brief 查询当前状态已运行时间。
 * @param p_object 状态机对象指针。
 * @return 累计时间（µs）；参数为空时返回 0。
 */
uint32_t svc_fsm_get_state_elapsed_us(const svc_fsm_t *p_object)
{
    if (p_object == NULL) {
        return 0U;
    }
    return p_object->state_elapsed_us;
}

/**
 * @brief 判断当前是否处于指定状态。
 * @param p_object 状态机对象指针。
 * @param state_index 待比较的状态号。
 * @return true 表示当前状态号与之一致。
 */
bool svc_fsm_is_state(const svc_fsm_t *p_object, uint16_t state_index)
{
    if (p_object == NULL) {
        return false;
    }
    return (p_object->current_state_index == state_index);
}

/** @brief 状态机服务函数表（本文件私有）。 */
static const svc_fsm_ops_t s_svc_fsm_ops = {
    .pf_init = svc_fsm_init,
    .pf_request = svc_fsm_request,
    .pf_run = svc_fsm_run,
    .pf_reset = svc_fsm_reset,
    .pf_get_state = svc_fsm_get_state,
    .pf_get_state_index = svc_fsm_get_state_index,
    .pf_get_state_elapsed_us = svc_fsm_get_state_elapsed_us,
    .pf_is_state = svc_fsm_is_state
};

/** @brief 状态机服务函数表指针（绑定到本文件的私有函数表）。 */
const svc_fsm_ops_t *const svc_fsm_ops = &s_svc_fsm_ops;
