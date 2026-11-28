#include "pid_controller.h"

#include <float.h>
#include <stddef.h>

static uint8_t PID_IsFinite(float value)
{
    return (value == value) &&
           (value <= FLT_MAX) &&
           (value >= -FLT_MAX);
}

static float PID_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

PID_Status PID_ConfigValidate(const PID_Config *config)
{
    if ((config == NULL) ||
        !PID_IsFinite(config->kp) ||
        !PID_IsFinite(config->ki) ||
        !PID_IsFinite(config->kd) ||
        !PID_IsFinite(config->output_min) ||
        !PID_IsFinite(config->output_max) ||
        !PID_IsFinite(config->integral_min) ||
        !PID_IsFinite(config->integral_max) ||
        !PID_IsFinite(config->derivative_filter_alpha) ||
        (config->output_min >= config->output_max) ||
        (config->integral_min > config->integral_max) ||
        (config->derivative_filter_alpha < 0.0f) ||
        (config->derivative_filter_alpha > 1.0f))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    return PID_STATUS_OK;
}

PID_Status PID_HistoryReset(PID_History *history)
{
    if (history == NULL)
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    history->integral = 0.0f;
    history->previous_error = 0.0f;
    history->filtered_derivative = 0.0f;
    history->has_previous_error = 0U;
    return PID_STATUS_OK;
}

static PID_Status PID_CalculateInternal(
    const PID_Config *config,
    float current_value,
    float target_value,
    float error_derivative,
    float dt_seconds,
    uint8_t derivative_is_external,
    PID_History *history,
    float *output)
{
    float error;
    float derivative;
    float filtered_derivative;
    float candidate_integral;
    float integral_delta_output;
    float proportional_output;
    float integral_output;
    float derivative_output;
    float raw_output;

    if ((history == NULL) ||
        (output == NULL) ||
        (PID_ConfigValidate(config) != PID_STATUS_OK) ||
        !PID_IsFinite(current_value) ||
        !PID_IsFinite(target_value) ||
        !PID_IsFinite(dt_seconds) ||
        (dt_seconds <= 0.0f) ||
        ((derivative_is_external != 0U) &&
         !PID_IsFinite(error_derivative)) ||
        !PID_IsFinite(history->integral) ||
        !PID_IsFinite(history->previous_error) ||
        !PID_IsFinite(history->filtered_derivative) ||
        (history->has_previous_error > 1U))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    error = target_value - current_value;
    if (!PID_IsFinite(error))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    if (derivative_is_external != 0U)
    {
        derivative = error_derivative;
    }
    else if (history->has_previous_error != 0U)
    {
        derivative = (error - history->previous_error) / dt_seconds;
    }
    else
    {
        derivative = 0.0f;
    }
    if (!PID_IsFinite(derivative))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    filtered_derivative =
        (config->derivative_filter_alpha * derivative) +
        ((1.0f - config->derivative_filter_alpha) *
         history->filtered_derivative);
    candidate_integral = history->integral + (error * dt_seconds);
    if (!PID_IsFinite(filtered_derivative) ||
        !PID_IsFinite(candidate_integral))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }
    candidate_integral = PID_Clamp(
        candidate_integral,
        config->integral_min,
        config->integral_max);

    proportional_output = config->kp * error;
    integral_output = config->ki * candidate_integral;
    derivative_output = config->kd * filtered_derivative;
    integral_delta_output =
        config->ki * (candidate_integral - history->integral);
    raw_output = proportional_output + integral_output + derivative_output;
    if (!PID_IsFinite(raw_output) || !PID_IsFinite(integral_delta_output))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    if (((raw_output > config->output_max) &&
         (integral_delta_output > 0.0f)) ||
        ((raw_output < config->output_min) &&
         (integral_delta_output < 0.0f)))
    {
        candidate_integral = history->integral;
        integral_output = config->ki * candidate_integral;
        raw_output = proportional_output + integral_output + derivative_output;
        if (!PID_IsFinite(raw_output))
        {
            return PID_STATUS_INVALID_ARGUMENT;
        }
    }

    history->integral = candidate_integral;
    history->previous_error = error;
    history->filtered_derivative = filtered_derivative;
    history->has_previous_error = 1U;
    *output = PID_Clamp(raw_output, config->output_min, config->output_max);

    return PID_STATUS_OK;
}

PID_Status PID_Calculate(
    const PID_Config *config,
    float current_value,
    float target_value,
    float dt_seconds,
    PID_History *history,
    float *output)
{
    return PID_CalculateInternal(
        config,
        current_value,
        target_value,
        0.0f,
        dt_seconds,
        0U,
        history,
        output);
}

PID_Status PID_CalculateWithDerivative(
    const PID_Config *config,
    float current_value,
    float target_value,
    float error_derivative,
    float dt_seconds,
    PID_History *history,
    float *output)
{
    return PID_CalculateInternal(
        config,
        current_value,
        target_value,
        error_derivative,
        dt_seconds,
        1U,
        history,
        output);
}
