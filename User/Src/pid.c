#include "pid.h"
#include "stm32h5xx_hal.h"
#include "utils.h"

PID pid_create(PIDConfig *config)
{
    return (PID){
        .config = *config,
        .integral = 0,
        .lastPosition = 0,
        .smoothedVelocity = 0,
    };
}

float pid_compute_dt(uint32_t *lastTime)
{
    uint32_t currentTime = HAL_GetTick();
    float dt = (currentTime - *lastTime) / MS_PER_SECOND;
    *lastTime = currentTime;

    if (dt > PID_MAX_DT_S)
        dt = PID_FALLBACK_DT_S;

    return dt;
}

float pid_update(PID *pid, float position, float dt)
{
    float velocity = (position - pid->lastPosition) / dt;
    pid->lastPosition = position;

    pid->smoothedVelocity = PID_VELOCITY_SMOOTHING_FACTOR * velocity +
                            (1.0f - PID_VELOCITY_SMOOTHING_FACTOR) * pid->smoothedVelocity;

    float error = position;
    pid->integral += error * dt;
    pid->integral = MAX(MIN(pid->integral, pid->config.integralLimit), -pid->config.integralLimit);

    float output = pid->config.kp * error + pid->config.ki * pid->integral + pid->config.kd * pid->smoothedVelocity;
    output = MAX(MIN(output, pid->config.outputLimit), -pid->config.outputLimit);

    return output;
}
