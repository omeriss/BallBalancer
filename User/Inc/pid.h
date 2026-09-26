#pragma once

#include <stdint.h>

#define PID_VELOCITY_SMOOTHING_FACTOR 0.5f
#define PID_MAX_DT_S 0.1f
#define PID_FALLBACK_DT_S 0.01f
#define MS_PER_SECOND 1000.0f

typedef struct PIDConfig
{
    float kp;
    float ki;
    float kd;
    float integralLimit;
    float outputLimit;
} PIDConfig;

typedef struct PID
{
    PIDConfig config;
    float integral;
    float lastPosition;
    float smoothedVelocity;
} PID;

PID pid_create(PIDConfig *config);
float pid_compute_dt(uint32_t *lastTime);
float pid_update(PID *pid, float position, float dt);
