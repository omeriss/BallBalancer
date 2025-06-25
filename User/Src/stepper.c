#include "stepper.h"

uint16_t getAngleStep(const StepperMotorConfig *config, float angle)
{
    return (uint16_t)(angle / (360.0f / config->stepsPerRevolution)) * config->microstepping;
}

StepperMotor motor_create(StepperMotorConfig *config, GPIO_TypeDef *port, uint16_t stepPin, uint16_t dirPin, uint8_t startAngle, bool reverse)
{
    uint16_t initialStep = getAngleStep(config, startAngle);
    HAL_GPIO_WritePin(port, stepPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port, dirPin, GPIO_PIN_RESET);

    return (StepperMotor){
        .port = port,
        .stepPin = stepPin,
        .dirPin = dirPin,
        .config = *config,
        .speed = 0,
        .step = initialStep,
        .desiredStep = initialStep,
        .reverse = reverse,
        .stepPinState = false,
        .lastDirState = reverse,
    };
}

void motor_set_angle(StepperMotor *motor, float angle)
{
    if (angle < 0 || angle >= 360)
        return;

    motor->desiredStep = getAngleStep(&motor->config, angle);
}

void motor_run(StepperMotor *motor)
{
    // make step Gap largger the bigger the difference between desired and current step
    uint16_t stepDiff = motor->desiredStep > motor->step
                            ? motor->desiredStep - motor->step
                            : motor->step - motor->desiredStep;

    uint16_t stepTime = motor->config.maxStepsGap - (MIN(MAX_ANGLE_DIFF, stepDiff) * (motor->config.maxStepsGap - motor->config.minStepsGap) / MAX_ANGLE_DIFF);

    if (motor->desiredStep == motor->step)
        return;

    uint16_t time = __HAL_TIM_GET_COUNTER(motor->config.timerHandle);
    uint16_t timeDiff = (uint16_t)(time - motor->lastRunTime);
    timeDiff += motor->lastRunRemainder;

    if (timeDiff < stepTime)
        return;

    motor->lastRunTime = time;
    motor->lastRunRemainder = MAX(timeDiff - stepTime, MAX_REMAINDER);

    bool dir = (motor->desiredStep > motor->step);

    if (motor->lastDirState != dir)
    {
        motor->lastDirState = dir;
        HAL_GPIO_WritePin(motor->port, motor->dirPin, (dir ^ motor->reverse));

        if (!motor->stepPinState)
            return;
    }

    HAL_GPIO_WritePin(motor->port, motor->stepPin, !motor->stepPinState);
    motor->step += (dir * 2 - 1) * (!motor->stepPinState);
    motor->stepPinState = !motor->stepPinState;
}

bool motor_is_in_position(StepperMotor *motor)
{
    return motor->desiredStep == motor->step;
}
