#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32h5xx_hal.h"
#include "utils.h"

#define MAX_ANGLE_DIFF 90
#define MAX_REMAINDER 200

typedef struct StepperMotorConfig
{
  uint32_t acceleration;
  uint8_t stepsPerRevolution;
  uint8_t microstepping;
  TIM_HandleTypeDef *timerHandle;
  uint16_t maxStepsGap;
  uint16_t minStepsGap;
} StepperMotorConfig;

typedef struct StepperMotor
{
  bool reverse;
  GPIO_TypeDef *port;
  uint16_t stepPin;
  uint16_t dirPin;
  StepperMotorConfig config;
  uint16_t speed;
  uint16_t step;
  uint16_t desiredStep;
  uint16_t lastRunTime;
  uint16_t lastRunRemainder;
  bool stepPinState;
  bool lastDirState;
} StepperMotor;

StepperMotor motor_create(StepperMotorConfig *config, GPIO_TypeDef *port, uint16_t stepPin, uint16_t dirPin, uint8_t startAngle, bool reverse);
void motor_set_angle(StepperMotor *motor, float angle);
bool motor_is_in_position(StepperMotor *motor);
void motor_run(StepperMotor *motor);