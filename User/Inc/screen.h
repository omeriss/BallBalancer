#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#include "utils.h"

#define FLASH_CALIBRATION_PAGE_ADDRESS ((uint32_t)0x0800FC00)
#define SAMPLE_SIZE 6
#define TIME_BETWEEN_SAMPLES 100

typedef enum
{
    AXIS_X,
    AXIS_Y
} Axis;

typedef struct ScreenData
{
    int16_t x;
    int16_t y;
    bool stable;
} ScreenData;

typedef struct ScreenAxis
{
    // axis pinout
    GPIO_TypeDef *port1;
    uint32_t pin1;
    GPIO_TypeDef *port2;
    uint32_t pin2;

    // adc channel
    uint32_t adcChannel;
} ScreenAxis;

typedef struct ScreenCalibrationData
{
    uint16_t xMin;
    uint16_t xMax;
    uint16_t yMin;
    uint16_t yMax;
} ScreenCalibrationData;

typedef struct ScreenSampelingData
{
    uint16_t lastSample;
    uint16_t xSamples[SAMPLE_SIZE];
    uint16_t xIndex;
    uint16_t ySamples[SAMPLE_SIZE];
    uint16_t yIndex;
} ScreenSampelingData;

typedef struct Screen
{
    // screen dimensions
    uint16_t width;
    uint16_t height;

    // axis pinouts
    ScreenAxis xAxis;
    ScreenAxis yAxis;

    // axis data
    uint16_t xCenter;
    uint16_t yCenter;
    bool flipX;
    bool flipY;

    // calibration values
    ScreenCalibrationData calibrationData;

    // timer handle
    TIM_HandleTypeDef *timerHandle;

    // calibration data
    ScreenSampelingData sampelingData;

    ADC_HandleTypeDef *adcHandle;
} Screen;

Screen screen_create(uint16_t width, uint32_t height, GPIO_TypeDef *xPort1, uint32_t xPin1, GPIO_TypeDef *xPort2, uint32_t xPin2,
                     GPIO_TypeDef *yPort1, uint32_t yPin1, GPIO_TypeDef *yPort2, uint32_t yPin2, ADC_HandleTypeDef *adcHandle,
                     uint32_t adcChannelX, uint32_t adcChannelY);

void screen_calibration(Screen *screen);
void screen_load_calibration(Screen *screen);
ScreenData screen_read_step(Screen *screen);
ScreenData screen_read(Screen *screen);
