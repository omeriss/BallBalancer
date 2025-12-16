#include "screen.h"

void initAxis(Screen *screen, Axis axis, bool inputMode)
{
    ScreenAxis *mainAxisPtr = (axis == AXIS_X) ? &screen->xAxis : &screen->yAxis;
    ScreenAxis *otherAxisPtr = (axis == AXIS_X) ? &screen->yAxis : &screen->xAxis;

    HAL_GPIO_DeInit(mainAxisPtr->port1, mainAxisPtr->pin1);
    HAL_GPIO_DeInit(mainAxisPtr->port2, mainAxisPtr->pin2);
    HAL_GPIO_DeInit(otherAxisPtr->port1, otherAxisPtr->pin1);
    HAL_GPIO_DeInit(otherAxisPtr->port2, otherAxisPtr->pin2);

    // write to the pins
    HAL_GPIO_WritePin(mainAxisPtr->port1, mainAxisPtr->pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(mainAxisPtr->port2, mainAxisPtr->pin2, GPIO_PIN_SET);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = mainAxisPtr->pin1 | mainAxisPtr->pin2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(mainAxisPtr->port1, &GPIO_InitStruct);

    // make it read from the adc of the other axis
    GPIO_InitStruct.Pin = otherAxisPtr->pin1;
    GPIO_InitStruct.Mode = !inputMode ? GPIO_MODE_ANALOG : GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = !inputMode ? GPIO_NOPULL : GPIO_PULLDOWN;
    HAL_GPIO_Init(otherAxisPtr->port1, &GPIO_InitStruct);

    if (inputMode)
        return;

    // define sconfig
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = otherAxisPtr->adcChannel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    if (HAL_ADC_ConfigChannel(screen->adcHandle, &sConfig) != HAL_OK)
        Error_Handler();
}

void initAxisMesurement(Screen *screen, Axis axis)
{
    initAxis(screen, axis, false);
}

void initAxisInput(Screen *screen, Axis axis)
{
    initAxis(screen, axis, true);
}

Screen screen_create(uint16_t width, uint32_t height, TIM_HandleTypeDef *timerHandle, GPIO_TypeDef *xPort1, uint32_t xPin1, GPIO_TypeDef *xPort2, uint32_t xPin2, GPIO_TypeDef *yPort1, uint32_t yPin1, GPIO_TypeDef *yPort2, uint32_t yPin2, ADC_HandleTypeDef *adcHandle, uint32_t adcChannelX, uint32_t adcChannelY)
{
    Screen screen = {
        .width = width,
        .height = height,
        .xAxis = {
            .port1 = xPort1,
            .pin1 = xPin1,
            .port2 = xPort2,
            .pin2 = xPin2,
            .adcChannel = adcChannelX},
        .yAxis = {.port1 = yPort1, .pin1 = yPin1, .port2 = yPort2, .pin2 = yPin2, .adcChannel = adcChannelY},
        .adcHandle = adcHandle,
        .timerHandle = timerHandle,
    };

    return screen;
}

bool isScreenPressed(Screen *screen)
{
    initAxisInput(screen, AXIS_X);
    HAL_Delay(1);
    GPIO_PinState mesureState = HAL_GPIO_ReadPin(screen->yAxis.port1, screen->yAxis.pin1);

    return (mesureState == GPIO_PIN_RESET);
}

uint16_t readAxis(Screen *screen, Axis axis)
{
    initAxisMesurement(screen, axis);
    HAL_ADC_Start(screen->adcHandle);
    HAL_ADC_PollForConversion(screen->adcHandle, HAL_MAX_DELAY);
    uint16_t value = HAL_ADC_GetValue(screen->adcHandle);
    HAL_ADC_Stop(screen->adcHandle);

    return value;
}

void screen_calibration(Screen *screen)
{
    uint16_t xMin = readAxis(screen, AXIS_X);
    uint16_t xMax = xMin;

    uint16_t yMin = readAxis(screen, AXIS_Y);
    uint16_t yMax = yMin;

    for (int i = 0; i < 100; i++)
    {
        HAL_Delay(10);
        uint16_t xValue = readAxis(screen, AXIS_X);
        uint16_t yValue = readAxis(screen, AXIS_Y);

        if (xValue < xMin)
            xMin = xValue;
        if (xValue > xMax)
            xMax = xValue;

        if (yValue < yMin)
            yMin = yValue;
        if (yValue > yMax)
            yMax = yValue;
    }

    screen->calibrationData.xMin = xMin;
    screen->calibrationData.xMax = xMax;
    screen->calibrationData.yMin = yMin;
    screen->calibrationData.yMax = yMax;

    HAL_StatusTypeDef s = HAL_FLASH_Unlock();
    uint32_t address = FLASH_CALIBRATION_ADDRESS;

    FLASH_EraseInitTypeDef eraseInitStruct = {0};
    uint32_t delError = 0;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.Banks = FLASH_CALIBRATION_BANK;
    eraseInitStruct.Sector = FLASH_CALIBRATION_SECTOR;
    eraseInitStruct.NbSectors = 1;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInitStruct, &delError);
    if (status != HAL_OK)
        Error_Handler();

    // STM32H5 requires 128-bit (16-byte) aligned programming
    // Pack calibration data into 16-byte aligned buffer
    __attribute__((aligned(16))) uint16_t flashData[8] = {0}; // 16 bytes total, 16-byte aligned
    flashData[0] = screen->calibrationData.xMin;
    flashData[1] = screen->calibrationData.xMax;
    flashData[2] = screen->calibrationData.yMin;
    flashData[3] = screen->calibrationData.yMax;

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, (uint32_t)flashData);

    if (status != HAL_OK)
        Error_Handler();

    HAL_FLASH_Lock();
}

void screen_load_calibration(Screen *screen)
{

    screen->calibrationData.xMin = *((uint16_t *)(FLASH_CALIBRATION_ADDRESS + 0));
    screen->calibrationData.xMax = *((uint16_t *)(FLASH_CALIBRATION_ADDRESS + 2));
    screen->calibrationData.yMin = *((uint16_t *)(FLASH_CALIBRATION_ADDRESS + 4));
    screen->calibrationData.yMax = *((uint16_t *)(FLASH_CALIBRATION_ADDRESS + 6));
}

uint16_t get_sample_value_axis(Screen *screen, Axis axis)
{
    uint16_t *samples = (axis == AXIS_X) ? screen->sampelingData.xSamples : screen->sampelingData.ySamples;

    uint16_t minValue = samples[0];
    uint16_t maxValue = samples[0];

    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        if (samples[i] < minValue)
            minValue = samples[i];
        if (samples[i] > maxValue)
            maxValue = samples[i];
        sum += samples[i];
    }

    uint16_t sample = (sum - minValue - maxValue) / (SAMPLE_SIZE - 2);

    if (axis == AXIS_X)
        return (sample - screen->calibrationData.xMin) * screen->width / (screen->calibrationData.xMax - screen->calibrationData.xMin);
    else
        return (sample - screen->calibrationData.yMin) * screen->height / (screen->calibrationData.yMax - screen->calibrationData.yMin);
}

ScreenData screen_read_step(Screen *screen)
{
    return (ScreenData){};
}

ScreenData screen_read(Screen *screen)
{
    ScreenData data = {0};

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        screen->sampelingData.xSamples[i] = readAxis(screen, AXIS_X);
        delay_using_timer(screen->timerHandle, TIME_BETWEEN_SAMPLES);
    }

    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        screen->sampelingData.ySamples[i] = readAxis(screen, AXIS_Y);
        delay_using_timer(screen->timerHandle, TIME_BETWEEN_SAMPLES);
    }

    data.x = get_sample_value_axis(screen, AXIS_X);
    data.y = get_sample_value_axis(screen, AXIS_Y);
    data.stable = true;

    return data;
}
