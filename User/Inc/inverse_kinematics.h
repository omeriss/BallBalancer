#pragma once
#include <stdint.h>
#define _USE_MATH_DEFINES
#include <math.h>

typedef struct rrs3Options
{
    float buttomLeg;
    float topLeg;
    float baseR;
    float platformR;
} RRS3Options;

typedef enum
{
    A,
    B,
    C,
} RRS3Leg;

RRS3Options rrs3_options_create(float buttomLeg, float topLeg, float baseR, float platformR);
float rrs3_calculate_angles(RRS3Leg leg, RRS3Options *options, float h, float nx, float ny);
