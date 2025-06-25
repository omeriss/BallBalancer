#include "inverse_kinematics.h"

RRS3Options rrs3_options_create(float buttomLeg, float topLeg, float baseR, float platformR)
{
    return (RRS3Options){
        .buttomLeg = buttomLeg,
        .topLeg = topLeg,
        .baseR = baseR,
        .platformR = platformR};
}

float rrs3_calculate_angles(RRS3Leg leg, RRS3Options *options, float h, float nx, float ny)
{
    float vx, vy, vz, vd, angle;
    float nd = sqrtf(nx * nx + ny * ny + 1);
    nx /= nd;
    ny /= nd;
    float nz = 1 / nd;

    switch (leg)
    {
    case A:
        vy = options->baseR + (options->platformR / 2) * (1 - (powf(nx, 2) + 3 * powf(nz, 2) + 3 * nz) / (nz + 1 - powf(nx, 2) + (powf(nx, 4) - 3 * powf(nx, 2) * powf(ny, 2)) / ((nz + 1) * (nz + 1 - powf(nx, 2)))));
        vz = h + options->platformR * ny;
        vd = sqrtf(powf(vy, 2) + powf(vz, 2));
        angle = acosf(vy / vd) + acosf((pow(vd, 2) + powf(options->buttomLeg, 2) - powf(options->topLeg, 2)) / (2 * vd * options->buttomLeg));
        break;
    case B:
        vx = (sqrtf(3) / 2) * (options->platformR * (1 - (powf(nx, 2) + sqrtf(3) * nx * ny) / (nz + 1)) - options->baseR);
        vy = vx / sqrtf(3);
        vz = h - (options->platformR / 2) * (sqrtf(3) * nx + ny);
        vd = sqrtf(powf(vx, 2) + powf(vy, 2) + powf(vz, 2));
        angle = acosf((sqrtf(3) * vx + vy) / (-2 * vd)) + acosf((pow(vd, 2) + powf(options->buttomLeg, 2) - powf(options->topLeg, 2)) / (2 * vd * options->buttomLeg));
        break;
    case C:
        vx = (sqrtf(3) / 2) * (options->baseR - options->platformR * (1 - (powf(nx, 2) - sqrtf(3) * nx * ny) / (nz + 1)));
        vy = -vx / sqrtf(3);
        vz = h + (options->platformR / 2) * (sqrtf(3) * nx - ny);
        vd = sqrtf(powf(vx, 2) + powf(vy, 2) + powf(vz, 2));
        angle = acosf((sqrtf(3) * vx - vy) / (2 * vd)) + acosf((powf(vd, 2) + powf(options->buttomLeg, 2) - powf(options->topLeg, 2)) / (2 * vd * options->buttomLeg));
        break;
    }

    return angle * 180 / M_PI;
}
