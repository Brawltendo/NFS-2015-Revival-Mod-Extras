#include "pch.h"
#include "RevivalDriftComponent.h"

float map(float s, float a1, float a2, float b1, float b2)
{
    return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}

float sign(float in, float scale)
{
    if (in > 0)
        return 1 * abs(scale);
    if (in < 0)
        return -1 * abs(scale);
    else return 0;

}

float Radians2Degrees(float radian) {
    float pi = 3.14159;
    return(radian * (180 / pi));
}

__m128 Radians2DegreesVector(__m128* radian) {
    float pi = 3.14159;
    return _mm_mul_ps(*radian, _mm_shuffle_ps({ (180 / pi) }, { (180 / pi) }, 0));
}

float GetDeltaTime(NFSVehicle* nfsVehicle)
{
    float dT;
    ReadProcessMemory(GetCurrentProcess(), (BYTE*)nfsVehicle + 0xE78, &dT, sizeof(dT), nullptr);
    /*deltaTime = nfsVehicle + 0xE78*/
    return dT;
}

float GetAvgRearSlip(DriftComponent* driftComponent)
{
    //float slipAng;
    //ReadProcessMemory(GetCurrentProcess(), (BYTE*)(&driftComponent->avgSlipAngle.m128_f32[0]), &slipAng, sizeof(slipAng), nullptr);
    return driftComponent->avgSlipAngle.m128_f32[0];
}

float GetSpeedMph(NFSVehicle* nfsVehicle)
{
    //float speed;

    //ReadProcessMemory(GetCurrentProcess(), (BYTE*)(&nfsVehicle->speedMps), &speed, sizeof(speed), nullptr);
    return nfsVehicle->speedMps * 2.2369399;
}

bool CheckForEnteringDrift(NFSVehicle* nfsVehicle, DriftComponent* driftComp)
{
    float slipToEnterDrift = driftComp->driftParams->driftTriggerParams->Slip_angle_to_enter_drift;
    float minSpeedToEnterDrift = 30;

    //ReadProcessMemory(GetCurrentProcess(), (BYTE*)(&driftComp->driftParams->driftTriggerParams->Slip_angle_to_enter_drift), &slipToEnterDrift, sizeof(slipToEnterDrift), nullptr);
    float avgSlipAng = GetAvgRearSlip(driftComp);

    return fabsf(GetSpeedMph(nfsVehicle)) >= minSpeedToEnterDrift && fabsf(avgSlipAng) >= slipToEnterDrift;
}

void GetDriftScale(BrawlDriftComponent* driftComp)
{
    driftComp->driftScale = map(driftComp->driftAngle, -80, 80, -1, 1);
}