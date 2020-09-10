#include "pch.h"
#include "RevivalDriftComponent.h"

float map(float s, float a1, float a2, float b1, float b2)
{
    return b1 + (s - a1) * (b2 - b1) / (a2 - a1);
}

// Creates a PointGraph from a vector array. The arrays used for this are never greater or less than 10 members.
PointGraph8 initPointGraph8(__m128 curveData[10])
{
	PointGraph8 pgOut{};

	pgOut.min_x = curveData[0].m128_f32[0];
	pgOut.min_y = curveData[0].m128_f32[1];
	pgOut.max_x = curveData[1].m128_f32[0];
	pgOut.max_y = curveData[1].m128_f32[1];
	float rangeX = curveData[1].m128_f32[0] - curveData[0].m128_f32[0];
	float rangeY = curveData[1].m128_f32[1] - curveData[0].m128_f32[1];
	for (int i = 0; i < 8; i++)
	{
		pgOut.x[i] = (rangeX * curveData[i + 2].m128_f32[0]) + pgOut.min_x;
		pgOut.y[i] = (rangeY * curveData[i + 2].m128_f32[1]) + pgOut.min_y;
	}
	return pgOut;
}
float EvaluatePointGraph8(PointGraph8* pgIn, float xVal)
{
	if (xVal < pgIn->x[0]) return pgIn->y[0];
	if (xVal >= pgIn->x[7]) return pgIn->y[7];
	float v9 = 0;
	int v7 = 1;
	int v8 = 1;
	if (8 <= 1)
		return pgIn->y[0];
	while (1)
	{
		v9 = pgIn->x[v8];
		if (xVal < v9)
			break;
		++v8;
		++v7;
		if (v8 >= 8)
			return pgIn->y[0];
	}
	int v10 = v7 - 1;
	float v11 = pgIn->x[v7 - 1];
	float v12 = v9 - v11;
	float result = pgIn->y[v7];
	if (v12 > 0.0)
		result = ((result - pgIn->y[v7 - 1]) / v12) * (xVal - v11) + pgIn->y[v7 - 1];
	return result;
}

float sign(float in, float scale)
{
    if (in > 0)
        return 1 * fabsf(scale);
    if (in < 0)
        return -1 * fabsf(scale);
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