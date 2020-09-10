#pragma once
#include "main.h"

class BrawlDriftComponent
{
public:
	float timeSinceLastDrift;
	__m128 yawDamping;
	__m128 angDamping;
	float driftScale;
	float driftAngle;
};

float map(float s, float a1, float a2, float b1, float b2);
PointGraph8 initPointGraph8(__m128 curveData[10]);
float EvaluatePointGraph8(PointGraph8* pgIn, float xVal);
float sign(float in, float scale);
float Radians2Degrees(float radian);
__m128 Radians2DegreesVector(__m128* radian);
float GetDeltaTime(NFSVehicle* nfsVehicle);
float GetAvgRearSlip(DriftComponent* driftComponent);
float GetSpeedMph(NFSVehicle* nfsVehicle);
bool CheckForEnteringDrift(NFSVehicle* nfsVehicle, DriftComponent* driftComp);
void GetDriftScale(BrawlDriftComponent* driftComp);