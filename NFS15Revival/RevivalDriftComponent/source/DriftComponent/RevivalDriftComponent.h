#pragma once
#include <corecrt_math.h>
#include "PerformanceModification/PerformanceModification.h"

using namespace std;

class RevivalDriftComponent
{
public:
	float timeSinceLastDrift;
	__m128 yawDamping;
	__m128 angDamping;
	float driftScale;
	float driftAngle;
	float peakSlipAngleReached;
	float timeSteeringLeft;
	float timeSteeringRight;
	float timeApplyingChainDriftDamping;
	float chainedDriftDampingFactor;
	float counterSteeringSideMagnitude;
};

float GetDeltaTime(NFSVehicle* nfsVehicle);
float GetAvgRearSlip(DriftComponent* driftComponent);
float GetSpeedMph(NFSVehicle* nfsVehicle);
bool CheckForEnteringDrift(NFSVehicle* nfsVehicle, DriftComponent* driftComp);
void SetDriftScale(DriftComponent* const driftComp, RevivalDriftComponent* bDrift);
bool IsChainingDrift(DriftComponent* driftComp, RevivalDriftComponent* bDrift);
void UpdateCounterSteeringSideMagnitude(NFSVehicle* const nfsVehicle, DriftComponent* driftComp, RevivalDriftComponent* bDrift, const float lvfSteering, const float localAngVelDegrees, const float lvfTimeStep);
float RemapSteeringForDrift(DriftComponent* const driftComp, RevivalDriftComponent* bDrift, const float steeringInput, const float slipAngleDegrees, const float maxSteeringAngle, SteeringComponent* lpSteeringComponent);