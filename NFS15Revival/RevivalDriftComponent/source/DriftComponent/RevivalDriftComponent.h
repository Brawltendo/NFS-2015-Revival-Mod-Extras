#pragma once
#include "pch.h"
#include <corecrt_math.h>
#include <xmmintrin.h>
#include "PerformanceModification/PerformanceModification.h"

float GetAvgRearSlip(class DriftComponent* driftComponent);
void CheckForEnteringDrift(class NFSVehicle* nfsVehicle, DriftComponent* driftComp);
void UpdateDriftAngle(class NFSVehicle* nfsVehicle);
void UpdateDriftScale(class NFSVehicle* nfsVehicle, DriftComponent* const driftComp);
bool IsChainingDrift(class DriftComponent* driftComp);
void UpdateCounterSteeringSideMagnitude(class NFSVehicle* const nfsVehicle, class DriftComponent* driftComp, const float lvfSteering, const float localAngVelDegrees, const float lvfTimeStep);
float RemapSteeringForDrift(class DriftComponent* const driftComp, const float steeringInput, const float slipAngleDegrees, const float maxSteeringAngle, class SteeringComponent* lpSteeringComponent);