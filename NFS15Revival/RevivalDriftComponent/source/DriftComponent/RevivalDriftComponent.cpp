#include "pch.h"
#include "util\utils.h"
#include "RevivalDriftComponent.h"

// For some reason this is the only var I've run into that can't just directly be grabbed using the recreated class. Weird.
float GetDeltaTime(NFSVehicle* nfsVehicle)
{
    float dT;
    ReadProcessMemory(GetCurrentProcess(), (BYTE*)nfsVehicle + 0xE78, &dT, sizeof(dT), nullptr);
    return dT;
}

float GetAvgRearSlip(DriftComponent* driftComponent)
{
    return driftComponent->avgSlipAngle.m128_f32[0];
}

float GetSpeedMph(NFSVehicle* nfsVehicle)
{
    return nfsVehicle->speedMps * 2.2369399;
}

bool CheckForEnteringDrift(NFSVehicle* nfsVehicle, DriftComponent* driftComp)
{
    float slipToEnterDrift = driftComp->driftParams->driftTriggerParams->Slip_angle_to_enter_drift;
    float minSpeedToEnterDrift = 30;
    float avgSlipAng = GetAvgRearSlip(driftComp);

    return fabsf(GetSpeedMph(nfsVehicle)) >= minSpeedToEnterDrift && fabsf(avgSlipAng) >= slipToEnterDrift;
}

void SetDriftScale(DriftComponent* const driftComp, RevivalDriftComponent* bDrift)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->driftParams->otherParams;
    float slipForDeepDrift = driftParams->Slip_angle_for_deep_drift;
    bDrift->driftScale = map(bDrift->driftAngle, -slipForDeepDrift, slipForDeepDrift, -1, 1);
}

bool IsChainingDrift(DriftComponent* driftComp, RevivalDriftComponent* bDrift)
{
	//if (bDrift->currentDriftDirection * bDrift->lastDriftDirection < 0) // && (bDrift->timeChainingDrift > 0 && bDrift->timeChainingDrift <= 1.5))
	if (bDrift->timeSinceLastDrift <= bDrift->timeApplyingChainDriftDamping && bDrift->chainedDriftDampingFactor != 0)
		return true;
	else return false;
}

float RemapSteeringForDrift(DriftComponent* const driftComp, RevivalDriftComponent* bDrift, const float steeringInput, const float slipAngleDegrees, const float maxSteeringAngle, SteeringComponent* lpSteeringComponent)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->driftParams->otherParams;
    float defaultSteering = driftParams->Default_steering;
    float slipAngleToEnterDrift = driftParams->Slip_angle_to_enter_drift;
    float slipForDeepDrift = driftParams->Slip_angle_for_deep_drift;

    float deepDriftSlipAngleRange = slipForDeepDrift - slipAngleToEnterDrift;
    if (deepDriftSlipAngleRange <= 0.00000011920929)
        deepDriftSlipAngleRange = 0.00000011920929;
    float peakSlipAngleRange = bDrift->peakSlipAngleReached - slipAngleToEnterDrift;
    if (peakSlipAngleRange <= 0.00000011920929)
        peakSlipAngleRange = 0.00000011920929;
    float peakSADeepDriftRatio = peakSlipAngleRange / deepDriftSlipAngleRange;
    float absAvgSlipAngle = fabsf(slipAngleDegrees);

    float counterSteerScale =
        map(absAvgSlipAngle, driftParams->Low_slip_angle_for_counter_steering, driftParams->High_slip_angle_for_counter_steering,
            driftParams->Max_countersteering_at_low_slip_angle, driftParams->Max_countersteering_at_high_slip_angle);

    float DeepSARatioClamped = fminf(1, fmaxf(0, peakSADeepDriftRatio));
    float lowSACounterSteerVsPeakSARatio = driftParams->Max_countersteering_at_low_slip_angle - -1.f * DeepSARatioClamped + -1.f;
    float counterSteerSlipAngleRange = driftParams->High_slip_angle_for_counter_steering - driftParams->Low_slip_angle_for_counter_steering;
    bool v35 = counterSteerSlipAngleRange <= 0.00000011920929;
    float v36 = driftParams->Max_countersteering_at_high_slip_angle - -1.f * DeepSARatioClamped + -1.f - lowSACounterSteerVsPeakSARatio;
    float v37 = fminf(1, fmaxf(0, !v35 ? absAvgSlipAngle - driftParams->Low_slip_angle_for_counter_steering / counterSteerSlipAngleRange : 0));
    v37 = powf(v37, driftParams->Max_counter_steering_power);
    float counterSteeringAmount = v36 * fminf(1, fmaxf(0, v37)) + lowSACounterSteerVsPeakSARatio;
    __m128 v39 = _mm_or_ps(
        _mm_and_ps(_mm_cmplt_ps({ bDrift->driftScale }, { 0 }), { -1 }),
        _mm_and_ps(_mm_cmplt_ps({ 0 }, { bDrift->driftScale }), { 1 }));
    float steeringInputAtDriftScale = bDrift->driftScale * steeringInput;
    bool v41 = 0 <= steeringInputAtDriftScale;
    float steeringScale = lpSteeringComponent->steeringScale;
    lpSteeringComponent->steeringScale = driftComp->counterSteeringInDrift ? steeringScale * fabsf(counterSteerScale) : steeringScale; //fminf(1.0, fmaxf(counterSteeringAmount, steeringScale * v39.m128_f32[0])) * v39.m128_f32[0];
    lpSteeringComponent->wheelSteeringAngleRadians = maxSteeringAngle * steeringScale * 0.017453292;
    //fb::SteeringComponent::SetWheelSteeringAngleRadians(lpSteeringComponent, &steeringAngleRadians);

    return steeringInputAtDriftScale * (1 - defaultSteering) + defaultSteering * (driftComp->counterSteeringInDrift ? (counterSteeringAmount - defaultSteering) : 1);
}

void UpdateCounterSteeringSideMagnitude(NFSVehicle* const nfsVehicle, DriftComponent* driftComp, RevivalDriftComponent* bDrift, const float lvfSteering, const float localAngVelDegrees, const float lvfTimeStep)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->driftParams->otherParams;
    float sideForceMultiplier = driftParams->Side_force_multiplier;
    float absDriftScale = fabsf(bDrift->driftScale);
    float counterSteerSideMagnitude = bDrift->counterSteeringSideMagnitude;
    float localAngVelScale = fmaxf(0, fmaxf(-1 * localAngVelDegrees * 0.017453292, localAngVelDegrees * 0.017453292) * 0.5f);
    float angVelScaleClamped = fminf(1, localAngVelScale);
    driftComp->counterSteeringInDrift = bDrift->driftAngle * nfsVehicle->vehicleInput->steeringInput.X < 0;

    counterSteerSideMagnitude = driftComp->counterSteeringInDrift ? fmaxf(counterSteerSideMagnitude, angVelScaleClamped * localAngVelScale * sideForceMultiplier) : 0;
    bDrift->counterSteeringSideMagnitude = fmaxf(counterSteerSideMagnitude - (2 * lvfTimeStep), 0);
}

/*void DriftChaining(DriftComponent* driftComp, RevivalDriftComponent* bDrift)
{

}*/