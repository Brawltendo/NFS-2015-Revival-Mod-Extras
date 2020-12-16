#include "pch.h"
#include "RevivalDriftComponent.h"
#include <algorithm>
#include "util\utils.h"
#include "NFSClasses.h"

float RevivalDriftComponent::GetAvgRearSlip(DriftComponent* driftComponent)
{
    return driftComponent->mvfRearSlipAngle.m128_f32[0];
}

void RevivalDriftComponent::CheckForEnteringDrift(NFSVehicle* nfsVehicle, DriftComponent* driftComp)
{
    
    float minSpeedToEnterDrift = 30;
    float avgSlipAng = driftComp->mvfRearSlipAngle.m128_f32[0];
    float timePressingBrake = driftComp->mvfTimePressingBrake.m128_f32[0];
    float slipAngleToEnter;
    bool isBrakeStab = timePressingBrake <= 0.7f && nfsVehicle->m_input->brakeInput.X >= 0.25f && nfsVehicle->m_input->steeringInput.X != 0;
    float minAngleForDrift = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_MinimumAngleForDrift, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift);
    float brakeSlipAngForDrift = isBrakeStab ? 
        GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_SlipAngleToEnterWhenBrakeStab, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift_after_brake_stab) : 
        GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_SlipAngleToEnterWhenBraking, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift_when_braking);

    float handbrakeSlipAngForDrift = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_SlipAngleToEnterWhenHandbraking, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift_when_handbraking);
    
    if (nfsVehicle->m_input->brakeInput.X >= 0.25f)
    {
        slipAngleToEnter = brakeSlipAngForDrift;

        if (nfsVehicle->m_input->handbrakeInput)
        {
            slipAngleToEnter = brakeSlipAngForDrift + handbrakeSlipAngForDrift / 2.f;
        }
    }
    else if (nfsVehicle->m_input->handbrakeInput) slipAngleToEnter = handbrakeSlipAngForDrift;
    else slipAngleToEnter = minAngleForDrift;

    driftComp->mvfSlipAngleForDriftEntry = _mm_set1_ps(slipAngleToEnter);

    if (GetSpeedMph(nfsVehicle) >= minSpeedToEnterDrift && fabsf(avgSlipAng) >= slipAngleToEnter && !this->isDrifting)
    {
        this->isDrifting = true;
    }
    else return;
}

void RevivalDriftComponent::UpdateDriftAngle(NFSVehicle* nfsVehicle)
{
    this->driftAngle = RadiansToDegrees(nfsVehicle->m_carSlipAngle);
    nfsVehicle->m_driftComponent->mvfCurrentDriftAngle = _mm_shuffle_ps({ this->driftAngle }, { this->driftAngle }, 0);
    this->maxDriftAngle = fmaxf(this->maxDriftAngle, fabsf(this->driftAngle));
}

void RevivalDriftComponent::UpdateDriftScale(NFSVehicle* nfsVehicle, DriftComponent* const driftComp)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->otherParams;
    //float slipForDeepDrift = driftParams->Slip_angle_for_deep_drift;
    float driftScale;
    //float steeringDirection = sign(nfsVehicle->m_lastInputSteering, 1);
    float absSteeringInput = fabsf(nfsVehicle->m_lastInputSteering);
    float driftScaleDecay = driftParams->Drift_scale_decay * -sign(this->driftScale, 1);
    float scaleFromSteering = !driftComp->counterSteeringInDrift ? 
        GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromSteering, driftParams->Drift_scale_from_steering) * nfsVehicle->m_lastInputSteering :
        (GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromCounterSteering, driftParams->Drift_scale_from_counter_steering) * absSteeringInput * -sign(this->driftScale, 1));
    float scaleFromBraking = nfsVehicle->m_input->brakeInput.X * GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromBraking, driftParams->Drift_scale_from_braking);
    float scaleFromEbrakeLowSpeed = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromHandbrakeAtLowSpeed, driftParams->DriftScaleFromHandbrakeAtLowSpeed);
    float scaleFromEbrakeHighSpeed = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromHandbrake, driftParams->DriftScaleFromHandbrakeAtHighSpeed);
    float scaleFromHandbraking = map(GetSpeedMph(nfsVehicle), 40.f, 95.f, scaleFromEbrakeLowSpeed, scaleFromEbrakeHighSpeed) * nfsVehicle->m_input->handbrakeInput;
    float scaleFromGasStab = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromGasStab, driftParams->Drift_scale_from_gas_stab) * driftComp->mvfPreviousGasInput.m128_f32[0];
    float scaleFromNoGas = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftScaleFromGasLetOff, driftParams->Drift_scale_from_gas_let_off);
    //this->mvfDriftScale = map(this->driftAngle, -slipForDeepDrift, slipForDeepDrift, -1, 1);
    float combinedScale = scaleFromSteering + (scaleFromGasStab + scaleFromBraking + scaleFromHandbraking * sign(this->driftScale, 1));
    float combinedDecay = driftScaleDecay + scaleFromNoGas * sign(this->driftScale, 1);

    if (absSteeringInput != 0 && driftComp->mvfPreviousGasInput.m128_f32[0] > 0.1f)
    {
        this->driftScale += combinedScale * nfsVehicle->m_currentUpdateDt;
        this->driftScale = std::clamp(this->driftScale, -1.f, 1.f);
    }
    else
    {
        if (this->driftScale > 0 || this->driftScale < 0)
        {
            this->driftScale += combinedDecay * nfsVehicle->m_currentUpdateDt;
            this->driftScale = std::clamp(this->driftScale, -1.f, 1.f);
        }
        else this->driftScale = 0;
    }

    //this->driftScale = driftScale;
    driftComp->mvfDriftScale = _mm_shuffle_ps({ this->driftScale }, { this->driftScale }, 0);
}

bool RevivalDriftComponent::IsChainingDrift(DriftComponent* driftComp)
{
	//if (bDrift->currentDriftDirection * bDrift->lastDriftDirection < 0) // && (bDrift->timeChainingDrift > 0 && bDrift->timeChainingDrift <= 1.5))
	if (this->timeSinceDriftExit <= this->exitDriftTimer && this->driftExitDampingFactor != 0)
		return true;
	else return false;
}

float RevivalDriftComponent::RemapSteeringForDrift(DriftComponent* const driftComp, const float steeringInput, const float slipAngleDegrees, const float maxSteeringAngle, SteeringComponent* lpSteeringComponent)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->otherParams;
    float defaultSteering = driftParams->Default_steering;
    float slipAngleToEnterDrift = driftParams->Slip_angle_to_enter_drift;
    float slipForDeepDrift = driftParams->Slip_angle_for_deep_drift;

    float deepDriftSlipAngleRange = slipForDeepDrift - slipAngleToEnterDrift;
    if (deepDriftSlipAngleRange <= Epsilon)
        deepDriftSlipAngleRange = Epsilon;
    float peakSlipAngleRange = this->peakSlipAngleReached - slipAngleToEnterDrift;
    if (peakSlipAngleRange <= Epsilon)
        peakSlipAngleRange = Epsilon;
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
        _mm_and_ps(_mm_cmplt_ps({ this->driftScale }, { 0 }), { -1 }),
        _mm_and_ps(_mm_cmplt_ps({ 0 }, { this->driftScale }), { 1 }));
    float steeringInputAtDriftScale = this->driftScale * steeringInput;
    float steeringScale = lpSteeringComponent->steeringScale;
    lpSteeringComponent->steeringScale = driftComp->counterSteeringInDrift ? steeringScale * fabsf(counterSteerScale) : steeringScale; //fminf(1.0, fmaxf(counterSteeringAmount, steeringScale * v39.m128_f32[0])) * v39.m128_f32[0];
    lpSteeringComponent->wheelSteeringAngleRadians = maxSteeringAngle * steeringScale * 0.017453292f;
    //fb::SteeringComponent::SetWheelSteeringAngleRadians(lpSteeringComponent, &steeringAngleRadians);

    return steeringInputAtDriftScale * (1 - defaultSteering) + defaultSteering * (driftComp->counterSteeringInDrift ? (counterSteeringAmount - defaultSteering) : 1);
}

void RevivalDriftComponent::UpdateCounterSteeringSideMagnitude(NFSVehicle* const nfsVehicle, DriftComponent* driftComp, const float lvfSteering, const float localAngVelDegrees, const float lvfTimeStep)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->otherParams;
    float sideForceMultiplier = driftParams->Side_force_multiplier;
    float absDriftScale = fabsf(this->driftScale);
    driftComp->counterSteeringInDrift = this->driftAngle * nfsVehicle->m_input->steeringInput.X > 0;
    float counterSteerSideMagnitude = this->counterSteeringSideMagnitude;
    float localAngVelScale = fmaxf(0, fabsf(localAngVelDegrees * 0.017453292f) * 0.5f);
    //float angVelScaleClamped = fminf(1, localAngVelScale);
    float angVelScaleClamped = std::clamp(fabsf(localAngVelDegrees * 0.017453292f) * 0.5f, 0.f, 1.f);

    counterSteerSideMagnitude = driftComp->counterSteeringInDrift ? fmaxf(counterSteerSideMagnitude, angVelScaleClamped * absDriftScale * sideForceMultiplier) : 0;
    counterSteerSideMagnitude -= 2.f * lvfTimeStep;
    //this->counterSteeringSideMagnitude = fmaxf(counterSteerSideMagnitude - (2.f * lvfTimeStep), 0);
    this->counterSteeringSideMagnitude = fmaxf(counterSteerSideMagnitude, 0);
    driftComp->mvfCounterSteerSideMag = _mm_shuffle_ps({ this->counterSteeringSideMagnitude }, { this->counterSteeringSideMagnitude }, 0);
}

/*void DriftChaining(DriftComponent* driftComp, RevivalDriftComponent* bDrift)
{

}*/