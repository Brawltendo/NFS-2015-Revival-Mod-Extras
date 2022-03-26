#include "pch.h"
#include "RevivalDriftComponent.h"
#include <algorithm>
#include "NFSClasses.h"

float GetAvgRearSlip(DriftComponent* driftComponent)
{
    return driftComponent->mvfRearSlipAngle.m128_f32[0];
}

void CheckForEnteringDrift(NFSVehicle* nfsVehicle, DriftComponent* driftComp)
{
    
    //float minSpeedToEnterDrift = 30;
    float avgSlipAng = driftComp->mvfRearSlipAngle.m128_f32[0];
    float timePressingBrake = driftComp->mvfTimePressingBrake.m128_f32[0];
    float slipAngleToEnter;
    bool isBrakeStab = timePressingBrake <= 0.7f && nfsVehicle->m_input->brakeInput.X >= 0.25f && nfsVehicle->m_input->steeringInput.X != 0;
    float minAngleForDrift = nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift);
    float brakeSlipAngForDrift = isBrakeStab ? 
        nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenBrakeStab, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift_after_brake_stab) : 
        nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenBraking, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift_when_braking);

    float handbrakeSlipAngForDrift = nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_SlipAngleToEnterWhenHandbraking, driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift_when_handbraking);
    
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

    if (GetSpeedMph(nfsVehicle) >= driftComp->mpParams->driftTriggerParams->Minimum_speed_to_enter_drift && fabsf(avgSlipAng) >= slipAngleToEnter && !driftComp->isDrifting)
    {
        driftComp->isDrifting = true;
    }
    else return;
}

void UpdateDriftAngle(NFSVehicle* nfsVehicle)
{
    nfsVehicle->m_driftComponent->mvfCurrentDriftAngle.m128_f32[0] = RadiansToDegrees(nfsVehicle->m_carSlipAngle);
    nfsVehicle->m_driftComponent->mvfCurrentDriftAngle.m128_f32[1] = fmaxf(nfsVehicle->m_driftComponent->mvfCurrentDriftAngle.m128_f32[1], fabsf(nfsVehicle->m_driftComponent->mvfCurrentDriftAngle.m128_f32[0]));
    //nfsVehicle->m_driftComponent->mvfCurrentDriftAngle = _mm_shuffle_ps({ driftComp->mvfCurrentDriftAngle.m128_f32[0] }, { driftComp->mvfCurrentDriftAngle.m128_f32[0] }, 0);
}

void UpdateDriftScale(NFSVehicle* nfsVehicle, DriftComponent* const driftComp)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->otherParams;
    //float slipForDeepDrift = driftParams->Slip_angle_for_deep_drift;
    //float steeringDirection = sign(nfsVehicle->m_lastInputSteering, 1);
    float absSteeringInput = fabsf(nfsVehicle->m_lastInputSteering);
    float driftScaleDecay = driftParams->Drift_scale_decay * -sign(driftComp->mvfDriftScale.m128_f32[0], 1);
    float scaleFromSteering = !driftComp->counterSteeringInDrift ? 
        nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromSteering, driftParams->Drift_scale_from_steering) * nfsVehicle->m_lastInputSteering :
        (nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromCounterSteering, driftParams->Drift_scale_from_counter_steering) * absSteeringInput * -sign(driftComp->mvfDriftScale.m128_f32[0], 1));
    float scaleFromBraking = nfsVehicle->m_input->brakeInput.X * nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromBraking, driftParams->Drift_scale_from_braking);
    float scaleFromEbrakeLowSpeed = nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromHandbrakeAtLowSpeed, driftParams->DriftScaleFromHandbrakeAtLowSpeed);
    float scaleFromEbrakeHighSpeed = nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromHandbrake, driftParams->DriftScaleFromHandbrakeAtHighSpeed);
    float scaleFromHandbraking = map(GetSpeedMph(nfsVehicle), 40.f, 95.f, scaleFromEbrakeLowSpeed, scaleFromEbrakeHighSpeed) * nfsVehicle->m_input->handbrakeInput;
    float scaleFromGasStab = nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromGasStab, driftParams->Drift_scale_from_gas_stab) * driftComp->mvfPreviousGasInput.m128_f32[0];
    float scaleFromNoGas = nfsVehicle->m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromGasLetOff, driftParams->Drift_scale_from_gas_let_off);
    //this->mvfDriftScale = map(this->driftAngle, -slipForDeepDrift, slipForDeepDrift, -1, 1);
    float combinedScale = scaleFromSteering + (scaleFromGasStab + scaleFromBraking + scaleFromHandbraking * sign(driftComp->mvfDriftScale.m128_f32[0], 1));
    float combinedDecay = driftScaleDecay + scaleFromNoGas * sign(driftComp->mvfDriftScale.m128_f32[0], 1.f);

    if (absSteeringInput != 0 && driftComp->mvfPreviousGasInput.m128_f32[0] > 0.1f)
    {
        driftComp->mvfDriftScale.m128_f32[0] += combinedScale * nfsVehicle->m_currentUpdateDt;
        driftComp->mvfDriftScale.m128_f32[0] = std::clamp(driftComp->mvfDriftScale.m128_f32[0], -1.f, 1.f);
    }
    else
    {
        if (driftComp->mvfDriftScale.m128_f32[0] > 0 || driftComp->mvfDriftScale.m128_f32[0] < 0)
        {
            driftComp->mvfDriftScale.m128_f32[0] += combinedDecay * nfsVehicle->m_currentUpdateDt;
            driftComp->mvfDriftScale.m128_f32[0] = std::clamp(driftComp->mvfDriftScale.m128_f32[0], -1.f, 1.f);
        }
        else driftComp->mvfDriftScale.m128_f32[0] = 0;
    }

    //this->driftScale = driftScale;
    //driftComp->mvfDriftScale = _mm_shuffle_ps({ driftComp->mvfDriftScale.m128_f32[0] }, { driftComp->mvfDriftScale.m128_f32[0] }, 0);
}

bool IsChainingDrift(DriftComponent* driftComp)
{
	//if (bDrift->currentDriftDirection * bDrift->lastDriftDirection < 0) // && (bDrift->timeChainingDrift > 0 && bDrift->timeChainingDrift <= 1.5))
	if (driftComp->mvfTimeSinceExittingDrift.m128_f32[0] <= driftComp->mvfTimeSinceExittingDrift.m128_f32[1] && driftComp->mvfDriftYawDamping.m128_f32[1] != 0)
		return true;
	else return false;
}

float RemapSteeringForDrift(DriftComponent* const driftComp, const float steeringInput, const float slipAngleDegrees, const float maxSteeringAngle, SteeringComponent* lpSteeringComponent)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->otherParams;
    float defaultSteering = driftParams->Default_steering;
    float slipAngleToEnterDrift = driftParams->Slip_angle_to_enter_drift;
    float slipForDeepDrift = driftParams->Slip_angle_for_deep_drift;

    float deepDriftSlipAngleRange = slipForDeepDrift - slipAngleToEnterDrift;
    if (deepDriftSlipAngleRange <= Epsilon)
        deepDriftSlipAngleRange = Epsilon;
    float peakSlipAngleRange = driftComp->mvfRearSlipAngle.m128_f32[0] - slipAngleToEnterDrift;
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
        _mm_and_ps(_mm_cmplt_ps({ driftComp->mvfDriftScale.m128_f32[0] }, { 0 }), { -1 }),
        _mm_and_ps(_mm_cmplt_ps({ 0 }, { driftComp->mvfDriftScale.m128_f32[0] }), { 1 }));
    float steeringInputAtDriftScale = driftComp->mvfDriftScale.m128_f32[0] * steeringInput;
    float steeringScale = lpSteeringComponent->steeringScale;
    lpSteeringComponent->steeringScale = driftComp->counterSteeringInDrift ? steeringScale * fabsf(counterSteerScale) : steeringScale; //fminf(1.0, fmaxf(counterSteeringAmount, steeringScale * v39.m128_f32[0])) * v39.m128_f32[0];
    lpSteeringComponent->wheelSteeringAngleRadians = maxSteeringAngle * steeringScale * 0.017453292f;
    //fb::SteeringComponent::SetWheelSteeringAngleRadians(lpSteeringComponent, &steeringAngleRadians);

    return steeringInputAtDriftScale * (1 - defaultSteering) + defaultSteering * (driftComp->counterSteeringInDrift ? (counterSteeringAmount - defaultSteering) : 1);
}

void UpdateCounterSteeringSideMagnitude(NFSVehicle* const nfsVehicle, DriftComponent* driftComp, const float lvfSteering, const float localAngVelDegrees, const float lvfTimeStep)
{
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->otherParams;
    float sideForceMultiplier = driftParams->Side_force_multiplier;
    float absDriftScale = fabsf(driftComp->mvfDriftScale.m128_f32[0]);
    driftComp->counterSteeringInDrift = driftComp->mvfDriftScale.m128_f32[0] * nfsVehicle->m_input->steeringInput.X > 0;
    float counterSteerSideMagnitude = driftComp->mvfCounterSteerSideMag.m128_f32[0];
    float localAngVelScale = fmaxf(0, fabsf(localAngVelDegrees * 0.017453292f) * 0.5f);
    //float angVelScaleClamped = fminf(1, localAngVelScale);
    float angVelScaleClamped = std::clamp(fabsf(localAngVelDegrees * 0.017453292f) * 0.5f, 0.f, 1.f);

    counterSteerSideMagnitude = driftComp->counterSteeringInDrift ? fmaxf(counterSteerSideMagnitude, angVelScaleClamped * absDriftScale * sideForceMultiplier) : 0;
    counterSteerSideMagnitude -= 2.f * lvfTimeStep;
    //this->counterSteeringSideMagnitude = fmaxf(counterSteerSideMagnitude - (2.f * lvfTimeStep), 0);
    counterSteerSideMagnitude = fmaxf(counterSteerSideMagnitude, 0);
    driftComp->mvfCounterSteerSideMag = _mm_shuffle_ps({ counterSteerSideMagnitude }, { counterSteerSideMagnitude }, 0);
}

void RevivalDriftComponent::PreUpdate(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int& numWheelsOnGround)
{
    float steering = nfsVehicle.m_raceCarInputState.inputSteering;

    if (driftComp.pad_0017 == DriftState_Out && fabsf(steering) > s_GlobalDriftParams.mSteeringThreshold)
    {
        bool isBraking = s_GlobalDriftParams.mCanEnterDriftWithBrake && nfsVehicle.m_raceCarInputState.inputBrake > s_GlobalDriftParams.mBrakeThreshold;
        bool isHandbraking = s_GlobalDriftParams.mCanEnterDriftWithHandbrake && nfsVehicle.m_wasHandbrakeOnLastUpdate;
        bool isGasStab = false;
        if (s_GlobalDriftParams.mCanEnterDriftWithGasStab && nfsVehicle.m_raceCarInputState.inputGas > s_GlobalDriftParams.mThrottleThreshold)
            isGasStab = driftComp.mvfPreviousGasInput.m128_f32[0] > s_GlobalDriftParams.mMinTimeForGasStab && driftComp.mvfPreviousGasInput.m128_f32[0] < s_GlobalDriftParams.mMaxTimeForGasStab;

        // use drift config fields for compatibility with performance mod system
        const float angleToEnterDrift = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, driftComp.mpParams->driftScaleParams->Slip_angle_to_enter_drift);
        const float slipAngle = nfsVehicle.m_sideSlipAngle * 180.f * 0.31830987f;
        bool isSlipping = s_GlobalDriftParams.mCanEnterDriftWithSlipAngle && fabsf(slipAngle) >= angleToEnterDrift;

        // check if any of the initial controlled drift conditions have been met
        if (isHandbraking || isBraking || isSlipping || isGasStab)
        {
            driftComp.pad_0017 = DriftState_Entering;
            driftComp.mvfMaintainedSpeed.m128_f32[0] = sign(steering);
            driftComp.currentYawTorque = 0.f;
            Debug("Entered drift!\n");
        }
    }

    // update throttle release timer
    if (nfsVehicle.m_raceCarInputState.inputGas <= s_GlobalDriftParams.mThrottleThreshold)
        driftComp.mvfPreviousGasInput.m128_f32[0] += nfsVehicle.m_currentUpdateDt;
    else
        driftComp.mvfPreviousGasInput.m128_f32[0] = 0.f;

    for (int i = 0; i < 4; ++i)
    {
        if (nfsVehicle.m_grounddata.isgroundValid[i])
            ++numWheelsOnGround;
    }
}

void RevivalDriftComponent::UpdateAutoSteer(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int numWheelsOnGround)
{
    const float LowAngleForAutoDriftSteer  = DegreesToRadians(15.f);
    const float HighAngleForAutoDriftSteer = DegreesToRadians(40.f);
    const float MaxAutoSteerAngle = DegreesToRadians(90.f);
    const float MinSpeedForAutoDriftSteer = MphToMps(30.f);

    // car must be grounded (3+ wheels on the ground) in order to have these forces applied
    if (numWheelsOnGround >= 3 && (nfsVehicle.m_sideSlipAngle <= MaxAutoSteerAngle && nfsVehicle.m_sideSlipAngle >= -MaxAutoSteerAngle) && nfsVehicle.m_forwardSpeed >= MinSpeedForAutoDriftSteer)
    {
        Matrix44 matrix;
        RaceRigidBody_GetTransform(driftComp.mpChassisRigidBody, &matrix);
        vec4& linVel = SimdToVec4(nfsVehicle.m_linearVelocity);
        vec4& vRight = SimdToVec4(matrix.xAxis);
        vec4& vFwd   = SimdToVec4(matrix.zAxis);

        float angleRatio = (fabsf(nfsVehicle.m_sideSlipAngle) - LowAngleForAutoDriftSteer) / (HighAngleForAutoDriftSteer - LowAngleForAutoDriftSteer);
        float dpSideVel  = Dot(linVel, vRight);
        // use drift config fields for compatibility with performance mod system
        const float sideForceScale    = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_DefaultSteering, driftComp.mpParams->driftScaleParams->Default_steering);
        const float extForceMagnitude = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_SideForceMagnitude, driftComp.mpParams->driftScaleParams->Side_force_magnitude);
        const float forwardForceScale = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_CounterSteeringRemapping, driftComp.mpParams->driftScaleParams->Counter_steering_remapping);
        // overall force scales with the car's angle
        float force = fminf(fmaxf(angleRatio, 0.f), 1.f) * extForceMagnitude * nfsVehicle.m_originalMass;
        // scale force by clamped throttle
        // with zero throttle 10% of the total force will be applied so momentum can still be maintained in corners when the throttle needs to be released
        force *= fminf(fmaxf(nfsVehicle.m_raceCarInputState.inputGas, 0.1f), 1.f);

        const vec4 sideForce(force * sign(dpSideVel) * sideForceScale);
        const vec4 forwardForce(force * forwardForceScale);
        vec4 autoSteerForce = (vRight * sideForce) + (vFwd * forwardForce);
        vec4 timestep(nfsVehicle.m_currentUpdateDt);
        AddWorldCOMForceLogged(driftComp.mpChassisRigidBody, &autoSteerForce.simdValue, &timestep.simdValue);
    }
}

void RevivalDriftComponent::ResetDrift(DriftComponent& driftComp)
{
    driftComp.currentYawTorque = 0.f;
    driftComp.mvfTimeSteerLeft.m128_f32[0] = 0.f;
    driftComp.mvfTimeSteerLeft.m128_f32[1] = 0.f;
    driftComp.mvfMaintainedSpeed.m128_f32[0] = 0.f;
    driftComp.counterSteeringInDrift = false;
}

void RevivalDriftComponent::Update(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int numWheelsOnGround)
{
    if (driftComp.pad_0017 != DriftState_Out)
    {
        // start exiting drift when the throttle has been released for long enough
        if (driftComp.mvfPreviousGasInput.m128_f32[0] > s_GlobalDriftParams.mOffGasTimeForExitingDrift)
            driftComp.pad_0017 = DriftState_Exiting;

        Matrix44 matrix;
        RaceRigidBody_GetTransform(driftComp.mpChassisRigidBody, &matrix);
        vec4& linVel = SimdToVec4(nfsVehicle.m_linearVelocity);
        vec4& angVel = SimdToVec4(nfsVehicle.m_angularVelocity);
        vec4& vRight = SimdToVec4(matrix.xAxis);
        vec4& vUp    = SimdToVec4(matrix.yAxis);
        vec4& vFwd   = SimdToVec4(matrix.zAxis);
        const float speedMps = nfsVehicle.m_forwardSpeed;
        // exit drift completely if any of these conditions are met
        if (numWheelsOnGround <= 1 || speedMps <= s_GlobalDriftParams.mSpeedToExitDrift || speedMps <= MphToMps(13.5f))
        {
            driftComp.pad_0017 = DriftState_Out;
            RevivalDriftComponent::ResetDrift(driftComp);
            Debug("Exited drift!\n");
            return;
        }

        float bodySlipAngle = atan2f(Dot(vRight, linVel), Dot(vFwd, linVel));
        float saDegrees = -RadiansToDegrees(bodySlipAngle);
        float steering = nfsVehicle.m_raceCarInputState.inputSteering;
        float absSlipAngle = fabsf(saDegrees);
        const float dT = nfsVehicle.m_currentUpdateDt;
        bool isCountersteering = steering * driftComp.mvfMaintainedSpeed.m128_f32[0] < 0.f;
        // use drift config fields for compatibility with performance mod system
        const float angleToEnterDrift   = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, driftComp.mpParams->driftScaleParams->Slip_angle_to_enter_drift);
        const float externalForcesScale = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromBraking, driftComp.mpParams->driftScaleParams->Drift_scale_from_braking);
        const float externalAngVelScale = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_DriftScaleFromCounterSteering, driftComp.mpParams->driftScaleParams->Drift_scale_from_counter_steering);

        if (absSlipAngle < s_GlobalDriftParams.mAngleToExitDrift && (driftComp.pad_0017 == DriftState_Exiting || isCountersteering))
        {
            driftComp.pad_0017 = DriftState_Out;
            Debug("Exited drift!\n");
        }
        else if (absSlipAngle >= angleToEnterDrift)
        {
            driftComp.pad_0017 = DriftState_In;
            Debug("Drifting!\n");

            // reset timer when steering in the opposite direction or not steering at all
            if (driftComp.mvfTimeSteerLeft.m128_f32[1] * steering <= 0.f)
                driftComp.mvfTimeSteerLeft.m128_f32[0] = 0.f;
            driftComp.mvfTimeSteerLeft.m128_f32[1] = steering;

            vec4 fwdVel = linVel - Dot(linVel, vUp);
            float fwdVelMag = VecLength(fwdVel);
            float saRatio = (absSlipAngle - angleToEnterDrift) / (s_GlobalDriftParams.mAngleForMaxDriftScale - angleToEnterDrift);
            saRatio = fminf(fmaxf(saRatio, 0.f), 1.f); // clamp 0-1
            float speedRatio = (fwdVelMag - s_GlobalDriftParams.mSpeedForZeroSpeedMaintenance) / (s_GlobalDriftParams.mSpeedForFullSpeedMaintenance - s_GlobalDriftParams.mSpeedForZeroSpeedMaintenance);
            speedRatio = fminf(fmaxf(speedRatio, 0.f), 1.f); // clamp 0-1
            vec4 normalizedVel = fwdVel * (1.f / fwdVelMag);
            float maintainSpeedAmount = s_GlobalDriftParams.mDriftMaintainSpeedAmount * dT * externalForcesScale * saRatio * speedRatio;
            vec4 maintainSpeedForce = linVel + ((vFwd - normalizedVel) * s_GlobalDriftParams.mDriftMaintainSpeedScale + normalizedVel) * maintainSpeedAmount;
            // may or may not add this back at some point
            // don't really see a use for it with RevivalDriftComponent::UpdateAutoSteer in place though since that's already being used to maintain side and forward speed
            //SetLinearVelocity(nfsVehicle.dynamicPhysEnt, (uint16_t*)&(nfsVehicle.pad_00C8[0]), &maintainSpeedForce.simdValue);

            if (driftComp.currentYawTorque != 0.f && !driftComp.counterSteeringInDrift)
            {
                float steerTimeRatio = driftComp.mvfTimeSteerLeft.m128_f32[0] / s_GlobalDriftParams.mMaxSteeringTime;
                bool countersteeringInDrift = saDegrees * steering <= 0.f;
                float steeringScaleByTimeMin;
                float steeringScaleByTimeMax;
                if (countersteeringInDrift)
                {
                    steeringScaleByTimeMin = s_GlobalDriftParams.mCSYawScaleForZeroSteerTime;
                    steeringScaleByTimeMax = s_GlobalDriftParams.mCSYawScaleForMaxSteerTime;
                }
                else
                {
                    steeringScaleByTimeMin = s_GlobalDriftParams.mYawScaleForZeroSteerTime;
                    steeringScaleByTimeMax = s_GlobalDriftParams.mYawScaleForMaxSteerTime;
                }

                float yawScaleVsSteerTime = steerTimeRatio * (steeringScaleByTimeMax - steeringScaleByTimeMin) + steeringScaleByTimeMin;
                float steeringYawAccel = steering * (saRatio * s_GlobalDriftParams.mSlipAngleRatioScale + 1.f) * s_GlobalDriftParams.mYawAccelScale * yawScaleVsSteerTime * dT;
                float yawSpeedInDrift = driftComp.currentYawTorque + steeringYawAccel;
                if ((fwdVelMag + maintainSpeedAmount) > s_GlobalDriftParams.mSpeedForMidDriftScale && (sign(driftComp.currentYawTorque) * yawSpeedInDrift) > fabsf(driftComp.currentYawTorque))
                {
                    float extraYaw = (absSlipAngle - s_GlobalDriftParams.mAngleForMidDriftScale) / (s_GlobalDriftParams.mAngleForMaxDriftScale - s_GlobalDriftParams.mAngleForMidDriftScale);
                    if (extraYaw > 0.f)
                        yawSpeedInDrift -= extraYaw * steeringYawAccel;
                }

                const float saSign = sign(saDegrees);
                float yawAccelScale = countersteeringInDrift ? s_GlobalDriftParams.mCountersteerYawAccelScale : 1.f;
                float ninetyDegSlip = bodySlipAngle * -DegreesToRadians(90.f);
                // adds counter yaw to steer out from a 90 degree slip angle
                float autoCsYaw = ((ninetyDegSlip * ninetyDegSlip * saSign * s_GlobalDriftParams.mAutoCSYawScaleForHighSA) + (ninetyDegSlip * s_GlobalDriftParams.mAutoCSYawScaleForLowSA));
                float autoYawAccel = (saSign * s_GlobalDriftParams.mYawAutoAccelAmount /*+ autoCsYaw*/) * dT * yawAccelScale;
                if (yawSpeedInDrift * (yawSpeedInDrift - autoYawAccel) < 0.f || yawSpeedInDrift * driftComp.currentYawTorque < 0.f)
                    yawSpeedInDrift = 0.f;
                else
                    yawSpeedInDrift -= autoYawAccel;

                yawSpeedInDrift = (yawSpeedInDrift - driftComp.currentYawTorque) * externalAngVelScale + driftComp.currentYawTorque;
                // clamp to mMaxYawSpeedInDrift
                yawSpeedInDrift = fminf(fmaxf(yawSpeedInDrift, -s_GlobalDriftParams.mMaxYawSpeedInDrift), s_GlobalDriftParams.mMaxYawSpeedInDrift);
                Debug("Yaw speed = %g\n", yawSpeedInDrift);
                SetVehicleYaw(nfsVehicle, angVel.y, yawSpeedInDrift, dT);
            }
            driftComp.mvfTimeSteerLeft.m128_f32[0] = fminf(driftComp.mvfTimeSteerLeft.m128_f32[0] + dT, s_GlobalDriftParams.mMaxSteeringTime);
        }
        else if (driftComp.pad_0017 != DriftState_Entering)
        {
            driftComp.pad_0017 = DriftState_Exiting;
            Debug("Exiting drift!\n");
        }
        else if (isCountersteering)
        {
            driftComp.pad_0017 = DriftState_Out;
            Debug("Exited drift!\n");
        }
        else if (driftComp.currentYawTorque != 0.f)
        {
            // since we still haven't completely entered/exited the drift, we need to get the ratio of the car's current angle to the angle to enter a drift
            const float saRatio = absSlipAngle / angleToEnterDrift;
            const float yawAccelScaleForHighSpeed = s_GlobalDriftParams.mYawAccelScaleForHighSpeed;
            const float yawAccelScaleForLowSpeed  = s_GlobalDriftParams.mYawAccelScaleForLowSpeed;
            const float minSpeedForDrift = s_GlobalDriftParams.mLowSpeedForYawAccelScale;
            const float maxSpeedForDrift = s_GlobalDriftParams.mHighSpeedForYawAccelScale;
            float speedRatio = 0.f;
            if (minSpeedForDrift != maxSpeedForDrift)
                speedRatio = (fabsf(speedMps) - minSpeedForDrift) / (maxSpeedForDrift - minSpeedForDrift);

            // yaw acceleration scales with speed
            float yawAccelScaleVsSpeed = (yawAccelScaleForHighSpeed - yawAccelScaleForLowSpeed) * speedRatio + yawAccelScaleForLowSpeed;
            if (yawAccelScaleForLowSpeed >= yawAccelScaleForHighSpeed)
                yawAccelScaleVsSpeed = fminf(fmaxf(yawAccelScaleVsSpeed, yawAccelScaleForHighSpeed), yawAccelScaleForLowSpeed);
            else
                yawAccelScaleVsSpeed = fminf(fmaxf(yawAccelScaleVsSpeed, yawAccelScaleForLowSpeed), yawAccelScaleForHighSpeed);

            // invert slip angle ratio here so that yaw accel is subtracted or stops being added when the slip angle is >= the angle to enter a drift
            // a much larger yaw accel amount is used here compared to when a drift has been fully entered
            // this is in order to make it easier to fully enter a controlled drift at lower speeds or when there might be too much grip to kick the back out
            float newYawSpeed = driftComp.currentYawTorque + (driftComp.mvfMaintainedSpeed.m128_f32[0] * (1.f - saRatio) * externalAngVelScale * yawAccelScaleVsSpeed);
            Debug("Yaw speed = %g\n", newYawSpeed);
            SetVehicleYaw(nfsVehicle, angVel.y, newYawSpeed, dT);
        }

        driftComp.counterSteeringInDrift = steering * saDegrees < 0.f;
        if (driftComp.counterSteeringInDrift)
        {
            float yawSpeed = Dot(angVel, vUp);
            float newYawSpeed = yawSpeed;
            float saRatio = fminf(absSlipAngle / 65.f, 1.f);
            newYawSpeed += (s_GlobalDriftParams.mCSYawScaleForZeroSteerTime * s_GlobalDriftParams.mYawAccelScale) * (saRatio * s_GlobalDriftParams.mSlipAngleRatioScale + 1.f) * dT * steering;
            // reset yaw speed to zero when the target yaw has the opposite sign of the original yaw
            // this helps to control the tail kicking out the other way when exiting a drift
            if (yawSpeed * newYawSpeed < 0.f)
                newYawSpeed = 0.f;

            Debug("Yaw speed = %g\n", newYawSpeed);
            SetVehicleYaw(nfsVehicle, angVel.y, newYawSpeed, dT);
        }

        if (driftComp.pad_0017 != DriftState_Out)
            driftComp.currentYawTorque = angVel.y;
        else
            RevivalDriftComponent::ResetDrift(driftComp);
    }
}

float DriftRearFrictionData[] = { 1.1f, 0.95f, 0.87f, 0.77f, 0.67f, 0.6f, 0.51f, 0.43f, 0.37f, 0.34f };
Table DriftRearFrictionTable(10, 0.f, 1.f, 9.f, DriftRearFrictionData);
vec2 DriftStabilizerData[] =
{
    vec2(0.f, 0.f),
    vec2(0.2617994f, 0.1f),
    vec2(0.52359879f, 0.45f),
    vec2(0.78539819f, 0.85f),
    vec2(1.0471976f, 0.95f),
    vec2(1.5533431f, 1.15f),
    vec2(1.5707964f, 0.f)
};
tGraph<float> DriftStabilizerTable(DriftStabilizerData, 7);
void MW05Drift_Update(class NFSVehicle& nfsVehicle, class DriftComponent& driftComp)
{
    Matrix44 matrix;
    RaceRigidBody_GetTransform(driftComp.mpChassisRigidBody, &matrix);

    vec4 angVel = SimdToVec4(nfsVehicle.m_lastLocalAngularVelocity);
    int numWheelsOnGround = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (nfsVehicle.m_grounddata.isgroundValid[i])
            ++numWheelsOnGround;
    }

    float drift_change = 0.f;
    switch (driftComp.pad_0017)
    {
    case DriftState_Out:
    case DriftState_Exiting:
        // the drift value will decrement by (dT * 2) when not drifting or exiting a drift
        drift_change = -2.f;
        break;
    case DriftState_Entering:
    case DriftState_In:
        // the drift value will increment by (dT * 8) when entering and holding a drift
        drift_change = 8.f;
        break;
    default:
        break;
    }

    driftComp.mvfDriftScale.m128_f32[0] += drift_change * nfsVehicle.m_currentUpdateDt;
    // clamp the drift value between 0 and 1
    if (driftComp.mvfDriftScale.m128_f32[0] <= 0.f)
    {
        driftComp.pad_0017 = DriftState_Out;
        driftComp.mvfDriftScale.m128_f32[0] = 0.f;
    }
    else if (driftComp.mvfDriftScale.m128_f32[0] >= 1.f)
    {
        driftComp.pad_0017 = DriftState_In;
        driftComp.mvfDriftScale.m128_f32[0] = 1.f;
    }

    float slipangle = RadiansToAngle(nfsVehicle.m_sideSlipAngle);
    if (driftComp.pad_0017 > DriftState_Entering)
    {
        float steering = nfsVehicle.m_raceCarInputState.inputSteering;
        if ((angVel.y * slipangle) < 0.f
            && fabsf(slipangle) <= 0.25f && nfsVehicle.m_forwardSpeed > MphToMps(30.00005f)
            && (nfsVehicle.m_steeringOutputDirection * slipangle) <= 0.f && fabsf(slipangle) > DegreesToAngle(12.f))
        {
            driftComp.pad_0017 = DriftState_In;
        }
        else if ((steering * slipangle) * nfsVehicle.m_raceCarInputState.inputGas > DegreesToAngle(12.f) && nfsVehicle.m_forwardSpeed > MphToMps(30.00005f))
        {
            driftComp.pad_0017 = DriftState_In;
        }
        else if (!((fabsf(slipangle) * nfsVehicle.m_raceCarInputState.inputGas) > DegreesToAngle(12.f)))
        {
            driftComp.pad_0017 = DriftState_Exiting;
        }
        else
        {
            driftComp.pad_0017 = DriftState_Entering;
        }
    }
    else if (nfsVehicle.m_forwardSpeed > MphToMps(30.00005f) && (nfsVehicle.m_raceCarInputState.inputEBrake > 0.5f || fabsf(slipangle) > DegreesToAngle(12.f)))
    {
        driftComp.pad_0017 = DriftState_Entering;
    }

    if (driftComp.mvfDriftScale.m128_f32[0] > 0.f)
    {
        float yaw = angVel.y;
        // chassis slip angle is stored as a value in the range [-1,1]
        // so multiplying by 2pi gives the entire possible angle range in radians
        float slipangle_radians = slipangle * (2.f * PI);

        // apply yaw damping torque
        if ((yaw * slipangle_radians) < 0.f && numWheelsOnGround >= 2)
        {
            float damping;
            DriftStabilizerTable.GetValue(damping, fabsf(slipangle_radians));
            float yaw_coef = nfsVehicle.m_originalInertiaTensor.m128_f32[1] * driftComp.mvfDriftScale.m128_f32[0] * damping * yaw * 4.f * driftComp.mpParams->driftScaleParams->Drift_angular_damping;
            vec4 moment(yaw_coef);
            // multiply up vector by yaw coefficient to get the final amount of damping to apply
            moment *= matrix.yAxis;
            vec4 timestep(nfsVehicle.m_currentUpdateDt);
            AddWorldTorqueLogged(driftComp.mpChassisRigidBody, &moment.simdValue, &timestep.simdValue);
        }

        // detect counter steering
        //float countersteer = 0.f;
        //if ((slipangle_radians * state.steer_input) > 0.f)
        //    countersteer = fabsf(state.steer_input);
        //
        //float abs_slipangle = fabsf(slipangle_radians);
        //float abs_yaw = fabsf(yaw);
        //float driftmult_rear = DriftRearFrictionTable.GetValue(((abs_yaw + abs_slipangle) * 0.5f + countersteer * 4.f) * driftComp.mvfDriftScale.m128_f32[0]);
        //mTires[2]->mDriftFriction = driftmult_rear;
        //mTires[3]->mDriftFriction = driftmult_rear;
    }
}
