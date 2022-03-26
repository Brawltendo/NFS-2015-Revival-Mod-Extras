#include "pch.h"
#include "RevivalDriftComponent.h"
#include <algorithm>
#include "NFSClasses.h"

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
            DebugLogPrint("Entered drift!\n");
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
            DebugLogPrint("Exited drift!\n");
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
            DebugLogPrint("Exited drift!\n");
        }
        else if (absSlipAngle >= angleToEnterDrift)
        {
            driftComp.pad_0017 = DriftState_In;
            DebugLogPrint("Drifting!\n");

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
                // probably won't use this though so it's not actually used in the yaw calc
                float autoCsYaw = ((ninetyDegSlip * ninetyDegSlip * saSign * s_GlobalDriftParams.mAutoCSYawScaleForHighSA) + (ninetyDegSlip * s_GlobalDriftParams.mAutoCSYawScaleForLowSA));
                float autoYawAccel = (saSign * s_GlobalDriftParams.mYawAutoAccelAmount /*+ autoCsYaw*/) * dT * yawAccelScale;
                if (yawSpeedInDrift * (yawSpeedInDrift - autoYawAccel) < 0.f || yawSpeedInDrift * driftComp.currentYawTorque < 0.f)
                    yawSpeedInDrift = 0.f;
                else
                    yawSpeedInDrift -= autoYawAccel;

                yawSpeedInDrift = (yawSpeedInDrift - driftComp.currentYawTorque) * externalAngVelScale + driftComp.currentYawTorque;
                // clamp to mMaxYawSpeedInDrift
                yawSpeedInDrift = fminf(fmaxf(yawSpeedInDrift, -s_GlobalDriftParams.mMaxYawSpeedInDrift), s_GlobalDriftParams.mMaxYawSpeedInDrift);
                DebugLogPrint("Yaw speed = %g\n", yawSpeedInDrift);
                SetVehicleYaw(nfsVehicle, angVel.y, yawSpeedInDrift, dT);
            }
            driftComp.mvfTimeSteerLeft.m128_f32[0] = fminf(driftComp.mvfTimeSteerLeft.m128_f32[0] + dT, s_GlobalDriftParams.mMaxSteeringTime);
        }
        else if (driftComp.pad_0017 != DriftState_Entering)
        {
            driftComp.pad_0017 = DriftState_Exiting;
            DebugLogPrint("Exiting drift!\n");
        }
        else if (isCountersteering)
        {
            driftComp.pad_0017 = DriftState_Out;
            DebugLogPrint("Exited drift!\n");
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
            DebugLogPrint("Yaw speed = %g\n", newYawSpeed);
            SetVehicleYaw(nfsVehicle, angVel.y, newYawSpeed, dT);
        }

        driftComp.counterSteeringInDrift = steering * saDegrees < 0.f;
        // do countersteer yaw
        // having separate yaw accel for countersteering is done for complete control over the entire drift
        // so here we're just counteracting the accel being applied above to give a better feeling weight to the drift
        if (driftComp.counterSteeringInDrift)
        {
            float yawSpeed = Dot(angVel, vUp);
            float newYawSpeed = yawSpeed;
            float saRatio = fminf(absSlipAngle / 65.f, 1.f);
            newYawSpeed += (s_GlobalDriftParams.mCSYawScaleForZeroSteerTime * s_GlobalDriftParams.mYawAccelScale) * (saRatio * s_GlobalDriftParams.mSlipAngleRatioScale + 1.f) * dT * steering;
            // reset yaw speed to zero when the target yaw has the opposite sign of the original yaw
            // this helps to control the tail stepping out too fast the other way when exiting a drift
            if (yawSpeed * newYawSpeed < 0.f)
                newYawSpeed = 0.f;

            DebugLogPrint("Yaw speed = %g\n", newYawSpeed);
            SetVehicleYaw(nfsVehicle, angVel.y, newYawSpeed, dT);
        }

        if (driftComp.pad_0017 != DriftState_Out)
            driftComp.currentYawTorque = angVel.y;
        else
            RevivalDriftComponent::ResetDrift(driftComp);
    }
}