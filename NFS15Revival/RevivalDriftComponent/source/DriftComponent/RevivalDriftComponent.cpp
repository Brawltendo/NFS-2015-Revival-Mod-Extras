#include "pch.h"
#include "RevivalDriftComponent.h"
#include "NFSClasses.h"
#include "util/memoryutils.h"
#include <algorithm>

#ifdef _DEBUG
#include "util/debug/render/DebugRendererFB.h"
#include <sstream>


static NFSVehicle* s_playerVehicle = nullptr;

void RevivalDriftComponent::PreUpdate_Debug(NFSVehicle& nfsVehicle)
{
    // get the player vehicle pointer
    /*if (s_playerVehicle == nullptr)
    {
        uintptr_t game = (uintptr_t)GetModuleHandle(NULL);
        std::vector<size_t> offsets { 0x0, 0x28 };
        uintptr_t vehicleAddress = DerefPtr(game + 0x2C431A0, offsets);

        if (vehicleAddress == (uintptr_t)&nfsVehicle)
            s_playerVehicle = &nfsVehicle;
    }*/
}

fb::RaceVehicleJobHandler* fb::RaceVehicleJobHandler::m_instance = (RaceVehicleJobHandler*)0x142C431A0;

void GetPlayerVehiclePtr(NFSVehicle& nfsVehicle)
{
    static float showTextTimer = 0.f;

    //std::stringstream str;
    //str << "NfsVehicle Address:"; str << std::hex; str << fb::RaceVehicleJobHandler::m_instance->m_playerVehicle;
    //fb::g_debugRender->drawText(-0.8f, 0.f, str.str().c_str(), fb::Color32(255u, 0u, 0u, 255u), 2.5f);

    if (fb::RaceVehicleJobHandler::m_instance->m_vehicles[0] == &nfsVehicle)
    {
        if (s_playerVehicle != &nfsVehicle)
            s_playerVehicle = &nfsVehicle;

        if (showTextTimer <= 0.f)
            showTextTimer = 3.f;

        if (showTextTimer > 0.f)
        {
            uint8_t alpha = showTextTimer / 3.f * 255u;
            fb::g_debugRender->drawText(-0.8f, 0.f, "Found player vehicle ptr!", fb::Color32(255u, 0u, 0u, alpha), 3.f);
            showTextTimer = fmaxf(showTextTimer - fb::RaceVehicleJobHandler::m_instance->m_vehicles[0]->m_currentUpdateDt, 0.f);
        }
    }
}

#endif

void RevivalDriftComponent::PreUpdate(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int& numWheelsOnGround)
{
#ifdef _DEBUG
    
#endif // _DEBUG
    //GetPlayerVehiclePtr(nfsVehicle);
    DebugLogPrint("Player vehicle ptr: %I64X\n", fb::RaceVehicleJobHandler::m_instance->m_vehicles[0]);

    float steering = nfsVehicle.m_raceCarInputState.inputSteering;
    DriftEntryReason oldEntryReason = (DriftEntryReason)driftComp.someEnum;

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

            // set up initial drift scale based on how a drift was entered
            // if multiple drift entry conditions have been met, there will be priority on certain actions over others
            // handbraking has the highest priority, braking has the second highest, and gas stabbing has the lowest
            // drift entry from slip angle doesn't actually use anything that the other entry methods use, but the scale should be initialized regardless
            if (isHandbraking || isSlipping)
            {
                driftComp.mvfMaintainedSpeed.m128_f32[1] = 1.f;
                driftComp.someEnum = isHandbraking ? DriftEntryReason_Handbraking : DriftEntryReason_SlipAngle;
            }
            else if (isBraking)
            {
                driftComp.mvfMaintainedSpeed.m128_f32[1] = 0.4f;
                driftComp.someEnum = DriftEntryReason_Braking;
            }
            else if (isGasStab)
            {
                driftComp.mvfMaintainedSpeed.m128_f32[1] = 0.225f;
                driftComp.someEnum = DriftEntryReason_GasStab;
            }
            driftComp.currentYawTorque = 0.f;

        #ifdef _DEBUG
            {
                static float showTextTimer = 0.f;

                // draw debug only for the player vehicle
                if (fb::RaceVehicleJobHandler::m_instance->m_vehicles[0] == &nfsVehicle)
                {
                    if (showTextTimer <= 0.f)
                    {
                        showTextTimer = 3.f;
                    }
                         
                    if (showTextTimer > 0.f)
                    {
                        std::stringstream str;
                        switch (driftComp.someEnum)
                        {
                        DriftEntryReason_None:
                            str << "";
                            break;
                        DriftEntryReason_SlipAngle:
                            str << Stringize(DriftEntryReason_SlipAngle);
                            break;
                        DriftEntryReason_Handbraking:
                            str << Stringize(DriftEntryReason_Handbraking);
                            break;
                        DriftEntryReason_Braking:
                            str << Stringize(DriftEntryReason_Braking);
                            break;
                        DriftEntryReason_GasStab:
                            str << Stringize(DriftEntryReason_GasStab);
                            break;
                        }
                        uint8_t alpha = /*showTextTimer / 3.f * */255u;
                        fb::g_debugRender->drawText(-0.8f, 0.f, str.str().c_str(), fb::Color32(255u, 0u, 0u, alpha), 1.f);
                        showTextTimer = fmaxf(showTextTimer - nfsVehicle.m_currentUpdateDt, 0.f);
                    }
                }
            }
        #endif

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

        vec4 sideForce(force * sign(dpSideVel) * sideForceScale);
        sideForce *= vRight;
        vec4 forwardForce(force * forwardForceScale);
        forwardForce *= vFwd;

    #ifdef _DEBUG
        // draw debug forces only for the player vehicle
        //if (fb::RaceVehicleJobHandler::m_instance->m_vehicles[0] == &nfsVehicle)
        {
            vec4 pos(nfsVehicle.m_raceCarInputState.matrix.wAxis);

            // draw side force
            vec4 scaledSideForce(sideForce / 1000.f + pos);
            fb::g_debugRender->drawLine3d((float*)&pos, (float*)&scaledSideForce, fb::Color32(255u, 0u, 0u, 255u));
            // draw forward force
            vec4 scaledFwdForce(forwardForce / 1000.f + pos);
            fb::g_debugRender->drawLine3d((float*)&pos, (float*)&scaledFwdForce, fb::Color32(0u, 255u, 0u, 255u));
        }
    #endif

        vec4 autoSteerForce = sideForce + forwardForce;
        vec4 timestep(nfsVehicle.m_currentUpdateDt);
        AddWorldCOMForceLogged(driftComp.mpChassisRigidBody, &autoSteerForce.simdValue, &timestep.simdValue);
    }
}

void RevivalDriftComponent::ResetDrift(DriftComponent& driftComp)
{
    driftComp.currentYawTorque = 0.f;
    driftComp.someEnum = DriftEntryReason_None;
    driftComp.mvfTimeSteerLeft.m128_f32[0] = 0.f;
    driftComp.mvfTimeSteerLeft.m128_f32[1] = 0.f;
    driftComp.mvfMaintainedSpeed.m128_f32[0] = 0.f;
    driftComp.mvfMaintainedSpeed.m128_f32[1] = 1.f;
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
        if (numWheelsOnGround <= 1 || speedMps <= s_GlobalDriftParams.mSpeedToExitDrift || speedMps <= MphToMps(13.f))
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
            saRatio = clamp01(saRatio);
            float speedRatio = (fwdVelMag - s_GlobalDriftParams.mSpeedForZeroSpeedMaintenance) / (s_GlobalDriftParams.mSpeedForFullSpeedMaintenance - s_GlobalDriftParams.mSpeedForZeroSpeedMaintenance);
            speedRatio = clamp01(speedRatio);
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
                yawSpeedInDrift = clamp(yawSpeedInDrift, -s_GlobalDriftParams.mMaxYawSpeedInDrift, s_GlobalDriftParams.mMaxYawSpeedInDrift);
                DebugLogPrint("Yaw speed = %g\n", yawSpeedInDrift);
                SetVehicleYaw(nfsVehicle, angVel.y, yawSpeedInDrift, dT);
            }
            driftComp.mvfTimeSteerLeft.m128_f32[0] = fminf(driftComp.mvfTimeSteerLeft.m128_f32[0] + dT, s_GlobalDriftParams.mMaxSteeringTime);
        }
        else if (driftComp.pad_0017 != DriftState_Entering)
        {
            // start exiting the drift if below the minimum slip angle but are past entering the drift
            driftComp.pad_0017 = DriftState_Exiting;
            DebugLogPrint("Exiting drift!\n");
        }
        else if (isCountersteering)
        {
            // if we're countersteering while entering the drift and are below the minimum slip angle, end the drift
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
                yawAccelScaleVsSpeed = clamp(yawAccelScaleVsSpeed, yawAccelScaleForHighSpeed, yawAccelScaleForLowSpeed);
            else
                yawAccelScaleVsSpeed = clamp(yawAccelScaleVsSpeed, yawAccelScaleForLowSpeed, yawAccelScaleForHighSpeed);

            float newYawSpeed = driftComp.currentYawTorque;
            // invert slip angle ratio here so that yaw accel is subtracted or stops being added when the slip angle is >= the angle to enter a drift
            // a much larger yaw accel amount is used here compared to when a drift has been fully entered
            // this is in order to make it easier to fully enter a controlled drift at lower speeds or when there might be too much grip to kick the back out
            // yaw accel is also scaled based on the drift entry action 
            newYawSpeed += driftComp.mvfMaintainedSpeed.m128_f32[0] * (1.f - saRatio) * externalAngVelScale * yawAccelScaleVsSpeed * driftComp.mvfMaintainedSpeed.m128_f32[1];
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