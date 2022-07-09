#include "pch.h"
#include "RevivalDriftComponent.h"
#include "NFSClasses.h"
#include "util/memoryutils.h"
#include <algorithm>
#include "util/debug/debug.h"

#ifdef _DEBUG
#include "util/debug/render/DebugRendererFB.h"

fb::RaceVehicleJobHandler** fb::RaceVehicleJobHandler::m_instance = (RaceVehicleJobHandler**)0x142C431A0;
extern std::stringstream debug_controlledDriftStr;
extern std::stringstream debug_driftEntryReasonStr;
extern __m128 debug_carPos;
extern __m128 debug_sideForceWorldPos;
extern __m128 debug_fwdForceWorldPos;

#endif

void RevivalDriftComponent::PreUpdate(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int& numWheelsOnGround, const DriftParameters& params)
{
    for (int i = 0; i < 4; ++i)
    {
        if (nfsVehicle.m_grounddata.isgroundValid[i])
            ++numWheelsOnGround;
    }
    
#ifdef _DEBUG
    if ((*fb::RaceVehicleJobHandler::m_instance)->m_vehicles[0] == &nfsVehicle)
    {
        debug_driftEntryReasonStr.str(std::string());
    }
#endif

    float steering = nfsVehicle.m_raceCarInputState.inputSteering;
    DriftEntryReason oldEntryReason = (DriftEntryReason)driftComp.someEnum;

    if (driftComp.pad_0017 == DriftState_Out && fabsf(steering) > params.mSteeringThreshold)
    {
        // assist level is determined by slider in the handling tuning menu
        // full assists are enabled by default globally
        const float assistSliderVal = nfsVehicle.m_performanceModificationComponent->m_modifiers[ATM_TorqueSplitInDrift].modifier;
        int assistLevel;
        if (assistSliderVal <= 0.1f)
            assistLevel = DriftAssistLevel_Minimal;
        else if (assistSliderVal > 0.1f && assistSliderVal < 0.9f)
            assistLevel = DriftAssistLevel_Balanced;
        else
            assistLevel = DriftAssistLevel_Full;

        bool isBraking = assistLevel == DriftAssistLevel_Full && nfsVehicle.m_raceCarInputState.inputBrake > params.mBrakeThreshold;
        bool isHandbraking = (assistLevel == DriftAssistLevel_Full || assistLevel == DriftAssistLevel_Balanced) && nfsVehicle.m_wasHandbrakeOnLastUpdate;
        bool isGasStab = false;
        if (assistLevel == DriftAssistLevel_Full && nfsVehicle.m_input->m_throttle > params.mThrottleThreshold)
            isGasStab = driftComp.mvfPreviousGasInput.m128_f32[0] > params.mMinTimeForGasStab && driftComp.mvfPreviousGasInput.m128_f32[0] < params.mMaxTimeForGasStab;

        // use drift config fields for compatibility with performance mod system
        const float angleToEnterDrift = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_MinimumAngleForDrift, driftComp.mpParams->driftScaleParams->Slip_angle_to_enter_drift);
        const float slipAngle = nfsVehicle.m_sideSlipAngle * 180.f * 0.31830987f;
        bool isSlipping = fabsf(slipAngle) >= angleToEnterDrift;

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
            DebugLogPrint("Entered drift!\n");
        }
    }

#ifdef _DEBUG
    static float showTextTimer = 0.f;

    // draw debug only for the player vehicle
    if ((*fb::RaceVehicleJobHandler::m_instance)->m_vehicles[0] == &nfsVehicle)
    {
        if (showTextTimer <= 0.f && oldEntryReason != driftComp.someEnum)
        {
            showTextTimer = 3.f;
        }

        //if (showTextTimer > 0.f)
        {
            switch (driftComp.someEnum)
            {
                default:
                case DriftEntryReason_None:
                    debug_driftEntryReasonStr.clear();
                    break;
                case DriftEntryReason_SlipAngle:
                    debug_driftEntryReasonStr << Stringize(DriftEntryReason_SlipAngle);
                    break;
                case DriftEntryReason_Handbraking:
                    debug_driftEntryReasonStr << Stringize(DriftEntryReason_Handbraking);
                    break;
                case DriftEntryReason_Braking:
                    debug_driftEntryReasonStr << Stringize(DriftEntryReason_Braking);
                    break;
                case DriftEntryReason_GasStab:
                    debug_driftEntryReasonStr << Stringize(DriftEntryReason_GasStab);
                    break;
            }
        }
    }
#endif

    // update throttle release timer
    // we need to use the raw throttle value from VehicleInput because any other throttle input value in NFSVehicle is set to zero when changing gears
    // this obviously makes for a bad experience when drift entry via gas stab is enabled so let's not have that
    if (nfsVehicle.m_input->m_throttle <= params.mThrottleThreshold)
        driftComp.mvfPreviousGasInput.m128_f32[0] += nfsVehicle.m_currentUpdateDt;
    else
        driftComp.mvfPreviousGasInput.m128_f32[0] = 0.f;
}

void RevivalDriftComponent::UpdateStabilizationForces(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int numWheelsOnGround, const DriftParameters& params)
{
    const float LowAngleForAutoDriftSteer  = DegreesToRadians(15.f);
    const float HighAngleForAutoDriftSteer = DegreesToRadians(40.f);
    const float MaxAutoSteerAngle = DegreesToRadians(90.f);
    const float MinSpeedForAutoDriftSteer = MphToMps(30.f);

#ifdef _DEBUG
    // draw debug forces only for the player vehicle
    if ((*fb::RaceVehicleJobHandler::m_instance)->m_vehicles[0] == &nfsVehicle)
    {
        debug_carPos = vec4::s_Zero.simdValue;
        debug_sideForceWorldPos = vec4::s_Zero.simdValue;
        debug_fwdForceWorldPos = vec4::s_Zero.simdValue;
    }
#endif

    // car must be grounded (3+ wheels on the ground) in order to have these forces applied
    if (numWheelsOnGround >= 3)
    {
        Matrix44 matrix;
        RaceRigidBody_GetTransform(driftComp.mpChassisRigidBody, &matrix);
        vec4& vUp = SimdToVec4(matrix.yAxis);
        vec4& vFwd = SimdToVec4(matrix.zAxis);
        vec4 timestep(nfsVehicle.m_currentUpdateDt);

        if ((nfsVehicle.m_sideSlipAngle <= MaxAutoSteerAngle && nfsVehicle.m_sideSlipAngle >= -MaxAutoSteerAngle) && nfsVehicle.m_forwardSpeed >= MinSpeedForAutoDriftSteer)
        {
            vec4& vRight = SimdToVec4(matrix.xAxis);
            vec4& linVel = SimdToVec4(nfsVehicle.m_linearVelocity);
            float angleRatio = (fabsf(nfsVehicle.m_sideSlipAngle) - LowAngleForAutoDriftSteer) / (HighAngleForAutoDriftSteer - LowAngleForAutoDriftSteer);
            float dpSideVel = Dot(linVel, vRight);
            // use drift config fields for compatibility with performance mod system
            const float sideForceScale = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_DefaultSteering, driftComp.mpParams->driftScaleParams->Default_steering);
            const float extForceMagnitude = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_SideForceMagnitude, driftComp.mpParams->driftScaleParams->Side_force_magnitude);
            const float forwardForceScale = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_CounterSteeringRemapping, driftComp.mpParams->driftScaleParams->Counter_steering_remapping);
            // overall force scales with the car's angle
            float force = fminf(fmaxf(angleRatio, 0.f), 1.f) * extForceMagnitude * nfsVehicle.m_originalMass;
            // scale force by clamped throttle
            // with zero throttle 10% of the total force will be applied so momentum can still be maintained in corners when the throttle needs to be released
            force *= fminf(fmaxf(nfsVehicle.m_raceCarInputState.inputGas, 0.1f), 1.f);

            // counter steering side force scale is only active during a controlled drift
            float countersteerMultiplier = 1.f;
            if (driftComp.counterSteeringInDrift)
                countersteerMultiplier += clamp(fabsf(nfsVehicle.m_steeringOutputDirection), 0.f, 0.6f);
            vec4 sideForce(force * sign(dpSideVel) * sideForceScale * countersteerMultiplier);
            sideForce *= vRight;
            vec4 forwardForce(force * forwardForceScale);
            forwardForce *= vFwd;

        #ifdef _DEBUG
            // draw debug forces only for the player vehicle
            if ((*fb::RaceVehicleJobHandler::m_instance)->m_vehicles[0] == &nfsVehicle)
            {
                vec4 pos(nfsVehicle.m_raceCarInputState.matrix.wAxis);
                debug_carPos = pos.simdValue;

                debug_sideForceWorldPos = (sideForce / 1000.f + pos).simdValue;
                debug_fwdForceWorldPos = (forwardForce / 1000.f + pos).simdValue;

                //std::stringstream str;
                //str << "Applying stabilization forces!";
                //fb::g_debugRender->drawText(-0.5f, 0.1f, str.str().c_str(), fb::Color32(0u, 255u, 0u, 255u), 1.f);
            }
        #endif

            vec4 autoSteerForce = sideForce + forwardForce;
            AddWorldCOMForceLogged(driftComp.mpChassisRigidBody, &autoSteerForce.simdValue, &timestep.simdValue);
        }

        // add counter yaw to give some extra weight to the steering
        vec4 torque(nfsVehicle.m_raceCar->mChassisResult.outputState.torqueAppliedToCar);
        torque.w = 0.f;
        float yaw = Dot(torque, vUp);
        float counterYaw = yaw * driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_AligningTorqueEffectInDrift, nfsVehicle.m_data->Steering->AligningTorqueEffectInDrift) - yaw;
        /*bool countersteering = nfsVehicle.m_raceCarInputState.inputSteering * -nfsVehicle.m_sideSlipAmount < 0.f;
        if (fabsf(nfsVehicle.m_sideSlipAmount) > 0.4f && nfsVehicle.m_forwardSpeed >= MinSpeedForAutoDriftSteer && countersteering)
        {
            const float minScale = nfsVehicle.m_data->Drift->Drift_scale_from_handbrake;
            const float maxScale = nfsVehicle.m_data->Drift->Drift_scale_decay;
            float yawVsSlip = counterYaw * map(fabsf(nfsVehicle.m_sideSlipAmount), 0.4f, 1.f, minScale, maxScale);
            counterYaw = -yawVsSlip;
        }*/
        torque = vUp.simdValue * counterYaw;
        
        AddWorldTorqueLogged(driftComp.mpChassisRigidBody, &torque.simdValue, &timestep.simdValue);
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

void RevivalDriftComponent::Update(NFSVehicle& nfsVehicle, DriftComponent& driftComp, int numWheelsOnGround, const DriftParameters& params)
{
#ifdef _DEBUG
    if ((*fb::RaceVehicleJobHandler::m_instance)->m_vehicles[0] == &nfsVehicle)
    {
        debug_controlledDriftStr.str(std::string());
    }
#endif
    float latFrictionRear = driftComp.m_performanceModificationComponent->GetModifiedValue(ATM_LateralFrictionScaleRear, nfsVehicle.m_data->Tire->AnalyticalTireDataRear->LateralFrictionScale);


    if (driftComp.pad_0017 != DriftState_Out)
    {
        // start exiting drift when the throttle has been released for long enough
        if (driftComp.mvfPreviousGasInput.m128_f32[0] > params.mOffGasTimeForExitingDrift)
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
        if (numWheelsOnGround <= 1 || speedMps <= params.mSpeedToExitDrift || speedMps <= MphToMps(13.f))
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

        if (absSlipAngle < params.mAngleToExitDrift && (driftComp.pad_0017 == DriftState_Exiting || isCountersteering))
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
            float saRatio = (absSlipAngle - angleToEnterDrift) / (params.mAngleForMaxDriftScale - angleToEnterDrift);
            saRatio = clamp01(saRatio);
            float speedRatio = (fwdVelMag - params.mSpeedForZeroSpeedMaintenance) / (params.mSpeedForFullSpeedMaintenance - params.mSpeedForZeroSpeedMaintenance);
            speedRatio = clamp01(speedRatio);
            vec4 normalizedVel = fwdVel * (1.f / fwdVelMag);
            float maintainSpeedAmount = params.mDriftMaintainSpeedAmount * dT * externalForcesScale * saRatio * speedRatio;
            vec4 maintainSpeedForce = linVel + ((vFwd - normalizedVel) * params.mDriftMaintainSpeedScale + normalizedVel) * maintainSpeedAmount;
            // may or may not add this back at some point
            // don't really see a use for it with RevivalDriftComponent::UpdateStabilizationForces in place though since that's already being used to maintain side and forward speed
            //SetLinearVelocity(nfsVehicle.dynamicPhysEnt, (uint16_t*)&(nfsVehicle.pad_00C8[0]), &maintainSpeedForce.simdValue);

            if (driftComp.currentYawTorque != 0.f && !driftComp.counterSteeringInDrift)
            {
                float steerTimeRatio = driftComp.mvfTimeSteerLeft.m128_f32[0] / params.mMaxSteeringTime;
                bool countersteeringInDrift = saDegrees * steering <= 0.f;
                float steeringScaleByTimeMin;
                float steeringScaleByTimeMax;
                if (countersteeringInDrift)
                {
                    steeringScaleByTimeMin = params.mCSYawScaleForZeroSteerTime;
                    steeringScaleByTimeMax = params.mCSYawScaleForMaxSteerTime;
                }
                else
                {
                    steeringScaleByTimeMin = params.mYawScaleForZeroSteerTime;
                    steeringScaleByTimeMax = params.mYawScaleForMaxSteerTime;
                }

                float yawScaleVsSteerTime = steerTimeRatio * (steeringScaleByTimeMax - steeringScaleByTimeMin) + steeringScaleByTimeMin;
                float steeringYawAccel = steering * (saRatio * params.mSlipAngleRatioScale + 1.f) * params.mYawAccelScale * yawScaleVsSteerTime * dT;
                float yawSpeedInDrift = driftComp.currentYawTorque + steeringYawAccel;
                if ((fwdVelMag + maintainSpeedAmount) > params.mSpeedForMidDriftScale && (sign(driftComp.currentYawTorque) * yawSpeedInDrift) > fabsf(driftComp.currentYawTorque))
                {
                    float extraYaw = (absSlipAngle - params.mAngleForMidDriftScale) / (params.mAngleForMaxDriftScale - params.mAngleForMidDriftScale);
                    if (extraYaw > 0.f)
                        yawSpeedInDrift -= extraYaw * steeringYawAccel;
                }

                const float saSign = sign(saDegrees);
                float yawAccelScale = countersteeringInDrift ? params.mCountersteerYawAccelScale : 1.f;
                float ninetyDegSlip = bodySlipAngle * -DegreesToRadians(90.f);
                // add counter yaw to steer out from a 90 degree slip angle
                float autoCsYaw = (ninetyDegSlip * ninetyDegSlip * saSign * params.mAutoCSYawScaleForHighSA) + (ninetyDegSlip * params.mAutoCSYawScaleForLowSA);
                float autoYawAccel = (saSign * params.mYawAutoAccelAmount + autoCsYaw) * dT * yawAccelScale;
                if (yawSpeedInDrift * (yawSpeedInDrift - autoYawAccel) < 0.f || yawSpeedInDrift * driftComp.currentYawTorque < 0.f)
                    yawSpeedInDrift = 0.f;
                else
                    yawSpeedInDrift -= autoYawAccel;

                yawSpeedInDrift = (yawSpeedInDrift - driftComp.currentYawTorque) * externalAngVelScale + driftComp.currentYawTorque;
                // clamp to mMaxYawSpeedInDrift
                yawSpeedInDrift = clamp(yawSpeedInDrift, -params.mMaxYawSpeedInDrift, params.mMaxYawSpeedInDrift);
                DebugLogPrint("Yaw speed = %g\n", yawSpeedInDrift);
                SetVehicleYaw(nfsVehicle, angVel.y, yawSpeedInDrift, dT);

                const float FrictionLossForLowSlipAngle  = 0.825f;
                const float FrictionLossForHighSlipAngle = 0.9f;
                latFrictionRear *= saRatio * (FrictionLossForHighSlipAngle - FrictionLossForLowSlipAngle) + FrictionLossForLowSlipAngle;
            }
            driftComp.mvfTimeSteerLeft.m128_f32[0] = fminf(driftComp.mvfTimeSteerLeft.m128_f32[0] + dT, params.mMaxSteeringTime);
        }
        else if (driftComp.pad_0017 != DriftState_Entering)
        {
            // start exiting the drift if below the minimum slip angle but past entering the drift
            driftComp.pad_0017 = DriftState_Exiting;
            DebugLogPrint("Exiting drift!\n");
        }
        else if (isCountersteering)
        {
            // if we're countersteering while entering the drift and below the minimum slip angle, end the drift
            driftComp.pad_0017 = DriftState_Out;
            DebugLogPrint("Exited drift!\n");
        }
        else if (driftComp.currentYawTorque != 0.f)
        {
            // since we still haven't completely entered/exited the drift, we need to get the ratio of the car's current angle to the angle to enter a drift
            const float saRatio = absSlipAngle / angleToEnterDrift;
            const float yawAccelScaleForHighSpeed = params.mYawAccelScaleForHighSpeed;
            const float yawAccelScaleForLowSpeed  = params.mYawAccelScaleForLowSpeed;
            const float minSpeedForDrift = params.mLowSpeedForYawAccelScale;
            const float maxSpeedForDrift = params.mHighSpeedForYawAccelScale;
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
            latFrictionRear /= yawAccelScaleVsSpeed;
        }

    #ifdef _DEBUG
        // draw debug only for the player vehicle
        if ((*fb::RaceVehicleJobHandler::m_instance)->m_vehicles[0] == &nfsVehicle)
        {
            debug_controlledDriftStr << "Controlled drift active!";
        }
    #endif

        driftComp.counterSteeringInDrift = steering * saDegrees < 0.f;
        // do countersteer yaw
        // having separate yaw accel for countersteering is done for complete control over the entire drift
        // so here we're just counteracting the accel being applied above to give a better feeling of weight to the drift
        if (driftComp.counterSteeringInDrift)
        {
            float yawSpeed = Dot(angVel, vUp);
            float newYawSpeed = yawSpeed;
            float saRatio = fminf(absSlipAngle / 65.f, 1.f);
            newYawSpeed += (params.mCSYawScaleForZeroSteerTime * params.mYawAccelScale) * (saRatio * params.mSlipAngleRatioScale + 1.f) * dT * steering;
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

        nfsVehicle.m_raceCar->mChassis.mTirePatchStaticState[2].criterionTire.lateralFrictionScale = latFrictionRear;
        nfsVehicle.m_raceCar->mChassis.mTirePatchStaticState[3].criterionTire.lateralFrictionScale = latFrictionRear;
    }
}

void RevivalDriftComponent::UpdateHardSteering(NFSVehicle& nfsVehicle)
{
    // car needs to be grounded and not drifting
    if (nfsVehicle.m_lastVehicleState == NFSVehicleState_OnGround && nfsVehicle.m_vehicleState == NFSVehicleState_OnGround && nfsVehicle.m_driftComponent->someEnum == DriftEntryReason_None)
    {
        const float HardSteeringSpeedThreshold = MphToMps(85.f);
        const float HardSteeringThreshold = 0.175f;

        // get how much the car is steering relative to the current max steering
        float steering = fabsf(nfsVehicle.m_raceCarOutputState.wheelSteeringAngleRadians) / fabsf(nfsVehicle.m_steeringOutputDirection);
        bool isHandbraking = nfsVehicle.m_input->m_handBrake > 0.f;
        if (steering >= HardSteeringThreshold && fabsf(nfsVehicle.m_forwardSpeed) >= HardSteeringSpeedThreshold
            && nfsVehicle.m_input->m_throttle > 0.1f && nfsVehicle.m_input->m_brake < 0.1f && !isHandbraking)
        {
            Matrix44 matrix;
            RaceRigidBody_GetTransform(nfsVehicle.m_rigidBodyInterface, &matrix);
            vec4& vFwd = SimdToVec4(matrix.zAxis);
            vec4 lastUpdateForce(nfsVehicle.m_driftComponent->mvfSideForceMagnitude);
            vec4 thisUpdateForce(nfsVehicle.m_raceCarOutputState.forceAppliedToCar);
            thisUpdateForce.w = 0.f;
            float fwdForceDelta = fabsf(Dot(lastUpdateForce, vFwd)) - fabsf(Dot(thisUpdateForce, vFwd));
            if (fwdForceDelta > 0.f)
            {
                const float maintainSpeedScale = nfsVehicle.m_performanceModificationComponent->GetModifiedValue(ATM_GasLetOffYawTorque, nfsVehicle.m_data->Drift->Steering_amount_on_exit_drift);
                vec4 force(vFwd * fwdForceDelta * maintainSpeedScale);
                vec4 timestep(nfsVehicle.m_currentUpdateDt);
                AddWorldCOMForceLogged(nfsVehicle.m_rigidBodyInterface, &force.simdValue, &timestep.simdValue);
            }
        }
    }

    nfsVehicle.m_driftComponent->mvfSideForceMagnitude = nfsVehicle.m_raceCarOutputState.forceAppliedToCar;
    nfsVehicle.m_driftComponent->mvfSideForceMagnitude.m128_f32[3] = 0.f;
}