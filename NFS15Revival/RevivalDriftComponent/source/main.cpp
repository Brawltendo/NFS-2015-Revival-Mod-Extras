#include "pch.h"
#include "psapi.h"
#include "main.h"
#include "util\utils.h"
#include "DriftComponent\RevivalDriftComponent.h"
#include "NFSClasses.h"
#include <algorithm>

#ifndef NFS_NATIVE_FUNCTIONS

typedef PointGraph8* (__fastcall* _initPointGraph8FromCurveData)(PointGraph8* pointGraphIn, __m128 (*curveData)[10]);
typedef float(__fastcall* _PointGraph8__Evaluate)(int pgCount, float(*pgInX)[8], float(*pgInY)[8], float xVal);
typedef void(__fastcall* _AddWorldCOMForceLogged)(RaceRigidBody* rigidBody, __m128* force, __m128* lfTimeStep);
typedef void(__fastcall* _AddWorldTorqueLogged)(RaceRigidBody* rigidBody, __m128* torque, __m128* lvfTimeStep);
typedef void(__fastcall* _DampPitchYawRoll)(RaceRigidBody* chassis, __m128* pitchDampening, __m128* yawDampening, __m128* rollDampening, __m128* lfTimeStep);
// Returns Frostbite transformation matrix: Side vector = X, up vector = Y, forward vector = Z, location = W
typedef Matrix44* (__fastcall* _GetTransform)(RaceRigidBody* chassis, Matrix44* transformIn);
typedef __m128* (__fastcall* _GetAngularVelocity)(RaceRigidBody* const chassis, __m128* angVelIn);
typedef void(__fastcall* _MaintainDriftSpeed)(DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lDirection, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep);
typedef void(__fastcall* _UpdateDriftScale)(DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lvfSteeringInputIn, HandbrakeComponent* lpHandbrake, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep, __m128* lvfInvTimeStep);
typedef void(__fastcall* _UpdateDriftAngleDegrees)(DriftComponent* const driftComp);
typedef void(__fastcall* _UpdateSideForce)(DriftComponent* driftComp, __m128* lvfAbsDriftScale, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfSteeringInput, __m128* lvfAverageSurfaceGripFactor, __m128* lvfTimeStep);
typedef void(__fastcall* _ApplyDamping)(DriftComponent* driftComp, __m128* lvfAverageSurfaceGripFactor, __m128* lvfDampingScale, __m128* lvfTimeStep);
typedef void(__fastcall* _ApplyDriftForces)(DriftComponent* const driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lvfSteeringInput, __m128* lvfAbsDriftScale, HandbrakeComponent* lpHandbrake, class RaceCarPhysicsObject* lpRaceCar, float lfSpeedMPS, __m128* lvfAverageSurfaceGripFactor, __m128* lvfTimeStep, __m128* lvfInvTimeStep);

_initPointGraph8FromCurveData initPointGraph8FromCurveData = (_initPointGraph8FromCurveData)0x1441B7BF0;
_PointGraph8__Evaluate PointGraph8__Evaluate = (_PointGraph8__Evaluate)0x143F6A8E0;
_AddWorldCOMForceLogged AddWorldCOMForceLogged = (_AddWorldCOMForceLogged)0x144170EE0;
_AddWorldTorqueLogged AddWorldTorqueLogged = (_AddWorldTorqueLogged)0x144170F50;
_DampPitchYawRoll DampPitchYawRoll = (_DampPitchYawRoll)0x1441712D0;
_GetTransform GetTransform = (_GetTransform)0x1441718C0;
_GetAngularVelocity GetAngularVelocity = (_GetAngularVelocity)0x1441717E0;
_MaintainDriftSpeed MaintainDriftSpeed = (_MaintainDriftSpeed)0x1441960E0;
//_UpdateDriftScale UpdateDriftScale = (_UpdateDriftScale)0x144197840;
_UpdateDriftAngleDegrees UpdateDriftAngleDegrees = (_UpdateDriftAngleDegrees)0x144197690;
_UpdateSideForce UpdateSideForce = (_UpdateSideForce)0x144197FC0;
_ApplyDriftForces ApplyDriftForces = (_ApplyDriftForces)0x144193890;
_ApplyDamping ApplyDamping = (_ApplyDamping)0x1441935E0;

#endif // NFS_NATIVE_FUNCTIONS

#ifndef REVIVAL_FUNCTIONS_REGION

float GetLocalAngVelDegrees(RaceRigidBody* chassis)
{
    Matrix44 matrix;
    __m128 angVelOut;
    __m128 upVector = GetTransform(chassis, &matrix)->yAxis;
    __m128 angVel = *GetAngularVelocity(chassis, &angVelOut);

    __m128 mulByUpVec = upVector * angVel;
    __m128 localAngVel = _mm_shuffle_ps(mulByUpVec, mulByUpVec, 2) + _mm_shuffle_ps(mulByUpVec, mulByUpVec, 1) + mulByUpVec;
    __m128 localAngVelFinal = _mm_shuffle_ps(localAngVel, localAngVel, 0);
    return RadiansToDegrees(localAngVelFinal.m128_f32[0]);
}

void UpdateDriftAngle(RaceRigidBody* chassis, DriftComponent* driftComp)
{
    UpdateDriftAngleDegrees(driftComp);
    driftComp->mvfCurrentDriftAngle.m128_f32[0] = sign(GetLocalAngVelDegrees(chassis), 1) * driftComp->mvfCurrentDriftAngle.m128_f32[0];
    // Debug("driftAngle: %f\n", bDrift->driftAngle);
}

void DriftSideForce(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis)
{
    //Debug("driftAngle: %f\n", bDrift->driftAngle);
    Matrix44 matrix;
    __m128 sideVector = GetTransform(chassis, &matrix)->xAxis;
    __m128 timeStep = _mm_shuffle_ps({ nfsVehicle->m_currentUpdateDt }, { nfsVehicle->m_currentUpdateDt }, 0);
    RaceVehicleDriftConfigData* sideForceParams = driftComp->mpParams->sideForceParams;
    UpdateCounterSteeringSideMagnitude(nfsVehicle, driftComp, nfsVehicle->m_input->steeringInput.X, GetLocalAngVelDegrees(chassis), timeStep.m128_f32[0]);
    float absDriftAngle = fabsf(driftComp->mvfCurrentDriftAngle.m128_f32[0]);
    float sideForceMagnitude = -GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_SideForceMagnitude, sideForceParams->Side_force_magnitude);
    float sideForceSpeedRatio = fminf(1.f, fmaxf(0.f, (GetSpeedMph(nfsVehicle) - sideForceParams->Speed_for_no_sideforce) / (sideForceParams->Speed_for_maximum_side_force - sideForceParams->Speed_for_no_sideforce)));
    float angleForSideForceRange = sideForceParams->Minimum_drift_angle_for_side_force - sideForceParams->Drift_angle_for_side_force;
    float angleForSideForceRatio = fminf(angleForSideForceRange, fmaxf(0.f, absDriftAngle - sideForceParams->Drift_angle_for_side_force)) / angleForSideForceRange;
    float driftScaleInfluenceOnSideForce = fminf(fabsf(driftComp->mvfDriftScale.m128_f32[0]), sideForceParams->Drift_scale_for_maximum_side_force);
    float decaySideForceMag = fmaxf(0.f, sideForceMagnitude - sideForceParams->Min_decay_side_force_magnitude / sideForceMagnitude);
    float sideForceDecay = fminf(fmaxf(absDriftAngle - sideForceParams->Drift_angle_for_decay, 0) * (!driftComp->counterSteeringInDrift ? sideForceParams->Decay_rate : 1.f), fminf(1, decaySideForceMag));
    float latSideForceFactor = fmaxf(angleForSideForceRatio - sideForceDecay, 0.f) * driftScaleInfluenceOnSideForce / sideForceParams->Drift_scale_for_maximum_side_force;
    latSideForceFactor = fminf(latSideForceFactor * sideForceSpeedRatio * nfsVehicle->m_grounddata.averageDriveGrip * -(1.f + driftComp->mvfCounterSteerSideMag.m128_f32[0]), 5.f);
    float scaledSideForce = latSideForceFactor * nfsVehicle->m_originalMass * sideForceMagnitude;
    driftComp->mvfSideForceMagnitude = _mm_set1_ps(scaledSideForce);

    __m128 avgGroundNormal = _mm_unpacklo_ps(
        _mm_unpacklo_ps(
            _mm_set1_ps( nfsVehicle->m_grounddata.avgGroundNormal.m128_f32[0]),
            _mm_set1_ps( nfsVehicle->m_grounddata.avgGroundNormal.m128_f32[2])),
        _mm_unpacklo_ps(
            _mm_set1_ps( nfsVehicle->m_grounddata.avgGroundNormal.m128_f32[1]),
            _mm_set1_ps( nfsVehicle->m_grounddata.avgGroundNormal.m128_f32[0])));

    __m128 localSideForceA = sideVector * (scaledSideForce * -sign(driftComp->mvfCurrentDriftAngle.m128_f32[0], 1));
    __m128 localSideForceB = localSideForceA * avgGroundNormal;
    __m128 localSideForceC = _mm_shuffle_ps(localSideForceB, localSideForceB, 2) + _mm_shuffle_ps(localSideForceB, localSideForceB, 1) + localSideForceB;
    __m128 localSideForceD = _mm_shuffle_ps(localSideForceC, localSideForceC, 0);
    __m128 sideForceToAdd = _mm_set1_ps(0.f) - (localSideForceA - (avgGroundNormal * localSideForceD) * nfsVehicle->m_grounddata.averageDriveGrip);

    /*float sideForceToAdd = sideForceMagnitude * counterSteerSideMagnitude;
    float sideForceGasLetOffMul = nfsVehicle->m_input->throttleInput.X <= 0.1 ? driftComp->mpParams->sideForceParams->Drift_scale_from_gas_stab : 1;
    
    for (int i = 0; i < 4; ++i)
    {
        force.m128_f32[i] = sideForceToAdd * ((angleForSideForceRatio * sideForceSpeedRatio * sideForceGasLetOffMul) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[i];
    };*/

    AddWorldCOMForceLogged(chassis, &sideForceToAdd, &timeStep);
}

void DriftSpeedRetention(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis)
{
    Matrix44 matrix;
    float absSpeedMps = fabsf(nfsVehicle->m_forwardSpeed);
    __m128 forwardVector = GetTransform(chassis, &matrix)->zAxis;
    __m128 gasInput = _mm_shuffle_ps({ nfsVehicle->m_input->throttleInput.X }, { nfsVehicle->m_input->throttleInput.X }, 0);
    __m128 brakeInput = _mm_shuffle_ps({ nfsVehicle->m_input->brakeInput.X }, { nfsVehicle->m_input->brakeInput.X }, 0);
    __m128 timeStep = _mm_shuffle_ps({ nfsVehicle->m_currentUpdateDt }, { nfsVehicle->m_currentUpdateDt }, 0);

    // Call original game's speed retention function, but use MUCH smaller scaling for Reduce_forward_speed_amount and Maintain_entry_speed_amount in the vehicle drift configs.
    MaintainDriftSpeed(driftComp, &gasInput, &brakeInput, &forwardVector, nfsVehicle->m_raceCarPhysicsObjectInterface, &timeStep);
}

void DriftTorque(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis, __m128 timeStep)
{
    Matrix44 matrix;
    __m128 upVector = GetTransform(chassis, &matrix)->yAxis;
    RaceVehicleDriftConfigData* driftParams = driftComp->mpParams->driftTriggerParams;

    float yawTorque = -driftParams->Initial_yaw_torque;
    float minYawTorque = driftParams->Minimum_yaw_torque;
    float midYawTorque = driftParams->Mid_yaw_torque;
    float defaultSteeringRemap = driftParams->Default_steering_remapping;
    float counterSteerRemap = driftParams->Counter_steering_remapping;
    float gasLetOffTorque = driftParams->Gas_let_off_yaw_torque;
    float slipAngleForZeroScale = driftParams->Slip_angle_for_zero_drift_scale;
    float slipAngleForBlendDown = driftParams->Slip_angle_to_start_blending_down_drift_scale;

    float absSteeringInput = fabsf(nfsVehicle->m_lastInputSteering);
    float steeringRemaining = 1.f - absSteeringInput;
    bool counterSteering = nfsVehicle->m_lastInputSteering * driftComp->mvfDriftScale.m128_f32[0] < 0;
    float absDriftScale = fabsf(driftComp->mvfDriftScale.m128_f32[0]);
    float absSlipAngle = fabsf(driftComp->mvfRearSlipAngle.m128_f32[0]);
    float blendMinMidYaw = midYawTorque - minYawTorque * absDriftScale + minYawTorque;
    float calcYawA = driftParams->Maximum_yaw_torque - midYawTorque * absDriftScale + midYawTorque - blendMinMidYaw * absDriftScale + blendMinMidYaw;

    if (absDriftScale > Epsilon)
    {
        float steeringRemap = (steeringRemaining * counterSteerRemap) + (absSteeringInput * defaultSteeringRemap);
        float calcYawB = 0.f - (sign(driftComp->mvfDriftScale.m128_f32[0], 1) * (gasLetOffTorque * driftComp->mvfPreviousGasInput.m128_f32[0] + steeringRemap)) * calcYawA;
        float driftScaleBlendDownRange = slipAngleForBlendDown - slipAngleForZeroScale;
        if (driftScaleBlendDownRange <= Epsilon) driftScaleBlendDownRange = Epsilon;
        float calcYawC = !counterSteering * nfsVehicle->m_grounddata.averageDriveGrip * calcYawB * driftComp->mvfTireGrip.m128_f32[0];
        float finalYaw = GetModifiedValue(nfsVehicle->m_performanceModificationComponent, ATM_DriftYawTorque, calcYawC);
        driftComp->currentYawTorque = finalYaw;
        //float torque = yawTorque * bDrift->driftScale;
        __m128 yawTorqueToAdd = upVector * finalYaw;
        AddWorldTorqueLogged(chassis, &yawTorqueToAdd, &timeStep);
    }
}

void ApplyDriftDamping(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis, __m128 timeStep)
{
    float slipToEnterDrift = driftComp->mpParams->driftTriggerParams->Slip_angle_to_enter_drift;
    float slipForDeepDrift = driftComp->mpParams->driftTriggerParams->Slip_angle_for_deep_drift;
    float slipForFullDamping = driftComp->mpParams->driftTriggerParams->Slip_angle_for_full_damping_fade;
    float timeToBlendDamping = driftComp->mpParams->driftTriggerParams->Time_to_blend_damping;
    float gasInput = nfsVehicle->m_input->throttleInput.X;
    //PointGraph8 dampingAtAngle = initPointGraph8(*driftComp->mpParams->driftTriggerParams->YawDampeningAtAngle);
    //PointGraph8 dampingAtSpeed = initPointGraph8(*driftComp->mpParams->driftTriggerParams->YawDampeningAtSpeed);
    float avgTireGrip = driftComp->mvfTireGrip.m128_f32[0];

    float dampingAng = EvaluatePointGraph8(&nfsVehicle->m_angularDampeningAtAngle, fabsf(driftComp->mvfCurrentDriftAngle.m128_f32[0]));
    Debug("YawDampeningAtAngle: %f\n", dampingAng);
    float dampingSpeed = EvaluatePointGraph8(&nfsVehicle->m_angularDampeningAtSpeed, fabsf(GetSpeedMph(nfsVehicle)));
    Debug("YawDampeningAtSpeed: %f\n", dampingSpeed);

    float avgSlipAng = GetAvgRearSlip(driftComp);
    float dampingBlendTime = 1 - (driftComp->mvfTimeSinceExittingDrift.m128_f32[0] / timeToBlendDamping);
    float v32 = fminf(1, fmaxf(0, (fabsf(avgSlipAng) / 1))) * gasInput;
    __m128 v36 = _mm_cmplt_ps({ 0,0,0,0 }, { slipForFullDamping,slipForFullDamping,slipForFullDamping,slipForFullDamping });

    /*float speedRatio = map(fabsf(GetSpeedMph(nfsVehicle)), 30, 95, 0, 1);
    float dampingScale = map(fabsf(avgSlipAng), slipToEnterDrift, slipForDeepDrift, 0, 1) * speedRatio;
    __m128 yawDamping = { dampingScale * angDamping};*/

    __m128 dampingScale = _mm_min_ps({ 1,1,1,1 }, _mm_max_ps({ 0,0,0,0 }, _mm_sub_ps({ dampingBlendTime,dampingBlendTime,dampingBlendTime,dampingBlendTime },
        _mm_or_ps(_mm_andnot_ps(v36, { 0,0,0,0 }), _mm_and_ps({ v32,v32,v32,v32 }, v36)))));

    dampingScale =
        _mm_mul_ps(
            _mm_mul_ps(dampingScale,
                _mm_shuffle_ps({ dampingAng }, { dampingAng }, 0)),
            _mm_shuffle_ps({ dampingSpeed }, { dampingSpeed }, 0)
        );
    __m128 surfaceGripFactor = _mm_shuffle_ps({ nfsVehicle->m_grounddata.averageDriveGrip }, { nfsVehicle->m_grounddata.averageDriveGrip }, 0);
    ApplyDamping(driftComp, &surfaceGripFactor, &dampingScale, &timeStep);
}

void UpdateDriftExit(NFSVehicle* const nfsVehicle, DriftComponent* driftComponent)
{
    __m128 timeStep = _mm_shuffle_ps({ nfsVehicle->m_currentUpdateDt }, { nfsVehicle->m_currentUpdateDt }, 0);

    if (driftComponent->mvfTimeSinceExittingDrift.m128_f32[1] > 0)
    {
        Debug("Exiting drift!\n");
        driftComponent->mvfDriftYawDamping.m128_f32[2] = driftComponent->mpParams->driftTriggerParams->Drift_sideways_damping * 1.5f;

        if (driftComponent->mvfDriftScale.m128_f32[0] > 0 || driftComponent->mvfDriftScale.m128_f32[0] < 0)
        {
            driftComponent->mvfDriftScale.m128_f32[0] += driftComponent->mpParams->driftTriggerParams->Drift_scale_decay * -sign(driftComponent->mvfDriftScale.m128_f32[0], 1) * nfsVehicle->m_currentUpdateDt;
            driftComponent->mvfDriftScale.m128_f32[0] = std::clamp(driftComponent->mvfDriftScale.m128_f32[0], -1.f, 1.f);
        }
        else driftComponent->mvfDriftScale.m128_f32[0] = 0;

        driftComponent->mvfTimeSinceExittingDrift.m128_f32[1] -= timeStep.m128_f32[0];
        driftComponent->mvfDriftYawDamping.m128_f32[1] =
            map(driftComponent->mvfTimeSinceExittingDrift.m128_f32[1], 0, driftComponent->mpParams->driftTriggerParams->Starting_drift_scale_after_scandinavian_flick, 0, driftComponent->mpParams->driftTriggerParams->Steering_amount_on_exit_drift);

        __m128 driftExitDamping = _mm_shuffle_ps({ driftComponent->mvfDriftYawDamping.m128_f32[1] }, { driftComponent->mvfDriftYawDamping.m128_f32[1] }, 0);
        //nfsVehicle->m_raceCarInputState.inputSteering =
        //    bDriftComp->RemapSteeringForDrift(driftComponent, nfsVehicle->m_input->steeringInput.X, driftComponent->mvfRearSlipAngle.m128_f32[0], nfsVehicle->m_raceCar->mVehicleTuning.maxSteeringAngle, nfsVehicle->m_steeringInterface);
        __m128 surfaceGripFactor = _mm_shuffle_ps({ nfsVehicle->m_grounddata.averageDriveGrip }, { nfsVehicle->m_grounddata.averageDriveGrip }, 0);
        ApplyDamping(driftComponent, &surfaceGripFactor, &driftExitDamping, &timeStep);
    }
    else if (driftComponent->mvfTimeSinceExittingDrift.m128_f32[1] <= 0)
    {
        driftComponent->isDrifting = false;
        driftComponent->isExitingDrift = false;
        driftComponent->mvfDriftYawDamping.m128_f32[2] = driftComponent->mpParams->driftTriggerParams->Drift_sideways_damping;
        driftComponent->currentYawTorque = 0;
    }
}

void UpdateDrift(NFSVehicle* const nfsVehicle, DriftComponent* driftComponent, RaceRigidBody* const rigidBody)
{
    if (driftComponent->isDrifting && !driftComponent->isExitingDrift)
    {
        __m128 timeStep = _mm_shuffle_ps({ nfsVehicle->m_currentUpdateDt }, { nfsVehicle->m_currentUpdateDt }, 0);
        float slipAngleDegrees = driftComponent->mvfRearSlipAngle.m128_f32[0];
        __m128 avgSlipAng = _mm_shuffle_ps({ slipAngleDegrees }, { slipAngleDegrees }, 0);
        float speedMph = GetSpeedMph(nfsVehicle);
        driftComponent->mvfDriftYawDamping.m128_f32[2] = driftComponent->mpParams->driftTriggerParams->Drift_sideways_damping;

        driftComponent->mvfTimeDrifting = { 0,0,0,0 };
        //bDriftComp->driftExitDampingFactor = driftComponent->mpParams->driftTriggerParams->Starting_drift_scale_after_scandinavian_flick;
        driftComponent->mvfPropSpeedMaintainAlongZ =
            _mm_shuffle_ps(
                { driftComponent->mpParams->otherParams->Reduce_forward_speed_amount },
                { driftComponent->mpParams->otherParams->Reduce_forward_speed_amount },
                0);
        driftComponent->mvfPropSpeedMaintainAlongVel =
            _mm_shuffle_ps(
                { driftComponent->mpParams->otherParams->Maintain_entry_speed_amount },
                { driftComponent->mpParams->otherParams->Maintain_entry_speed_amount },
                0);

        /*driftComponent->mvfSideForceMagnitude = { 0,0,0,0 };
        driftComponent->counterSteeringSideMagnitude = { 0,0,0,0 };*/
        driftComponent->mvfDriftYawDamping.m128_f32[0] = 0;
        driftComponent->mvfDriftYawDamping.m128_f32[1] = 0;
        driftComponent->mvfTimeDrifting.m128_f32[0] += timeStep.m128_f32[0];
        driftComponent->mvfTimeSteerLeft.m128_f32[0] = nfsVehicle->m_input->steeringInput.X > 0 ? driftComponent->mvfTimeSteerLeft.m128_f32[0] + timeStep.m128_f32[0] : 0;
        driftComponent->mvfTimeSteerRight.m128_f32[0] = nfsVehicle->m_input->steeringInput.X < 0 ? driftComponent->mvfTimeSteerRight.m128_f32[0] + timeStep.m128_f32[0] : 0;
        driftComponent->mvfMaxSlipAngle.m128_f32[0] = fmaxf(driftComponent->mvfRearSlipAngle.m128_f32[0], fmaxf(driftComponent->mvfRearSlipAngle.m128_f32[0] * -1.f, driftComponent->mvfRearSlipAngle.m128_f32[0]));

        //UpdateDriftAngle(rigidBody, driftComponent);
        UpdateDriftAngle(nfsVehicle);
        UpdateDriftScale(nfsVehicle, driftComponent);
        if (fabsf(driftComponent->mvfCurrentDriftAngle.m128_f32[0]) < driftComponent->mvfCurrentDriftAngle.m128_f32[1]
            && fabsf(driftComponent->mvfCurrentDriftAngle.m128_f32[0]) <= driftComponent->mpParams->driftScaleParams->Drift_angle_to_exit_drift
            || nfsVehicle->m_vehicleState == NFSVehicleState_Collided)
        {
            driftComponent->isExitingDrift = true;
            driftComponent->mvfTimeSinceExittingDrift.m128_f32[1] = driftComponent->mpParams->driftScaleParams->Starting_drift_scale_after_scandinavian_flick;
            return;
        }
        //nfsVehicle->m_raceCarInputState.inputSteering =
        //    bDriftComp->RemapSteeringForDrift(driftComponent, nfsVehicle->m_input->steeringInput.X, slipAngleDegrees, nfsVehicle->m_raceCar->mVehicleTuning.maxSteeringAngle, nfsVehicle->m_steeringInterface);
        DriftSideForce(nfsVehicle, driftComponent, rigidBody);
        DriftSpeedRetention(nfsVehicle, driftComponent, rigidBody);
        DriftTorque(nfsVehicle, driftComponent, rigidBody, timeStep);
        ApplyDriftDamping(nfsVehicle, driftComponent, rigidBody, timeStep);

        driftComponent->mvfTimeSinceExittingDrift.m128_f32[0] = 0;
    }
    else if (driftComponent->isExitingDrift)
        UpdateDriftExit(nfsVehicle, driftComponent);
}

void UpdatePostDriftExit(NFSVehicle* const nfsVehicle, DriftComponent* driftComponent)
{
    if (!driftComponent->isDrifting)
    {
        __m128 timeStep = _mm_shuffle_ps({ nfsVehicle->m_currentUpdateDt }, { nfsVehicle->m_currentUpdateDt }, 0);
        driftComponent->mvfTimeDrifting.m128_f32[0] = 0;
        driftComponent->mvfTimeSinceExittingDrift.m128_f32[0] += timeStep.m128_f32[0];
        driftComponent->mvfTimeSinceExittingDrift.m128_f32[1] = 0;
        driftComponent->mvfTimeSteerLeft.m128_f32[0] = 0;
        driftComponent->mvfTimeSteerRight.m128_f32[0] = 0;
        driftComponent->mvfDriftYawDamping.m128_f32[0] = 0;
        driftComponent->mvfDriftYawDamping.m128_f32[1] = 0;
        driftComponent->mvfCurrentDriftAngle = { 0,0,0,0 };
        driftComponent->mvfCurrentDriftAngle.m128_f32[0] = 0;
        driftComponent->mvfCurrentDriftAngle.m128_f32[1] = 0;
        driftComponent->mvfDriftScale.m128_f32[0] = 0;
        driftComponent->mvfMaintainedSpeed = { 0,0,0,0 };
        driftComponent->mvfDriftScale = { 0,0,0,0 };
        driftComponent->mvfCounterSteerSideMag = { 0,0,0,0 };
        driftComponent->mvfSideForceMagnitude = { 0,0,0,0 };
        driftComponent->currentYawTorque = 0;
    }
}

void __fastcall UpdateDrift_Orig(DriftComponent* driftComponent, const __m128* lvfGasInput, const __m128* lvfBrakeInput, const __m128* lvfSteeringInput, const __m128* lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, const class RaceCarPhysicsObject* lpRaceCar, SteeringComponent& lpSteeringComponent, const class SteeringParams& lpSteeringParams, const __m128& lvfAverageSurfaceGripFactor, const __m128& lvfTimeStep, const __m128& lvfInvTimeStep)
{
    NFSVehicle& nfsVehicle = **(NFSVehicle**)lpRaceCar;
    driftComponent->mvfRearSlipAngle.m128_f32[0] = nfsVehicle.m_sideSlipAngle * 180.f * 0.31830987f;
    driftComponent->mvfMaxSlipAngle.m128_f32[0] = nfsVehicle.m_sideSlipAngle * 180.f * 0.31830987f;
    CheckForEnteringDrift(driftComponent->mpChassisRigidBody->nfsVehicle, driftComponent);
    UpdateDrift(driftComponent->mpChassisRigidBody->nfsVehicle, driftComponent, driftComponent->mpChassisRigidBody);
    UpdatePostDriftExit(driftComponent->mpChassisRigidBody->nfsVehicle, driftComponent);
}

float __fastcall RemapSteeringForDrift_Orig(DriftComponent* driftComp, float steeringInput, float slipAngleDegrees, float maxSteeringAngle, SteeringComponent& lpSteeringComponent)
{
    // stop the game from normally calculating this because it'll lock up the steering with the new drift code
    return steeringInput;
}

#endif // REVIVAL_FUNCTIONS_REGION 

DWORD WINAPI Start(LPVOID lpParam)
{
    Sleep(1000);
    FILE* pFile = nullptr;

    // Initialize console in order to output useful debug data
    #ifdef _DEBUG
    AllocConsole();
    freopen_s(&pFile, "CONIN$", "r", stdin);
    freopen_s(&pFile, "CONOUT$", "w", stdout);
    freopen_s(&pFile, "CONOUT$", "w", stderr);
    SetWindowTextW(GetConsoleWindow(), L"Need for Speed� Revival");
    #endif

    HWND gameWindow = NULL;
    uintptr_t game = (uintptr_t)GetModuleHandle(NULL);
    //NFSVehicle* nfsVehicle = NULL;
    //DriftComponent* driftComponent = NULL;
    //RaceRigidBody* rigidBody = NULL;
    //uintptr_t nfsVehicleBase = game + 0x02C431A0;
    uintptr_t gameContext = game + 0x0289CDC0;
    Debug("gameContext: %I64X\n", gameContext);
    InjectHook(0x144197250, UpdateDrift_Orig);
    InjectHook(0x1441967A0, RemapSteeringForDrift_Orig);
    return 0;
}

BOOL WINAPI DllMain(HINSTANCE hinstDLL, DWORD fdwReason, LPVOID lpvReserved)
{
    switch (fdwReason)
    {
    case DLL_PROCESS_ATTACH:
        // attach to process
        // return FALSE to fail DLL load
        CreateThread(0, 0, (LPTHREAD_START_ROUTINE)Start, 0, 0, 0);
        break;

    case DLL_PROCESS_DETACH:
        // detach from process
        break;

    case DLL_THREAD_ATTACH:
        // attach to thread
        break;

    case DLL_THREAD_DETACH:
        // detach from thread
        break;
    }
    return TRUE; // successful
}