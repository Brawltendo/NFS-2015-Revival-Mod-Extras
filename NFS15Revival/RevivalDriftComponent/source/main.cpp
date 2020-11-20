#include "pch.h"
#include "psapi.h"
#include "main.h"
#include "util\utils.h"
#include "DriftComponent\RevivalDriftComponent.h"

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
_UpdateDriftScale UpdateDriftScale = (_UpdateDriftScale)0x144197840;
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

    __m128 mulByUpVec = _mm_mul_ps(upVector, angVel);
    __m128 localAngVel = _mm_add_ps(
        _mm_add_ps(
            _mm_shuffle_ps(mulByUpVec, mulByUpVec, 2),
            _mm_shuffle_ps(mulByUpVec, mulByUpVec, 1)),
        mulByUpVec);
    __m128 localAngVelFinal = _mm_shuffle_ps(localAngVel, localAngVel, 0);
    return RadiansToDegrees(localAngVelFinal.m128_f32[0]);
}

void UpdateDriftAngle(RaceRigidBody* chassis, DriftComponent* driftComp, RevivalDriftComponent* bDrift)
{
    UpdateDriftAngleDegrees(driftComp);
    bDrift->driftAngle = sign(GetLocalAngVelDegrees(chassis), 1) * driftComp->driftAngle.m128_f32[0];
    // Debug("driftAngle: %f\n", bDrift->driftAngle);
}

void DriftSideForce(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RevivalDriftComponent* bDrift, RaceRigidBody* chassis)
{
    Matrix44 matrix;
    __m128 sideVector = GetTransform(chassis, &matrix)->xAxis;
    __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);
    float sideForceMagnitude = driftComp->driftParams->sideForceParams->Side_force_magnitude;
    UpdateCounterSteeringSideMagnitude(nfsVehicle, driftComp, bDrift, nfsVehicle->vehicleInput->steeringInput.X, GetLocalAngVelDegrees(chassis), timeStep.m128_f32[0]);
    float counterSteerSideMagnitude = 1 + bDrift->counterSteeringSideMagnitude;

    Debug("driftAngle: %f\n", bDrift->driftAngle);

    float driftAngRange = map(fabsf(bDrift->driftAngle), driftComp->driftParams->sideForceParams->Minimum_drift_angle_for_side_force, driftComp->driftParams->sideForceParams->Drift_angle_for_side_force, 0.2f, 1);
    float sideForceSpeedRatio = map(GetSpeedMph(nfsVehicle), driftComp->driftParams->sideForceParams->Speed_for_no_sideforce, driftComp->driftParams->sideForceParams->Speed_for_maximum_side_force, 0, 1);
    float sideForceToAdd = sideForceMagnitude * counterSteerSideMagnitude;
    float sideForceGasLetOffMul = nfsVehicle->vehicleInput->throttleInput.X <= 0.1 ? driftComp->driftParams->sideForceParams->Drift_scale_from_gas_stab : 1;

    __m128 force = {
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * sideForceGasLetOffMul) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[0],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * sideForceGasLetOffMul) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[1],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * sideForceGasLetOffMul) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[2],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * sideForceGasLetOffMul) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[3]
    };
    AddWorldCOMForceLogged(chassis, &force, &timeStep);
}

void DriftSpeedRetention(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis)
{
    Matrix44 matrix;
    float absSpeedMps = fabsf(nfsVehicle->speedMps);
    __m128 forwardVector = GetTransform(chassis, &matrix)->zAxis;
    __m128 gasInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->throttleInput.X }, { nfsVehicle->vehicleInput->throttleInput.X }, 0);
    __m128 brakeInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->brakeInput.X }, { nfsVehicle->vehicleInput->brakeInput.X }, 0);
    __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);

    // Call original game's speed retention function, but use MUCH smaller scaling for Reduce_forward_speed_amount and Maintain_entry_speed_amount in the vehicle drift configs.
    MaintainDriftSpeed(driftComp, &gasInput, &brakeInput, &forwardVector, nfsVehicle->raceCarPhysObj, &timeStep);
}

void SteerSameDirectionTorque(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RevivalDriftComponent* bDrift, RaceRigidBody* chassis, __m128 timeStep)
{
    Matrix44 matrix;
    __m128 upVector = GetTransform(chassis, &matrix)->yAxis;
    RaceVehicleDriftConfigData* driftParams = driftComp->driftParams->driftTriggerParams;
    float yawTorque = -driftParams->Initial_yaw_torque;

    __m128 steeringYawScaleVsTime[] =
    {
        {0,driftParams->Starting_drift_scale,0,0},
        {driftParams->Brake_stab_return_speed,driftParams->Drift_scale_from_steering,0,0},
        {0,0,0,0},
        {0.14f,0.1f,0,0},
        {0.223f,0.15f,0,0},
        {0.338f,0.28f,0,0},
        {0.436f,0.45f,0,0},
        {0.542f,0.675f,0,0},
        {0.72f,0.88f,0,0},
        {1,1,0,0}
    };

    PointGraph8 pgSteeringYawScaleVsTime = initPointGraph8(steeringYawScaleVsTime);

    float sameDirMul =
        driftComp->timeSteerLeft.m128_f32[0] > 0 ? EvaluatePointGraph8(&pgSteeringYawScaleVsTime, driftComp->timeSteerLeft.m128_f32[0]) :
        driftComp->timeSteerRight.m128_f32[0] > 0 ? EvaluatePointGraph8(&pgSteeringYawScaleVsTime, driftComp->timeSteerRight.m128_f32[0]) : 0;

    // Lowers the amount of torque applied when countersteering to stop the shimmy thing that happens with vanilla DriftComponent's yaw torque.
    float countersteerTorqueMul = driftComp->counterSteeringInDrift ? driftParams->Drift_scale_from_counter_steering : 1;
    float gasLetOffTorqueMul = nfsVehicle->vehicleInput->throttleInput.X < 0.1f ? driftParams->Drift_scale_from_gas_let_off : 1;

    float torque = yawTorque * sameDirMul * countersteerTorqueMul * gasLetOffTorqueMul * nfsVehicle->vehicleInput->steeringInput.X;

    __m128 finalTorque = _mm_mul_ps(upVector, _mm_shuffle_ps({ torque }, { torque }, 0));
    AddWorldTorqueLogged(chassis, &finalTorque, &timeStep);
}

void ApplyDriftDamping(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RevivalDriftComponent* bDrift, RaceRigidBody* chassis, __m128 timeStep)
{
    PointGraph8 pgIn;
    float slipToEnterDrift = driftComp->driftParams->driftTriggerParams->Slip_angle_to_enter_drift;
    float slipForDeepDrift = driftComp->driftParams->driftTriggerParams->Slip_angle_for_deep_drift;
    float slipForFullDamping = driftComp->driftParams->driftTriggerParams->Slip_angle_for_full_damping_fade;
    float timeToBlendDamping = driftComp->driftParams->driftTriggerParams->Time_to_blend_damping;
    float lvfGasInput = nfsVehicle->vehicleInput->throttleInput.X;
    PointGraph8 dampingAtAngle = initPointGraph8(*driftComp->driftParams->driftTriggerParams->YawDampeningAtAngle);
    PointGraph8 dampingAtSpeed = initPointGraph8(*driftComp->driftParams->driftTriggerParams->YawDampeningAtSpeed);
    float avgTireGrip = driftComp->avgTireGrip.m128_f32[0];

    float dampingAng = EvaluatePointGraph8(&dampingAtAngle, fabsf(bDrift->driftAngle));
    Debug("YawDampeningAtAngle: %f\n", dampingAng);
    float dampingSpeed = EvaluatePointGraph8(&dampingAtSpeed, fabsf(GetSpeedMph(nfsVehicle)));
    Debug("YawDampeningAtSpeed: %f\n", dampingSpeed);

    float avgSlipAng = GetAvgRearSlip(driftComp);
    float dampingBlendTime = 1 - (bDrift->timeSinceLastDrift / timeToBlendDamping);
    float v32 = fminf(1, fmaxf(0, (fabsf(avgSlipAng) / 1))) * lvfGasInput;
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
    ApplyDamping(driftComp, &_mm_shuffle_ps({ 1 }, { 1 }, 0), &dampingScale, &timeStep);
}

void UpdateDrifting(NFSVehicle* const nfsVehicle, DriftComponent* driftComponent, RevivalDriftComponent* bDriftComp, RaceRigidBody* const rigidBody)
{
    if (CheckForEnteringDrift(nfsVehicle, driftComponent))
    {
        __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);
        float slipAngleDegrees = (nfsVehicle->outputState.tireSlipAngle[3] + nfsVehicle->outputState.tireSlipAngle[2]) * 0.5f * 0.15915507f * 360.f;
        __m128 avgSlipAng = _mm_shuffle_ps({ slipAngleDegrees }, { slipAngleDegrees }, 0);
        float speedMph = GetSpeedMph(nfsVehicle);

        driftComponent->timeSpentInDrift = { 0,0,0,0 };
        bDriftComp->chainedDriftDampingFactor = driftComponent->driftParams->driftTriggerParams->Starting_drift_scale_after_scandinavian_flick;
        driftComponent->reduceForwardSpeedAmount =
            _mm_shuffle_ps(
                { driftComponent->driftParams->otherParams->Reduce_forward_speed_amount },
                { driftComponent->driftParams->otherParams->Reduce_forward_speed_amount },
                0);
        driftComponent->maintainEntrySpeedAmount =
            _mm_shuffle_ps(
                { driftComponent->driftParams->otherParams->Maintain_entry_speed_amount },
                { driftComponent->driftParams->otherParams->Maintain_entry_speed_amount },
                0);

        /*driftComponent->currentSideForce = { 0,0,0,0 };
        driftComponent->counterSteeringSideMagnitude = { 0,0,0,0 };*/
        driftComponent->angularDamping = { 0,0,0,0 };
        /*__m128 timeInDrift = _mm_add_ps(driftComponent->timeSpentInDrift, timeStep);
        driftComponent->timeSpentInDrift = timeInDrift;*/
        bDriftComp->timeSteeringLeft = nfsVehicle->vehicleInput->steeringInput.X > 0 ? bDriftComp->timeSteeringLeft + timeStep.m128_f32[0] : 0;
        bDriftComp->timeSteeringRight = nfsVehicle->vehicleInput->steeringInput.X < 0 ? bDriftComp->timeSteeringRight + timeStep.m128_f32[0] : 0;
        bDriftComp->peakSlipAngleReached = fmaxf(bDriftComp->peakSlipAngleReached, fmaxf(driftComponent->avgSlipAngle.m128_f32[0] * -1.f, driftComponent->avgSlipAngle.m128_f32[0]));

        UpdateDriftAngle(rigidBody, driftComponent, bDriftComp);
        SetDriftScale(driftComponent, bDriftComp);
        nfsVehicle->inputState.inputSteering =
            RemapSteeringForDrift(driftComponent, bDriftComp, nfsVehicle->vehicleInput->steeringInput.X, slipAngleDegrees, nfsVehicle->raceCar->mVehicleTuning.maxSteeringAngle, nfsVehicle->steeringComponent);
        DriftSideForce(nfsVehicle, driftComponent, bDriftComp, rigidBody);
        DriftSpeedRetention(nfsVehicle, driftComponent, rigidBody);
        SteerSameDirectionTorque(nfsVehicle, driftComponent, bDriftComp, rigidBody, timeStep);
        ApplyDriftDamping(nfsVehicle, driftComponent, bDriftComp, rigidBody, timeStep);

        //bDriftComp.timeSinceLastDrift = 0;
        //driftComponent->timeSinceLastDrift = _mm_shuffle_ps({ bDriftComp.timeSinceLastDrift }, { bDriftComp.timeSinceLastDrift }, 0);
    }
    else if (!CheckForEnteringDrift(nfsVehicle, driftComponent))
    {
        __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);
        /*bDriftComp.timeSinceLastDrift += timeStep.m128_f32[0];
        driftComponent->timeSinceLastDrift = _mm_shuffle_ps({ bDriftComp.timeSinceLastDrift }, { bDriftComp.timeSinceLastDrift }, 0);
        bDriftComp.timeSteeringLeft = 0;
        bDriftComp.timeSteeringRight = 0;*/
        bDriftComp->yawDamping = { 0,0,0,0 };
        bDriftComp->angDamping = { 0,0,0,0 };

        //GetLocalAngVelDegrees(rigidBody);
        driftComponent->driftAngle = { 0,0,0,0 };
        bDriftComp->driftAngle = 0;
        bDriftComp->driftScale = 0;
        /*while (bDriftComp.timeSinceLastDrift <= bDriftComp.timeApplyingChainDriftDamping && bDriftComp.chainedDriftDampingFactor != 0)
        {
            bDriftComp.chainedDriftDampingFactor -= timeStep.m128_f32[0];
            if (bDriftComp.chainedDriftDampingFactor < 0)
                bDriftComp.chainedDriftDampingFactor = 0;
            __m128 chainedDriftDamping = _mm_shuffle_ps({ bDriftComp.chainedDriftDampingFactor }, { bDriftComp.chainedDriftDampingFactor }, 0);
            ApplyDamping(driftComponent, &_mm_shuffle_ps({ 1 }, { 1 }, 0), &chainedDriftDamping, &timeStep);
        }*/
    }
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
    SetWindowTextW(GetConsoleWindow(), L"Need for Speed™ Revival");
    #endif
    
    const int TICKS_PER_SECOND = 30;
    const int SKIP_TICKS = 1000 / TICKS_PER_SECOND;
    const int MAX_FRAMESKIP = 5;
    DWORD nextTick = GetTickCount();
    int loops;
    float interpolation;

    uintptr_t game = (uintptr_t)GetModuleHandle(NULL);
    NFSVehicle* nfsVehicle = NULL;
    DriftComponent* driftComponent = NULL;
    uintptr_t nfsVehicleBase = game + 0x02C431A0;
    uintptr_t gameContext = game + 0x0289CDC0;
    Debug("gameContext: %I64X\n", gameContext);
    RevivalDriftComponent bDriftComp{};

    /*
    Debug("AddWorldCOMForceLogged: %I64X\n", AddWorldCOMForceLogged);
    Debug("DampPitchYawRoll: %I64X\n", DampPitchYawRoll);
    Debug("GetTransform: %I64X\n", GetTransform);
    Debug("GetAngularVelocity: %I64X\n", GetAngularVelocity);
    Debug("MaintainDriftSpeed: %I64X\n", MaintainDriftSpeed);
    Debug("UpdateDriftAngleDegrees: %I64X\n", UpdateDriftAngleDegrees);
    Debug("ApplyDamping: %I64X\n", ApplyDamping);
    */

    while (1)
    {
        //SetWindowTextW(GetForegroundWindow(), L"Need for Speed™ Revival");
        nfsVehicle = (NFSVehicle*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x0 });

        // Check to see whether or not the NFSVehicle pointer is valid
        float nfsVehicleChecker;
        float nfsVehicleChecker1;
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)(uintptr_t)nfsVehicle + 0x64, &nfsVehicleChecker, sizeof(nfsVehicleChecker), nullptr);
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)(uintptr_t)nfsVehicle + 0x68, &nfsVehicleChecker1, sizeof(nfsVehicleChecker1), nullptr);

        driftComponent = (DriftComponent*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x118, 0xE80, 0x0 }); //nfsVehicle + 0x11A8; nfsVehicle->driftComponent;

        loops = 0;
        while (IsGameTicking(nextTick, loops, MAX_FRAMESKIP))
        {
            //Debug("Ticking\n");
            if (nfsVehicleChecker == 150 && nfsVehicleChecker1 == 10)
            {
                float angVelDeg{};
                RaceRigidBody* rigidBody = (RaceRigidBody*)FindDMAAddy(GetCurrentProcess(), (uintptr_t)driftComponent + 0x18, { 0x0 }); //nfsVehicle->raceRigidBody;
                UpdateDrifting(nfsVehicle, driftComponent, &bDriftComp, rigidBody);
                /*
                //Uncomment block to allow adding force upwards by pressing numpad 8, useful for exploring out of boundsand making other players say wtf
                if (GetAsyncKeyState(VK_NUMPAD8) && GetConsoleWindow() == GetForegroundWindow())
                {
                    Debug("nfsVehicleAddress: %I64X\n", nfsVehicle);
                    float dT = GetDeltaTime((uintptr_t)nfsVehicle);
                    Debug("deltaTime: %f\n", dT);
                    Debug("deltaTimeAddress: %I64X\n", &dT);
                    Debug("driftComponentAddress: %I64X\n", driftComponent);
                    __m128 lvfTimeStep = { dT,dT,dT,dT };
                    __m128 force = { 0, 300.f, 0, 0 };
                    Debug("rigidBodyAddress: %I64X\n", rigidBody);
                    AddWorldCOMForceLogged(rigidBody, &force, &lvfTimeStep);
                    Debug("Adding force!\n");
                }
                */
            }
            nextTick += SKIP_TICKS;
            loops++;
        }
        //Debug("Not ticking\n");
    }
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