#include "pch.h"
#include "main.h"
#include "DriftComponent\RevivalDriftComponent.h"

#ifndef NFS_NATIVE_FUNCTIONS
typedef PointGraph8* (__fastcall* _initPointGraph8FromCurveData)(PointGraph8* pointGraphIn, __m128 (*curveData)[10]);
typedef float(__fastcall* _PointGraph8__Evaluate)(int pgCount, float(*pgInX)[8], float(*pgInY)[8], float xVal);
typedef void(__fastcall* _AddWorldCOMForceLogged)(RaceRigidBody* rigidBody, __m128* force, __m128* lfTimeStep);
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
typedef float(__fastcall* _calcSteeringFromPitchAndYaw)(const NFSVehicle* const nfsVehicle, float pitch, float yaw);

_initPointGraph8FromCurveData initPointGraph8FromCurveData = (_initPointGraph8FromCurveData)0x1441B7BF0;
_PointGraph8__Evaluate PointGraph8__Evaluate = (_PointGraph8__Evaluate)0x143F6A8E0;
_AddWorldCOMForceLogged AddWorldCOMForceLogged = (_AddWorldCOMForceLogged)0x144170EE0;
_DampPitchYawRoll DampPitchYawRoll = (_DampPitchYawRoll)0x1441712D0;
_GetTransform GetTransform = (_GetTransform)0x1441718C0;
_GetAngularVelocity GetAngularVelocity = (_GetAngularVelocity)0x1441717E0;
_MaintainDriftSpeed MaintainDriftSpeed = (_MaintainDriftSpeed)0x1441960E0;
_UpdateDriftScale UpdateDriftScale = (_UpdateDriftScale)0x144197840;
_UpdateDriftAngleDegrees UpdateDriftAngleDegrees = (_UpdateDriftAngleDegrees)0x144197690;
_UpdateSideForce UpdateSideForce = (_UpdateSideForce)0x144197FC0;
_ApplyDriftForces ApplyDriftForces = (_ApplyDriftForces)0x144193890;
_ApplyDamping ApplyDamping = (_ApplyDamping)0x1441935E0;
_calcSteeringFromPitchAndYaw calcSteeringFromPitchAndYaw = (_calcSteeringFromPitchAndYaw)0x144173270;
#endif // NFS_NATIVE_FUNCTIONS

uintptr_t FindDMAAddy(HANDLE hProc, uintptr_t ptr, std::vector<unsigned int> offsets)
{
    uintptr_t addr = ptr;
    for (unsigned int i = 0; i < offsets.size(); ++i)
    {
        ReadProcessMemory(hProc, (BYTE*)addr, &addr, sizeof(addr), 0);
        addr += offsets[i];
    }
    return addr;
}


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
    return Radians2Degrees(localAngVelFinal.m128_f32[0]);
}

void GetDriftAngle(RaceRigidBody* chassis, BrawlDriftComponent* driftComp, float lastAngVel)
{
    float angVelDeg = GetLocalAngVelDegrees(chassis);

    float currentDriftAngle = (fabsf(angVelDeg) - fabsf(lastAngVel)) * sign(angVelDeg, 1);
    driftComp->driftAngle = currentDriftAngle;
}

void DriftSideForce(NFSVehicle* nfsVehicle, DriftComponent* driftComp, BrawlDriftComponent* bDrift, RaceRigidBody* chassis)
{
    Matrix44 matrix;
    __m128 sideVector = GetTransform(chassis, &matrix)->xAxis;
    __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);
    float sideForceMagnitude = driftComp->driftParams->sideForceParams->Side_force_magnitude;
    float sideForceMultiplier = driftComp->driftParams->sideForceParams->Side_force_multiplier;

    Debug("driftAngle: %f\n", bDrift->driftAngle);

    float driftAngRange = map(fabsf(bDrift->driftAngle), driftComp->driftParams->sideForceParams->Minimum_drift_angle_for_side_force, driftComp->driftParams->sideForceParams->Drift_angle_for_side_force, 0.2f, 1);
    float sideForceSpeedRatio = map(GetSpeedMph(nfsVehicle), driftComp->driftParams->sideForceParams->Speed_for_no_sideforce, driftComp->driftParams->sideForceParams->Speed_for_maximum_side_force, 0, 1);
    bool isCounterSteering = bDrift->driftAngle * nfsVehicle->vehicleInput->steeringInput.X < 0;
    driftComp->counterSteeringInDrift = isCounterSteering;
    float sideForceToAdd = isCounterSteering ? sideForceMagnitude * sideForceMultiplier : sideForceMagnitude;

    __m128 force = {
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * nfsVehicle->vehicleInput->throttleInput.X) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[0],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * nfsVehicle->vehicleInput->throttleInput.X) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[1],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * nfsVehicle->vehicleInput->throttleInput.X) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[2],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio * nfsVehicle->vehicleInput->throttleInput.X) * (sign(bDrift->driftAngle, 1) * -1)) * sideVector.m128_f32[3]
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

void ApplyDriftDamping(NFSVehicle* nfsVehicle, DriftComponent* driftComp, BrawlDriftComponent* bDrift, RaceRigidBody* chassis)
{
    PointGraph8 pgIn;
    __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);
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
    float v32 = min(1, max(0, (fabsf(avgSlipAng) / 1))) * lvfGasInput;
    __m128 v36 = _mm_cmplt_ps({ 0,0,0,0 }, { slipForFullDamping,slipForFullDamping,slipForFullDamping,slipForFullDamping });

    //float speedRatio = map(fabsf(GetSpeedMph(nfsVehicle)), 30, 95, 0, 1);
    //float dampingScale = map(fabsf(avgSlipAng), slipToEnterDrift, slipForDeepDrift, 0, 1) * speedRatio;
    //__m128 yawDamping = { dampingScale * angDamping};

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
#endif // REVIVAL_FUNCTIONS_REGION 

DWORD WINAPI Start(LPVOID lpParam)
{
    Sleep(1000);
    FILE* pFile = nullptr;

    // Initialize console in order to output useful debug data
    AllocConsole();
    freopen_s(&pFile, "CONIN$", "r", stdin);
    freopen_s(&pFile, "CONOUT$", "w", stdout);
    freopen_s(&pFile, "CONOUT$", "w", stderr);

    uintptr_t game = (uintptr_t)GetModuleHandle(NULL);
    NFSVehicle* nfsVehicle = NULL;
    DriftComponent* driftComponent = NULL;
    uintptr_t nfsVehicleBase = game + 0x02C431A0;
    Debug("gameMainAddress: %I64X\n", game);
    BrawlDriftComponent bDriftComp{};

    Debug("initPointGraph8FromCurveData: %I64X\n", initPointGraph8FromCurveData);
    Debug("PointGraph8::Evaluate: %I64X\n", PointGraph8__Evaluate);
    Debug("AddWorldCOMForceLogged: %I64X\n", AddWorldCOMForceLogged);
    Debug("DampPitchYawRoll: %I64X\n", DampPitchYawRoll);
    Debug("GetTransform: %I64X\n", GetTransform);
    Debug("GetAngularVelocity: %I64X\n", GetAngularVelocity);
    Debug("MaintainDriftSpeed: %I64X\n", MaintainDriftSpeed);
    Debug("UpdateDriftAngleDegrees: %I64X\n", UpdateDriftAngleDegrees);
    Debug("UpdateSideForce: %I64X\n", UpdateSideForce);
    Debug("ApplyDriftForces: %I64X\n", ApplyDriftForces);
    Debug("ApplyDamping: %I64X\n", ApplyDamping);
    Debug("calcSteeringFromPitchAndYaw: %I64X\n", calcSteeringFromPitchAndYaw);

    while (1)
    {
        nfsVehicle = (NFSVehicle*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x0 });
        
        /* 
        Uncomment block to allow adding force upwards by pressing numpad 8, useful for exploring out of bounds and making other players say WTF
        if (GetAsyncKeyState(VK_NUMPAD8))
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

        // Check to see whether or not the NFSVehicle pointer is valid
        float nfsVehicleChecker;
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)(uintptr_t)nfsVehicle + 0x64, &nfsVehicleChecker, sizeof(nfsVehicleChecker), nullptr);

        driftComponent = (DriftComponent*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x118, 0xE80, 0x0 }); //nfsVehicle + 0x11A8; nfsVehicle->driftComponent;

        if (nfsVehicleChecker == 150 && nfsVehicle->driftComponent != NULL)
        {
            float angVelDeg{};
            RaceRigidBody* rigidBody = (RaceRigidBody*)FindDMAAddy(GetCurrentProcess(), (uintptr_t)driftComponent + 0x18, { 0x0 }); //nfsVehicle->raceRigidBody;

            if (CheckForEnteringDrift(nfsVehicle, driftComponent))
            {
                __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime(nfsVehicle) }, { GetDeltaTime(nfsVehicle) }, 0);
                __m128 avgSlipAng = _mm_shuffle_ps({ GetAvgRearSlip(driftComponent) }, { GetAvgRearSlip(driftComponent) }, 0);
                float speedMph = GetSpeedMph(nfsVehicle);

                driftComponent->timeSpentInDrift = { 0,0,0,0 };
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
                driftComponent->driftAngle = { 0,0,0,0 };
                driftComponent->currentSideForce = { 0,0,0,0 };
                driftComponent->counterSteeringSideMagnitude = { 0,0,0,0 };
                driftComponent->angularDamping = { 0,0,0,0 };
                __m128 timeInDrift = _mm_add_ps(driftComponent->timeSpentInDrift, timeStep);
                driftComponent->timeSpentInDrift = timeInDrift;

                GetDriftAngle(rigidBody, &bDriftComp, angVelDeg);
                GetDriftScale(&bDriftComp);
                DriftSideForce(nfsVehicle, driftComponent, &bDriftComp, rigidBody);
                DriftSpeedRetention(nfsVehicle, driftComponent, rigidBody);
                ApplyDriftDamping(nfsVehicle, driftComponent, &bDriftComp, rigidBody);

                bDriftComp.timeSinceLastDrift = 0;
                driftComponent->timeSinceLastDrift = _mm_shuffle_ps({ bDriftComp.timeSinceLastDrift }, { bDriftComp.timeSinceLastDrift }, 0);
            }
            else if (!CheckForEnteringDrift(nfsVehicle, driftComponent))
            {
                bDriftComp.timeSinceLastDrift += GetDeltaTime(nfsVehicle);
                driftComponent->timeSinceLastDrift = _mm_shuffle_ps({ bDriftComp.timeSinceLastDrift }, { bDriftComp.timeSinceLastDrift }, 0);
                bDriftComp.yawDamping = { 0,0,0,0 };
                bDriftComp.angDamping = { 0,0,0,0 };

                GetLocalAngVelDegrees(rigidBody);
                bDriftComp.driftAngle = 0;
                bDriftComp.driftScale = 0;
            }
        }
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