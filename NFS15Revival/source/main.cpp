#include "pch.h"
#include "main.h"
#include "DriftComponent\RevivalDriftComponent.h"

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

bool DataCompare(const BYTE* OpCodes, const BYTE* Mask, const char* StrMask)
{
    while (*StrMask)
    {
        if (*StrMask == 'x' && *OpCodes != *Mask)
            return false;
        ++StrMask;
        ++OpCodes;
        ++Mask;
    }
    return true;
}

DWORD64 FindPattern(DWORD64 StartAddress, DWORD CodeLen, BYTE* Mask, const char* StrMask, unsigned short ignore)
{
    unsigned short Ign = 0;
    DWORD i = 0;
    while (Ign <= ignore)
    {
        if (DataCompare((BYTE*)(StartAddress + i++), Mask, StrMask))
            ++Ign;
        else if (i >= CodeLen)
            return 0;
    }
    return StartAddress + i - 1;
}

BrawlDriftComponent bDriftComp;

typedef void(__fastcall* _AddWorldCOMForceLogged)(RaceRigidBody* rigidBody, __m128* force, __m128* lfTimeStep);
typedef void(__fastcall* _DampPitchYawRoll)(RaceRigidBody* chassis, __m128* pitchDampening, __m128* yawDampening, __m128* rollDampening, __m128* lfTimeStep);
// Frostbite transformation matrix: Side vector = X, up vector = Y, forward vector = Z, location = W
typedef Matrix44* (__fastcall* _GetTransform)(RaceRigidBody* chassis, Matrix44* transformOut);
typedef __m128* (__fastcall* _GetAngularVelocity)(RaceRigidBody* const chassis, __m128* angVelOut);
typedef void(__fastcall* _MaintainDriftSpeed)(DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lDirection, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep);
typedef void(__fastcall* _UpdateDriftScale)(DriftComponent* driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lvfSteeringInputIn, HandbrakeComponent* lpHandbrake, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfTimeStep, __m128* lvfInvTimeStep);
typedef void(__fastcall* _UpdateDriftAngleDegrees)(DriftComponent* const driftComp);
typedef void(__fastcall* _UpdateSideForce)(DriftComponent* driftComp, __m128* lvfAbsDriftScale, class RaceCarPhysicsObject* lpRaceCar, __m128* lvfSteeringInput, __m128* lvfAverageSurfaceGripFactor, __m128* lvfTimeStep);
typedef void(__fastcall* _ApplyDamping)(DriftComponent* driftComp, __m128* lvfAverageSurfaceGripFactor, __m128* lvfDampingScale, __m128* lvfTimeStep);
typedef void(__fastcall* _ApplyDriftForces)(DriftComponent* const driftComp, __m128* lvfGasInput, __m128* lvfBrakeInput, __m128* lvfSteeringInput, __m128* lvfAbsDriftScale, HandbrakeComponent* lpHandbrake, class RaceCarPhysicsObject* lpRaceCar, float lfSpeedMPS, __m128* lvfAverageSurfaceGripFactor, __m128* lvfTimeStep, __m128* lvfInvTimeStep);
typedef float(__fastcall* _calcSteeringFromPitchAndYaw)(const NFSVehicle* const nfsVehicle, float pitch, float yaw);

_AddWorldCOMForceLogged AddWorldCOMForceLogged;
_DampPitchYawRoll DampPitchYawRoll;
_GetTransform GetTransform;
_GetAngularVelocity GetAngularVelocity;
_MaintainDriftSpeed MaintainDriftSpeed;
_UpdateDriftScale UpdateDriftScale;
_UpdateDriftAngleDegrees UpdateDriftAngleDegrees;
_UpdateSideForce UpdateSideForce;
_ApplyDamping ApplyDamping;
_ApplyDriftForces ApplyDriftForces;
_calcSteeringFromPitchAndYaw calcSteeringFromPitchAndYaw;

void ApplyDriftDamping(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis)
{
    float slipToEnterDrift = driftComp->driftParams->driftTriggerParams->Slip_angle_to_enter_drift;
    float slipForDeepDrift = driftComp->driftParams->driftTriggerParams->Slip_angle_for_deep_drift;
    float slipForFullDamping = driftComp->driftParams->driftTriggerParams->Slip_angle_for_full_damping_fade;
    float timeToBlendDamping = driftComp->driftParams->driftTriggerParams->Time_to_blend_damping;
    float lvfGasInput = nfsVehicle->vehicleInput->throttleInput.X;
    float angDamping = driftComp->driftParams->driftTriggerParams->Drift_angular_damping;
    __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime((uintptr_t)nfsVehicle) }, { GetDeltaTime((uintptr_t)nfsVehicle) }, 0);
    float avgTireGrip = driftComp->avgTireGrip.m128_f32[0];

    //ReadProcessMemory(GetCurrentProcess(), (BYTE*)((uintptr_t)driftComp + 0x210), &avgTireGrip, sizeof(avgTireGrip), nullptr);

    float avgSlipAng = GetAvgRearSlip(driftComp);
    float dampingBlendTime = 1 - (bDriftComp.timeSinceLastDrift / timeToBlendDamping);
    float v32 = min(1, max(0, (fabsf(avgSlipAng) / 1))) * lvfGasInput;
    __m128 v36 = _mm_cmplt_ps({ 0,0,0,0 }, { slipForFullDamping,slipForFullDamping,slipForFullDamping,slipForFullDamping });

    //float speedRatio = map(fabsf(GetSpeedMph(nfsVehicle)), 30, 95, 0, 1);
    //float dampingScale = map(fabsf(avgSlipAng), slipToEnterDrift, slipForDeepDrift, 0, 1) * speedRatio;
    //__m128 yawDamping = { dampingScale * angDamping};

    __m128 dampingScale = _mm_min_ps({ 1,1,1,1 }, _mm_max_ps({ 0,0,0,0 }, _mm_sub_ps({ dampingBlendTime,dampingBlendTime,dampingBlendTime,dampingBlendTime },
        _mm_or_ps(_mm_andnot_ps(v36, { 0,0,0,0 }), _mm_and_ps({ v32,v32,v32,v32 }, v36)))));
    ApplyDamping(driftComp, &_mm_shuffle_ps({ 1 }, { 1 }, 0), &dampingScale, &timeStep);
}
/*
/ 
/
*/

void DriftSideForce(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis)
{
    Matrix44 matrix;
    __m128 angVelOut;
    __m128 upVector = GetTransform(chassis, &matrix)->yAxis;
    __m128 sideVector = GetTransform(chassis, &matrix)->xAxis;
    __m128 angVel = *GetAngularVelocity(chassis, &angVelOut);
    __m128 timeStep = { GetDeltaTime((uintptr_t)nfsVehicle), GetDeltaTime((uintptr_t)nfsVehicle), GetDeltaTime((uintptr_t)nfsVehicle), GetDeltaTime((uintptr_t)nfsVehicle) };

    __m128 mulByUpVec = _mm_mul_ps(upVector, angVel);
    __m128 localAngVel = _mm_add_ps(
        _mm_add_ps(
            _mm_shuffle_ps(mulByUpVec, mulByUpVec, 2),
            _mm_shuffle_ps(mulByUpVec, mulByUpVec, 1)),
        mulByUpVec);
    __m128 localAngVelFinal = _mm_shuffle_ps(localAngVel, localAngVel, 0);

    float sideForceMagnitude = driftComp->driftParams->sideForceParams->Side_force_magnitude;
    float sideForceMultiplier = driftComp->driftParams->sideForceParams->Side_force_multiplier;
    float angVelDeg = Radians2Degrees(localAngVelFinal.m128_f32[0]);
    Debug("angVel: %f\n", angVelDeg);
    float driftAngRange = map(abs(angVelDeg), driftComp->driftParams->sideForceParams->Minimum_drift_angle_for_side_force, driftComp->driftParams->sideForceParams->Drift_angle_for_side_force, 0.4f, 1);
    float sideForceSpeedRatio = map(GetSpeedMph(nfsVehicle), driftComp->driftParams->sideForceParams->Speed_for_no_sideforce, driftComp->driftParams->sideForceParams->Speed_for_maximum_side_force, 0, 1);
    bool isCounterSteering = sign(angVelDeg, 1) * nfsVehicle->vehicleInput->steeringInput.X < 0;
    float sideForceToAdd = isCounterSteering ? sideForceMagnitude * sideForceMultiplier : sideForceMagnitude;
    __m128 force = { 
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio) * (sign(angVelDeg, 1) * -1)) * sideVector.m128_f32[0],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio) * (sign(angVelDeg, 1) * -1)) * sideVector.m128_f32[1],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio) * (sign(angVelDeg, 1) * -1)) * sideVector.m128_f32[2],
        sideForceToAdd * ((driftAngRange * sideForceSpeedRatio) * (sign(angVelDeg, 1) * -1)) * sideVector.m128_f32[3] 
    };
    AddWorldCOMForceLogged(chassis, &force, &timeStep);
}

void DriftSpeedRetention(NFSVehicle* nfsVehicle, DriftComponent* driftComp, RaceRigidBody* chassis)
{
    Matrix44 matrix;
    float absSpeedMps = abs(nfsVehicle->speedMps);
    __m128 forwardVector = GetTransform(chassis, &matrix)->zAxis;
    __m128 gasInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->throttleInput.X }, { nfsVehicle->vehicleInput->throttleInput.X }, 0);
    __m128 brakeInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->brakeInput.X }, { nfsVehicle->vehicleInput->brakeInput.X }, 0);
    __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime((uintptr_t)nfsVehicle) }, { GetDeltaTime((uintptr_t)nfsVehicle) }, 0);

    // Call original game's speed retention function, but use MUCH smaller scaling for Reduce_forward_speed_amount and Maintain_entry_speed_amount in the vehicle drift configs.
    MaintainDriftSpeed(driftComp, &gasInput, &brakeInput, &forwardVector, nfsVehicle->raceCarPhysObj, &timeStep);
}

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
    NFSVehicle* nfsVehicle = nullptr;
    uintptr_t nfsVehicleBase = game + 0x02C431A0;
    Debug("gameMainAddress: %I64X\n", game);

    AddWorldCOMForceLogged = (_AddWorldCOMForceLogged)0x144170EE0;
    Debug("AddWorldCOMForceLogged: %I64X\n", AddWorldCOMForceLogged);

    DampPitchYawRoll = (_DampPitchYawRoll)0x1441712D0;
    Debug("DampPitchYawRoll: %I64X\n", DampPitchYawRoll);

    GetTransform = (_GetTransform)0x1441718C0;
    Debug("GetTransform: %I64X\n", GetTransform);

    GetAngularVelocity = (_GetAngularVelocity)0x1441717E0;
    Debug("GetAngularVelocity: %I64X\n", GetAngularVelocity);

    MaintainDriftSpeed = (_MaintainDriftSpeed)0x1441960E0;
    Debug("MaintainDriftSpeed: %I64X\n", MaintainDriftSpeed);

    UpdateDriftScale = (_UpdateDriftScale)0x144197840;
    Debug("UpdateDriftScale: %I64X\n", UpdateDriftScale);

    UpdateDriftAngleDegrees = (_UpdateDriftAngleDegrees)0x144197690;
    Debug("UpdateDriftAngleDegrees: %I64X\n", UpdateDriftAngleDegrees);

    UpdateSideForce = (_UpdateSideForce)0x144197FC0;
    Debug("UpdateSideForce: %I64X\n", UpdateSideForce);

    ApplyDriftForces = (_ApplyDriftForces)0x144193890;
    Debug("ApplyDriftForces: %I64X\n", ApplyDriftForces);

    ApplyDamping = (_ApplyDamping)0x1441935E0;
    Debug("ApplyDamping: %I64X\n", ApplyDamping);

    calcSteeringFromPitchAndYaw = (_calcSteeringFromPitchAndYaw)0x144173270;
    Debug("calcSteeringFromPitchAndYaw: %I64X\n", calcSteeringFromPitchAndYaw);

    while (1)
    {
        nfsVehicle = (NFSVehicle*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x0 });
        DriftComponent* driftComponent = nfsVehicle->driftComponent; //(DriftComponent*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x118, 0xE80, 0x0 }); //nfsVehicle + 0x11A8;
        RaceRigidBody* rigidBody = nfsVehicle->raceRigidBody; //(RaceRigidBody*)FindDMAAddy(GetCurrentProcess(), (uintptr_t)driftComponent + 0x18, { 0x0 });
        
        /* Uncomment to be able to add force upwards by pressing numpad 8, useful for exploring out of bounds and making other players say WTF
        
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

        if (GetAsyncKeyState(VK_UP))
        {
            if (CheckForEnteringDrift(nfsVehicle, driftComponent))
            {
                /*
                __m128 gasInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->throttleInput.X }, { nfsVehicle->vehicleInput->throttleInput.X }, 0);
                __m128 brakeInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->brakeInput.X }, { nfsVehicle->vehicleInput->brakeInput.X }, 0);
                __m128 steeringInput = _mm_shuffle_ps({ nfsVehicle->vehicleInput->steeringInput.X }, { nfsVehicle->vehicleInput->steeringInput.X }, 0);
                */
                __m128 timeStep = _mm_shuffle_ps({ GetDeltaTime((uintptr_t)nfsVehicle) }, { GetDeltaTime((uintptr_t)nfsVehicle) }, 0);

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

                DriftSideForce(nfsVehicle, driftComponent, rigidBody);
                DriftSpeedRetention(nfsVehicle, driftComponent, rigidBody);
                ApplyDriftDamping(nfsVehicle, driftComponent, rigidBody);
                bDriftComp.timeSinceLastDrift = 0;
                driftComponent->timeSinceLastDrift = _mm_shuffle_ps({ bDriftComp.timeSinceLastDrift }, { bDriftComp.timeSinceLastDrift }, 0);
            }
            else if (!CheckForEnteringDrift(nfsVehicle, driftComponent))
            {
                bDriftComp.timeSinceLastDrift += GetDeltaTime((uintptr_t)nfsVehicle);
                driftComponent->timeSinceLastDrift = _mm_shuffle_ps({ bDriftComp.timeSinceLastDrift }, { bDriftComp.timeSinceLastDrift }, 0);
                bDriftComp.yawDamping = { 0,0,0,0 };
                bDriftComp.angDamping = { 0,0,0,0 };
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