#include "pch.h"
#include "psapi.h"
#include "main.h"
#include "util\utils.h"
#include "util\memoryutils.h"
#ifdef _DEBUG
#include <assert.h>
#include "util\debug\render\DebugRenderer.h"
#include "util\debug\render\DebugRendererFB.h"
#endif
#if USE_REVIVAL_COMPONENT
#include "DriftComponent\RevivalDriftComponent.h"
#else
#include "VehiclePhysics/Components/DriftComponent.h"
#include "VehiclePhysics/Components/DriftParams.h"
#include <FBTypes/VehiclePhysics/Components/SteeringComponent.h>
#include <FBTypes/VehiclePhysics/Components/HandbrakeComponent.h>
#include "PerformanceModification/PerformanceModification.h"
#endif
#include "NFSClasses.h"
#include <algorithm>
#include "util/debug/debug.h"


#pragma region PATCH_FUNCTIONS_REGION

#if USE_REVIVAL_COMPONENT
void __fastcall UpdateDrift_Orig(DriftComponent* driftComponent, const __m128* lvfGasInput, const __m128* lvfBrakeInput, const __m128* lvfSteeringInput, const __m128* lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, const class RaceCarPhysicsObject* lpRaceCar, SteeringComponent& lpSteeringComponent, const class SteeringParams& lpSteeringParams, const __m128& lvfAverageSurfaceGripFactor, const __m128& lvfTimeStep, const __m128& lvfInvTimeStep)
{
    NFSVehicle& nfsVehicle = **(NFSVehicle**)lpRaceCar;

    int numWheelsOnGround = 0;
    // use different global params for FWD cars
    const DriftParameters& params = nfsVehicle.m_raceCar->mVehicleTuning.torqueSplit > 0.5f ? s_GlobalDriftParamsFWD : s_GlobalDriftParams;
    RevivalDriftComponent::PreUpdate(nfsVehicle, *driftComponent, numWheelsOnGround, params);
    RevivalDriftComponent::UpdateStabilizationForces(nfsVehicle, *driftComponent, numWheelsOnGround, params);
    RevivalDriftComponent::Update(nfsVehicle, *driftComponent, numWheelsOnGround, params);
    RevivalDriftComponent::UpdateHardSteering(nfsVehicle);
}

float __fastcall RemapSteeringForDrift_Orig(DriftComponent* driftComp, float steeringInput, float slipAngleDegrees, float maxSteeringAngle, SteeringComponent& lpSteeringComponent)
{
    // stop the game from normally calculating this because it'll lock up the steering with the new drift code
    return steeringInput;
}
#else
void __fastcall ExitDrift_Override(fb::DriftComponent* comp, fb::SteeringComponent& lpSteeringComponent)
{
    comp->ExitDrift(lpSteeringComponent);
}

void UpdateDriftState_Override(fb::DriftComponent* comp, const Vec4& lvfGasInput, const Vec4& lvfBrakeInput, const Vec4& lvfSteeringInput, const Vec4& lvbSteeringUsingWheel, fb::HandbrakeComponent& lpHandbrake, fb::SteeringComponent& lpSteeringComponent, class RaceCarPhysicsObject& lpRaceCar, float lfSpeedMPS, const Vec4& lvfTimeStep, const Vec4& lvfInvTimeStep)
{
    comp->UpdateDriftState(lvfGasInput, lvfBrakeInput, lvfSteeringInput, lvbSteeringUsingWheel, lpHandbrake, lpSteeringComponent, lpRaceCar, lfSpeedMPS, lvfTimeStep, lvfInvTimeStep);
}

float __fastcall calcSteeringFromPitchAndYaw_Override(NFSVehicle* const nfsVehicle, float pitch, float yaw)
{
    //return clamp(nfsVehicle->m_input->m_yaw / (RadiansToDegrees(nfsVehicle->m_raceCar->mSteeringResultState.steeringRangeLeft) / nfsVehicle->m_raceCar->mVehicleTuning.maxSteeringAngle), -1.f, 1.f);  
    // they do some trig to get the steering input with an input that's completely unused, so let's just simplify that
    return nfsVehicle->m_input->m_yaw;
}
#endif

void __fastcall ComputeAckerman(CoreChassis* chassis, float steering, Matrix44& bodyMatrix, float wheelBase, float trackWidth, float frontToe, Vec4& left, Vec4& right, float& angleLeft, float& angleRight)
{
    float steer_inside = steering;
    // invert steering angle if past 180 degrees
    if (steering > DegreesToRadians(180.f))
        steer_inside -= DegreesToRadians(360.f);

    // Positive Ackermann steering geometry causes the inside wheel to have a larger turning angle than the outside wheel
    // this is determined by the distance of the wheel to the center of the rear axle
    // to get the inside wheel's angle we'll use the equation (A*L)/(A*T+L), where L is the wheelbase, A is the steering angle, and T is the front track width 
    if (steer_inside >= 0.f) // positive steering angle indicates a left turn
    {
        angleLeft  = (steer_inside * wheelBase) / (steer_inside * trackWidth + wheelBase);
        angleRight = steer_inside;
    }
    else
    {
        angleLeft  = steer_inside;
        angleRight = -((-steer_inside * wheelBase) / (-steer_inside * trackWidth + wheelBase));
    }
    Vec4 steerL(angleLeft  + frontToe);
    Vec4 steerR(angleRight - frontToe);

    right = _mm_shuffle_ps(
                        _mm_shuffle_ps(VecSin(steerR).simdValue, Vec4::kZero.simdValue, 16),
                        VecCos(steerR).simdValue,
                        40);
    right = _mm_add_ps(
        _mm_add_ps(
            _mm_mul_ps(VecSwizzleMask(right.simdValue, 0), bodyMatrix.xAxis),
            _mm_mul_ps(bodyMatrix.yAxis, VecSwizzleMask(right.simdValue, 85))),
        _mm_mul_ps(bodyMatrix.zAxis, VecSwizzleMask(right.simdValue, 170)));

    left = _mm_shuffle_ps(
                        _mm_shuffle_ps(VecSin(steerL).simdValue, Vec4::kZero.simdValue, 16),
                        VecCos(steerL).simdValue,
                        40);
    left = _mm_add_ps(
        _mm_add_ps(
            _mm_mul_ps(VecSwizzleMask(left.simdValue, 0), bodyMatrix.xAxis),
            _mm_mul_ps(bodyMatrix.yAxis, VecSwizzleMask(left.simdValue, 85))),
        _mm_mul_ps(bodyMatrix.zAxis, VecSwizzleMask(left.simdValue, 170)));
}

//void __fastcall DoSteering(CoreChassis* chassis, ChassisStaticState* staticState, ChassisDynamicState* state, ChassisResult* result, Vec4& steerL, Vec4& steerR)
//{
//    void(__fastcall* computeAckerman)(void*, float, Matrix44&, float, float, float, Vec4&, Vec4&) = reinterpret_cast<void(__fastcall*)(void*, float, Matrix44&, float, float, float, Vec4&, Vec4&)>(0x1441B2F20);
//
//    float steeringInput;
//    if (state->isAIControlled)
//    {
//        float steer_input = state->steer_input;
//        steeringInput = -0.78539819;
//        if (steer_input >= -0.78539819)
//        {
//            steeringInput = steer_input;
//            if (steer_input >= 0.78539819)
//                steeringInput = 0.78539819;
//        }
//    }
//    else
//    {
//        steeringInput = state->steering_value;
//    }
//    float speedMph = staticState->toe.Front.min_x;
//    float absSpeed = fabsf(state->speed);
//    if (speedMph <= (absSpeed * 2.2369399f))
//        speedMph = absSpeed * 2.2369399f;
//    if (staticState->toe.Front.max_x <= speedMph)
//        speedMph = staticState->toe.Front.max_x;
//    float frontToe = staticState->toe.Front.Evaluate(speedMph);
//    float toeAngle = staticState->toe.Front.min_y;
//    if (toeAngle <= frontToe)
//        toeAngle = frontToe;
//    if (staticState->toe.Front.max_y <= toeAngle)
//        toeAngle = staticState->toe.Front.max_y;
//    ComputeAckerman(
//        chassis,
//        steeringInput,
//        state->matrix,
//        staticState->wheelBase,
//        staticState->trackWidth[0],
//        toeAngle * 0.017453,
//        steerR,
//        steerL);
//}

inline void PatchDampPitchYawRoll()
{
    // the goal here is to essentially just turn this into a "SetAngularVelocity" function because I don't know where the actual native one is

    // set 1.0f to 0.0f
    {
        uint8_t patch[] = { 0x00, 0x00, 0x00, 0x00 };
        PatchInstruction(0x144171355, patch, sizeof(patch));
        PatchInstruction(0x144171361, patch, sizeof(patch));
        PatchInstruction(0x14417136D, patch, sizeof(patch));
    }
    
    // patch subps to addps
    {
        uint8_t patch[] = { 0x58 };
        PatchInstruction(0x1441713AD, patch, sizeof(patch));
    }
    
    // nop movss after each powf call
    {
        uint8_t patch[] = { 0x90, 0x90, 0x90, 0x90, 0x90, 0x90 };
        PatchInstruction(0x1441713C8, patch, sizeof(patch));
        PatchInstruction(0x1441713DC, patch, sizeof(patch));
        PatchInstruction(0x1441713FC, patch, sizeof(patch));
    }
    
    // patch mulps to addps
    {
        uint8_t patch[] = { 0x58 };
        PatchInstruction(0x144171529, patch, sizeof(patch));
    }
    
}

inline void PatchDampLinearVelocityXYZ()
{

    // set 1.0f to 0.0f
    {
        uint8_t patch[] = { 0x00, 0x00, 0x00, 0x00 };
        PatchInstruction(0x144171075, patch, sizeof(patch));
        PatchInstruction(0x144171081, patch, sizeof(patch));
        PatchInstruction(0x14417108D, patch, sizeof(patch));
    }

    // patch subps to addps
    {
        uint8_t patch[] = { 0x58 };
        PatchInstruction(0x1441710C4, patch, sizeof(patch));
    }

    // nop movss after each powf call
    {
        uint8_t patch[] = { 0x90, 0x90, 0x90, 0x90, 0x90, 0x90 };
        PatchInstruction(0x1441710D6, patch, sizeof(patch));
        PatchInstruction(0x1441710EA, patch, sizeof(patch));
        PatchInstruction(0x14417110A, patch, sizeof(patch));
    }

    // patch mulps to addps
    {
        uint8_t patch[] = { 0x58 };
        PatchInstruction(0x144171226, patch, sizeof(patch));
    }

}

#pragma endregion PATCH_FUNCTIONS_REGION

DWORD WINAPI Start(LPVOID lpParam)
{
    Sleep(1000);
    FILE* pFile = nullptr;

    #ifdef _DEBUG
    // Initialize console in order to output useful debug data
    AllocConsole();
    freopen_s(&pFile, "CONIN$", "r", stdin);
    freopen_s(&pFile, "CONOUT$", "w", stdout);
    freopen_s(&pFile, "CONOUT$", "w", stderr);
    SetWindowTextW(GetConsoleWindow(), L"Need for Speed™ Revival");

    // start up debug renderer
    //DebugRenderer::Init();
    #endif

    HWND gameWindow = NULL;
    uintptr_t game = (uintptr_t)GetModuleHandle(NULL);
    uintptr_t gameContext = game + 0x0289CDC0;
    DebugLogPrint("gameContext: %I64X\n", gameContext);
    
    #if !USE_REVIVAL_COMPONENT
    InjectHook(0x144194D20, ExitDrift_Override);
    InjectHook(0x1441980F0, UpdateDriftState_Override);
    InjectHook(0x144173270, calcSteeringFromPitchAndYaw_Override);
    #else
    PatchDampPitchYawRoll();
    PatchDampLinearVelocityXYZ();
    InjectHook(0x144197250, UpdateDrift_Orig);
    InjectHook(0x1441967A0, RemapSteeringForDrift_Orig);
    #endif
    InjectHook(0x1441B2F20, ComputeAckerman);
    //InjectHook(0x1441B3CF0, DoSteering);

    #ifdef _DEBUG
    fb::DebugRenderer::Init();
    #endif
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
        //kiero::shutdown();
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