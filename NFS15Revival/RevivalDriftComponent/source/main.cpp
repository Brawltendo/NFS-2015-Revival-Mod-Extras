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
#include "DriftComponent\RevivalDriftComponent.h"
#include "NFSClasses.h"
#include <algorithm>



#pragma region PATCH_FUNCTIONS_REGION

void __fastcall UpdateDrift_Orig(DriftComponent* driftComponent, const __m128* lvfGasInput, const __m128* lvfBrakeInput, const __m128* lvfSteeringInput, const __m128* lvbSteeringUsingWheel, HandbrakeComponent& lpHandbrake, const class RaceCarPhysicsObject* lpRaceCar, SteeringComponent& lpSteeringComponent, const class SteeringParams& lpSteeringParams, const __m128& lvfAverageSurfaceGripFactor, const __m128& lvfTimeStep, const __m128& lvfInvTimeStep)
{
    NFSVehicle& nfsVehicle = **(NFSVehicle**)lpRaceCar;

    // FWD biased cars probably shouldn't be getting this stuff applied, so let's just do this
    if (nfsVehicle.m_raceCar->mVehicleTuning.torqueSplit <= 0.5f)
    {
        int numWheelsOnGround = 0;
        RevivalDriftComponent::PreUpdate(nfsVehicle, *driftComponent, numWheelsOnGround);
        RevivalDriftComponent::UpdateStabilizationForces(nfsVehicle, *driftComponent, numWheelsOnGround);
        RevivalDriftComponent::Update(nfsVehicle, *driftComponent, numWheelsOnGround);
    }

    RevivalDriftComponent::UpdateHardSteering(nfsVehicle);
}

float __fastcall RemapSteeringForDrift_Orig(DriftComponent* driftComp, float steeringInput, float slipAngleDegrees, float maxSteeringAngle, SteeringComponent& lpSteeringComponent)
{
    // stop the game from normally calculating this because it'll lock up the steering with the new drift code
    return steeringInput;
}

float __fastcall ComputeAckerman(CoreChassis* chassis, float steeringInput, Matrix44& matrix, float wheelBase, float trackWidth, float toeAngle, vec4& steerR, vec4& steerL)
{
    bool goingRight = false;
    float steeringAngle = steeringInput;
    // clamp steering angle <= 180 degrees
    if (steeringInput > 3.1415927f)
        steeringAngle -= 6.2831855f;

    // negative steering angle indicates a right turn
    if (steeringAngle < 0.f)
    {
        goingRight = true;
        steeringAngle = -steeringAngle;
    }

    // Ackermann steering geometry causes the outside wheel to have a smaller turning angle than the inside wheel
    // this is determined by the distance of the wheel to the center of the rear axle
    // this equation is a modified version of 1/tan(L/(R+T/2)), where L is the wheelbase, R is the steering radius, and T is the track width
    vec4 steerLeft;
    vec4 steerRight;
    float steerOutside = (steeringAngle * wheelBase) / (steeringAngle * trackWidth + wheelBase);
    if (goingRight)
    {
        steerLeft = -steerOutside;
        steerRight = -steeringAngle;
    }
    else
    {
        steerLeft = steeringAngle;
        steerRight = steerOutside;
    }

    steerL = _mm_shuffle_ps(
                        _mm_shuffle_ps(VecSin(steerLeft).simdValue, vec4::s_Zero.simdValue, 16),
                        VecCos(steerLeft).simdValue,
                        40);
    steerL = _mm_add_ps(
        _mm_add_ps(
            _mm_mul_ps(VecSwizzleMask(steerL.simdValue, 0), matrix.xAxis),
            _mm_mul_ps(matrix.yAxis, VecSwizzleMask(steerL.simdValue, 85))),
        _mm_mul_ps(matrix.zAxis, VecSwizzleMask(steerL.simdValue, 170)));

    steerR = _mm_shuffle_ps(
                        _mm_shuffle_ps(VecSin(steerRight).simdValue, vec4::s_Zero.simdValue, 16),
                        VecCos(steerRight).simdValue,
                        40);
    steerR = _mm_add_ps(
        _mm_add_ps(
            _mm_mul_ps(VecSwizzleMask(steerR.simdValue, 0), matrix.xAxis),
            _mm_mul_ps(matrix.yAxis, VecSwizzleMask(steerR.simdValue, 85))),
        _mm_mul_ps(matrix.zAxis, VecSwizzleMask(steerR.simdValue, 170)));

    return steeringAngle;
}

void __fastcall DoSteering(CoreChassis* chassis, ChassisStaticState* staticState, ChassisDynamicState* state, ChassisResult* result, vec4& steerL, vec4& steerR)
{
    void(__fastcall* computeAckerman)(void*, float, Matrix44&, float, float, float, vec4&, vec4&) = reinterpret_cast<void(__fastcall*)(void*, float, Matrix44&, float, float, float, vec4&, vec4&)>(0x1441B2F20);

    float steeringInput;
    if (state->isAIControlled)
    {
        float steer_input = state->steer_input;
        steeringInput = -0.78539819;
        if (steer_input >= -0.78539819)
        {
            steeringInput = steer_input;
            if (steer_input >= 0.78539819)
                steeringInput = 0.78539819;
        }
    }
    else
    {
        steeringInput = state->steering_value;
    }
    float speedMph = staticState->toe.Front.min_x;
    float absSpeed = fabsf(state->speed);
    if (speedMph <= (absSpeed * 2.2369399f))
        speedMph = absSpeed * 2.2369399f;
    if (staticState->toe.Front.max_x <= speedMph)
        speedMph = staticState->toe.Front.max_x;
    float frontToe = staticState->toe.Front.Evaluate(speedMph);
    float toeAngle = staticState->toe.Front.min_y;
    if (toeAngle <= frontToe)
        toeAngle = frontToe;
    if (staticState->toe.Front.max_y <= toeAngle)
        toeAngle = staticState->toe.Front.max_y;
    ComputeAckerman(
        chassis,
        steeringInput,
        state->matrix,
        staticState->wheelBase,
        staticState->trackWidth[0],
        toeAngle * 0.017453,
        steerR,
        steerL);
}

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

    PatchDampPitchYawRoll();
    PatchDampLinearVelocityXYZ();
    InjectHook(0x144197250, UpdateDrift_Orig);
    InjectHook(0x1441967A0, RemapSteeringForDrift_Orig);
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