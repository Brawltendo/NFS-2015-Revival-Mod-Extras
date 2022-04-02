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
    InjectHook(0x144197250, UpdateDrift_Orig);
    InjectHook(0x1441967A0, RemapSteeringForDrift_Orig);

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