#include "pch.h"
#include "psapi.h"
#include <FBTypes\NFSClasses.h>

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

bool IsGameTicking(DWORD nextTick, int loops, int frameSkip)
{
    return GetTickCount() > nextTick && loops < frameSkip;
}

DWORD WINAPI Start(LPVOID lpParam)
{
    Sleep(1000);
    FILE* pFile = nullptr;

    // Initialize console in order to output useful debug data

    const int TICKS_PER_SECOND = 30;
    const int SKIP_TICKS = 1000 / TICKS_PER_SECOND;
    const int MAX_FRAMESKIP = 5;
    DWORD nextTick = GetTickCount();
    int loops;
    float interpolation;

    HWND gameWindow = NULL;
    bool hasFocus;
    uintptr_t game = (uintptr_t)GetModuleHandle(NULL);
    NFSVehicle* nfsVehicle = NULL;
    DriftComponent* driftComponent = NULL;
    uintptr_t nfsVehicleBase = game + 0x02C431A0;
    uintptr_t gameContext = game + 0x0289CDC0;

    while (1)
    {
        nfsVehicle = (NFSVehicle*)FindDMAAddy(GetCurrentProcess(), nfsVehicleBase, { 0x28, 0x20, 0x0, 0xD8, 0x0 });

        // Check to see whether or not the NFSVehicle pointer is valid
        float nfsVehicleChecker;
        float nfsVehicleChecker1;

        ReadProcessMemory(GetCurrentProcess(), (BYTE*)(uintptr_t)nfsVehicle + 0x64, &nfsVehicleChecker, sizeof(nfsVehicleChecker), nullptr);
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)(uintptr_t)nfsVehicle + 0x68, &nfsVehicleChecker1, sizeof(nfsVehicleChecker1), nullptr);
        bool isNFSVehicleValid = nfsVehicleChecker == 150 && nfsVehicleChecker1 == 10;

        loops = 0;
        while (IsGameTicking(nextTick, loops, MAX_FRAMESKIP))
        {
            if (isNFSVehicleValid)
            {
                driftComponent = nfsVehicle->driftComponent;
                driftComponent->driftParams->driftTriggerParams->Minimum_speed_to_enter_drift = 9999.f;
            }
            nextTick += SKIP_TICKS;
            loops++;
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