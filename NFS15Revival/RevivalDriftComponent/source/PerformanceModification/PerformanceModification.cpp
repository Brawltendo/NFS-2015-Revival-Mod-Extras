#include "pch.h"
#include "PerformanceModification.h"

float GetModifiedValueSideForceMagnitude(PerformanceModificationComponent* perfMod, float unmodifiedSideForce)
{
    float modified;
    float finalModified;
    RaceVehicleModificationType modType;

    if ((bool)perfMod + 0x304)
    {
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x300, &modified, sizeof(modified), nullptr);
    }
    else
    {
        float modified1;
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x2FC, &modified, sizeof(modified), nullptr);
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x300, &modified1, sizeof(modified1), nullptr);

        modified = (modified * unmodifiedSideForce) + modified1;
    }
    ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0xA04, &modType, sizeof(modType), nullptr);
    if (modType == RaceVehicleModificationType::ModificationType_Scalar )
    {
        float scalar;
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0xA08, &scalar, sizeof(scalar), nullptr);
        return modified * scalar; // ModificationType_Scalar
    }
    else
    {
        RaceVehicleModificationType nextModType = (RaceVehicleModificationType)((int)modType - 1);
        if (nextModType == RaceVehicleModificationType::ModificationType_Override)
        {
            float override;
            ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0xA08, &override, sizeof(override), nullptr);
            return override; // ModificationType_Override
        }
        else
        {
            float addend;
            ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0xA08, &addend, sizeof(addend), nullptr);
            return modified + addend; // ModificationType_Addition
        }
    }
}

float GetModifiedValueYawTorque(PerformanceModificationComponent* perfMod, float unmodifiedYawTorque)
{
    float modified;
    float finalModified;
    RaceVehicleModificationType modType;

    if ((bool)perfMod + 0x1CC)
    {
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x1C8, &modified, sizeof(modified), nullptr);
    }
    else
    {
        float modified1;
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x1C4, &modified, sizeof(modified), nullptr);
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x1C8, &modified1, sizeof(modified1), nullptr);

        modified = (modified * unmodifiedYawTorque) + modified1;
    }
    ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x934, &modType, sizeof(modType), nullptr);
    if (modType == RaceVehicleModificationType::ModificationType_Scalar)
    {
        float scalar;
        ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x938, &scalar, sizeof(scalar), nullptr);
        return modified * scalar; // ModificationType_Scalar
    }
    else
    {
        RaceVehicleModificationType nextModType = (RaceVehicleModificationType)((int)modType - 1);
        if (nextModType == RaceVehicleModificationType::ModificationType_Override)
        {
            float override;
            ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x938, &override, sizeof(override), nullptr);
            return override; // ModificationType_Override
        }
        else
        {
            float addend;
            ReadProcessMemory(GetCurrentProcess(), (BYTE*)perfMod + 0x938, &addend, sizeof(addend), nullptr);
            return modified + addend; // ModificationType_Addition
        }
    }
}