#include "pch.h"
#include "PerformanceModification.h"
#include "NFSClasses.h"

// Ghost/Criterion why do you have separate functions for every single performance mod instead of doing it like this???

float GetModifiedValue(class PerformanceModificationComponent* perfMod, int attributeToModify, float unmodifiedValue)
{
    float value = 0;

    if (perfMod->m_modifiedValueCache[attributeToModify].useOverride) value = perfMod->m_modifiedValueCache[attributeToModify].modifier;

    else
    {
        value = (unmodifiedValue * perfMod->m_modifiedValueCache[attributeToModify].scalar) + perfMod->m_modifiedValueCache[attributeToModify].modifier;
    }

    if (perfMod->m_modifiers[attributeToModify].modificationType == RaceVehicleModificationType::ModificationType_Scalar)
    {
        return value * perfMod->m_modifiers[attributeToModify].modifier;
    }
    else
    {
        if (perfMod->m_modifiers[attributeToModify].modificationType == RaceVehicleModificationType::ModificationType_Addition)
        {
            return value + perfMod->m_modifiers[attributeToModify].modifier;
        }
        else
        {
            return perfMod->m_modifiers[attributeToModify].modifier;
        }
    }
}