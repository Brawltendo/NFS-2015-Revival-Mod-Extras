#include "pch.h"
#include "PerformanceModification.h"
#include "NFSClasses.h"

float PerformanceModificationComponent::GetModifiedValue(int attributeToModify, float unmodifiedValue)
{
    float value = 0;

    if (m_modifiedValueCache[attributeToModify].useOverride)
        value = m_modifiedValueCache[attributeToModify].modifier;
    else
        value = (unmodifiedValue * m_modifiedValueCache[attributeToModify].scalar) + m_modifiedValueCache[attributeToModify].modifier;

    if (m_modifiers[attributeToModify].modificationType == RaceVehicleModificationType::ModificationType_Scalar)
    {
        return value * m_modifiers[attributeToModify].modifier;
    }
    else
    {
        if (m_modifiers[attributeToModify].modificationType == RaceVehicleModificationType::ModificationType_Addition)
        {
            return value + m_modifiers[attributeToModify].modifier;
        }
        else
        {
            return m_modifiers[attributeToModify].modifier;
        }
    }
}