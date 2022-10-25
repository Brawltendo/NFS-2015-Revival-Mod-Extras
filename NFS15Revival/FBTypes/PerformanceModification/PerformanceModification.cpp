#include "pch.h"
#include "PerformanceModification.h"
#include "NFSClasses.h"

#if !USE_REVIVAL_COMPONENT
namespace fb
{
#endif // !USE_REVIVAL_COMPONENT

float PerformanceModificationComponent::GetModifiedValue(int attributeToModify, float unmodifiedValue)
{
	float value = 0.f;

	if (m_modifiedValueCache[attributeToModify].useOverride)
		value = m_modifiedValueCache[attributeToModify].modifier;
	else
		value = (unmodifiedValue * m_modifiedValueCache[attributeToModify].scalar) + m_modifiedValueCache[attributeToModify].modifier;

	switch (m_modifiers[attributeToModify].modificationType)
	{
	case ModificationType_Scalar:
		value *= m_modifiers[attributeToModify].modifier;
		break;
	case ModificationType_Addition:
		value += m_modifiers[attributeToModify].modifier;
		break;
	case ModificationType_Override:
		value = m_modifiers[attributeToModify].modifier;
		break;
	}
	return value;
}

#if !USE_REVIVAL_COMPONENT
}
#endif // !USE_REVIVAL_COMPONENT