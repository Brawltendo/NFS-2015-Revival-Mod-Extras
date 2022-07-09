#include "pch.h"
#include "utils.h"
#include "FBTypes\NFSClasses.h"


float map(float in, float inMin, float inMax, float outMin, float outMax)
{
	return outMin + (in - inMin) * (outMax - outMin) / (inMax - inMin);
}

float clamp(float x, float min, float max)
{
	return fminf(fmaxf(x, min), max);
}

float clamp01(float x)
{
	return fminf(fmaxf(x, 0.f), 1.f);
}

float sign(float in, float scale)
{
	if (in > 0)
		return 1 * fabsf(scale);
	if (in < 0)
		return -1 * fabsf(scale);
	else
		return 0;
}

float sign(float in)
{
	if (in > 0.f)
		return 1.f;
	if (in < 0.f)
		return -1.f;
	else
		return 0.f;
}

float GetSpeedMph(NFSVehicle* nfsVehicle)
{
	return nfsVehicle->m_forwardSpeed * 2.2369399f;
}
