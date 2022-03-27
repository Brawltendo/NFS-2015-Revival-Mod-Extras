#include "pch.h"
#include "utils.h"
#include "FBTypes\NFSClasses.h"

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

// Creates a PointGraph from a 4D vector array. The original arrays from the vehicle configs are never greater or less than 10 members.
// First two members of array represent min and max x/y values respectively; all following indices are for each x/y curve point, x being the input, and y being the output.
PointGraph8 initPointGraph8(__m128 curveData[10])
{
	PointGraph8 pgOut{};

	pgOut.min_x = curveData[0].m128_f32[0];
	pgOut.min_y = curveData[0].m128_f32[1];
	pgOut.max_x = curveData[1].m128_f32[0];
	pgOut.max_y = curveData[1].m128_f32[1];
	float rangeX = curveData[1].m128_f32[0] - curveData[0].m128_f32[0];
	float rangeY = curveData[1].m128_f32[1] - curveData[0].m128_f32[1];
	for (int i = 0; i < 8; i++)
	{
		pgOut.x[i] = (rangeX * curveData[i + 2].m128_f32[0]) + pgOut.min_x;
		pgOut.y[i] = (rangeY * curveData[i + 2].m128_f32[1]) + pgOut.min_y;
	}
	return pgOut;
}

float EvaluatePointGraph8(PointGraph8* pgIn, float input)
{
	if (input < pgIn->x[0]) return pgIn->y[0];
	if (input >= pgIn->x[7]) return pgIn->y[7];
	float currentX = 0;
	int i = 1;
	while (1)
	{
		currentX = pgIn->x[i];
		if (input < currentX)
			break;
		++i;
		if (i >= 8)
			return pgIn->y[0];
	}
	float prevX = pgIn->x[i - 1];
	float xDiff = currentX - prevX;
	float outVal = pgIn->y[i];
	if (xDiff > 0.0)
		outVal = ((outVal - pgIn->y[i - 1]) / xDiff) * (input - prevX) + pgIn->y[i - 1];
	return outVal;
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
