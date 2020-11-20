#pragma once
#include <xmmintrin.h>
#include <vector>

uintptr_t FindDMAAddy(HANDLE hProc, uintptr_t ptr, std::vector<unsigned int> offsets);
float map(float s, float a1, float a2, float b1, float b2);
class PointGraph8 initPointGraph8(__m128 curveData[10]);
float EvaluatePointGraph8(class PointGraph8* pgIn, float xVal);
//int CurrentTick(uintptr_t gameContext);
bool IsGameTicking(DWORD nextTick, int loops, int frameSkip);
float sign(float in, float scale);
float RadiansToDegrees(float radian);