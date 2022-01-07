#pragma once
#include <xmmintrin.h>
#include <vector>

constexpr auto Epsilon = 0.00000011920929;

_forceinline __m128 operator + (const __m128& l, const __m128& r) { return __m128(_mm_add_ps(l, r)); }
_forceinline __m128 operator - (const __m128& l, const __m128& r) { return __m128(_mm_sub_ps(l, r)); }
_forceinline __m128 operator * (const __m128& l, const __m128& r) { return __m128(_mm_mul_ps(l, r)); }
_forceinline __m128 operator / (const __m128& l, const __m128& r) { return __m128(_mm_div_ps(l, r)); }
_forceinline __m128 operator + (const __m128& l, const float& r) { return __m128(_mm_add_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator - (const __m128& l, const float& r) { return __m128(_mm_sub_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator * (const __m128& l, const float& r) { return __m128(_mm_mul_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator / (const __m128& l, const float& r) { return __m128(_mm_div_ps(l, _mm_set1_ps(r))); }

enum
{
	PATCH_CALL,
	PATCH_JUMP,
	PATCH_NOTHING,
};

uintptr_t FindDMAAddy(HANDLE hProc, uintptr_t ptr, std::vector<unsigned int> offsets);
float map(float s, float a1, float a2, float b1, float b2);
class PointGraph8 initPointGraph8(__m128 curveData[10]);
float EvaluatePointGraph8(class PointGraph8* pgIn, float xVal);
//int CurrentTick(uintptr_t gameContext);
bool IsGameTicking(DWORD nextTick, int loops, int frameSkip);
float sign(float in, float scale);
float GetSpeedMph(class NFSVehicle* nfsVehicle);
float RadiansToDegrees(float radian);
