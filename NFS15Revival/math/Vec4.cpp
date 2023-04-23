#include "Vec4.h"


#pragma region Constants

const Vec4 Vec4::kZero(0.f);
const Vec4 Vec4::kQuarter(0.25f);
const Vec4 Vec4::kHalf(0.5f);
const Vec4 Vec4::kOne(1.f);
const Vec4 Vec4::kMinusOne(-1.f);
const Vec4 Vec4::kEpsilon(0.00000011920929f);
const Vec4 Vec4::kUnitX(1.f, 0.f, 1.f);
const Vec4 Vec4::kUnitY(0.f, 1.f, 0.f);
const Vec4 Vec4::kUnitZ(0.f, 0.f, 1.f);
const Vec4 Vec4::kUnitW(0.f, 0.f, 0.f, 1.f);

const Vec4 kNegOneOverPi(-0.15915494f);
const Vec4 kCosXCoef(6.2831855f);
const Vec4 kCosX3Coef(-41.341702f);
const Vec4 kCosX5Coef(81.605247f);
const Vec4 kCosX7Coef(-76.705856f);
const Vec4 kCosX9Coef(42.058693f);
const Vec4 kCosX11Coef(-15.094643f);
const Vec4 kCosX13Coef(3.8199525f);
const Vec4 kCosX15Coef(-0.7181223f);
const Vec4 kCosX17Coef(0.10422916f);
const Vec4 kCosX19Coef(-0.012031586f);

#pragma endregion Constants



#pragma region Vector Comparisons

// Returns if the input mask is true or false (all members)
bool VecIsTrue(const Vec4& v)
{
	return _mm_movemask_ps(v) == 0xF;
}

// Returns if the input mask is true or false (specified members)
// Use CmpMask to create the input mask
bool VecIsTrue(const Vec4& v, uint8_t cmpMask)
{
	return _mm_movemask_ps(v) == cmpMask;
}

// Returns if the input mask is true or false (any members)
bool VecIsTrueAny(const Vec4& v)
{
	return _mm_movemask_ps(v) != 0;
}

// Returns true if a < b (all members)
bool VecCmpLT(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmplt_ps(a, b);
	return _mm_movemask_ps(cmp) == 0xF;
}

// Returns true if a < b (specified members)
// Use CmpMask to create the input mask
bool VecCmpLT(const Vec4& a, const Vec4& b, uint8_t cmpMask)
{
	Vec4 cmp = _mm_cmplt_ps(a, b);
	return (_mm_movemask_ps(cmp) & cmpMask) == cmpMask;
}

// Returns true if a < b (any members)
bool VecCmpLTAny(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmplt_ps(a, b);
	return _mm_movemask_ps(cmp) != 0;
}

// Returns true if a <= b (all members)
bool VecCmpLE(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmple_ps(a, b);
	return _mm_movemask_ps(cmp) == 0xF;
}

// Returns true if a <= b (specified members)
// Use CmpMask to create the input mask
bool VecCmpLE(const Vec4& a, const Vec4& b, uint8_t cmpMask)
{
	Vec4 cmp = _mm_cmple_ps(a, b);
	return (_mm_movemask_ps(cmp) & cmpMask) == cmpMask;
}

// Returns true if a <= b (any members)
bool VecCmpLEAny(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmple_ps(a, b);
	return _mm_movemask_ps(cmp) != 0;
}

// Returns true if a > b (all members)
bool VecCmpGT(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpgt_ps(a, b);
	return _mm_movemask_ps(cmp) == 0xF;
}

// Returns true if a > b (specified members)
// Use CmpMask to create the input mask
bool VecCmpGT(const Vec4& a, const Vec4& b, uint8_t cmpMask)
{
	Vec4 cmp = _mm_cmpgt_ps(a, b);
	return (_mm_movemask_ps(cmp) & cmpMask) == cmpMask;
}

// Returns true if a > b (any members)
bool VecCmpGTAny(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpgt_ps(a, b);
	return _mm_movemask_ps(cmp) != 0;
}

// Returns true if a >= b (all members)
bool VecCmpGE(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpge_ps(a, b);
	return _mm_movemask_ps(cmp) == 0xF;
}

// Returns true if a >= b (specified members)
// Use CmpMask to create the input mask
bool VecCmpGE(const Vec4& a, const Vec4& b, uint8_t cmpMask)
{
	Vec4 cmp = _mm_cmpge_ps(a, b);
	return (_mm_movemask_ps(cmp) & cmpMask) == cmpMask;
}

// Returns true if a >= b (any members)
bool VecCmpGEAny(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpge_ps(a, b);
	return _mm_movemask_ps(cmp) != 0;
}

// Returns true if a == b (all members)
bool VecCmpEQ(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpeq_ps(a, b);
	return _mm_movemask_ps(cmp) == 0xF;
}

// Returns true if a == b (specified members)
// Use CmpMask to create the input mask
bool VecCmpEQ(const Vec4& a, const Vec4& b, uint8_t cmpMask)
{
	Vec4 cmp = _mm_cmpeq_ps(a, b);
	return (_mm_movemask_ps(cmp) & cmpMask) == cmpMask;
}

// Returns true if a == b (any members)
bool VecCmpEQAny(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpeq_ps(a, b);
	return _mm_movemask_ps(cmp) != 0;
}

// Returns true if a != b (all members)
bool VecCmpNEQ(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpneq_ps(a, b);
	return _mm_movemask_ps(cmp) == 0xF;
}

// Returns true if a != b (specified members)
// Use CmpMask to create the input mask
bool VecCmpNEQ(const Vec4& a, const Vec4& b, uint8_t cmpMask)
{
	Vec4 cmp = _mm_cmpneq_ps(a, b);
	return (_mm_movemask_ps(cmp) & cmpMask) == cmpMask;
}

// Returns true if a != b (any members)
bool VecCmpNEQAny(const Vec4& a, const Vec4& b)
{
	Vec4 cmp = _mm_cmpneq_ps(a, b);
	return _mm_movemask_ps(cmp) != 0;
}

#pragma endregion Vector Comparisons



#pragma region Math Operations

// Adds 2 vectors
Vec4 VecAdd(const Vec4& l, const Vec4& r)
{
	return _mm_add_ps(l, r);
}

// Subtracts 2 vectors
Vec4 VecSub(const Vec4& l, const Vec4& r)
{
	return _mm_sub_ps(l, r);
}

// Multiplies 2 vectors
Vec4 VecMul(const Vec4& l, const Vec4& r)
{
	return _mm_mul_ps(l, r);
}

// Divides 2 vectors
Vec4 VecDiv(const Vec4& l, const Vec4& r)
{
	return _mm_div_ps(l, r);
}

#pragma endregion Math Operations



#pragma region Logical Operations

Vec4 VecAnd(const Vec4& l, const Vec4& r)
{
	return _mm_and_ps(l, r);
}

Vec4 VecAndNot(const Vec4& l, const Vec4& r)
{
	return _mm_andnot_ps(l, r);
}

Vec4 VecOr(const Vec4& l, const Vec4& r)
{
	return _mm_or_ps(l, r);
}

Vec4 VecXor(const Vec4& l, const Vec4& r)
{
	return _mm_xor_ps(l, r);
}

Vec4 VecNot(const Vec4& v)
{
	Vec4 notMask = _mm_castsi128_ps(_mm_set1_epi32(0xFFFFFFFF));
	return _mm_xor_ps(v, notMask);
}

#pragma endregion Logical Operations



#pragma region Misc

Vec4& SimdToVec4(const __m128& simd)
{
	return *((Vec4*)&simd);
}

// Converts a bool input to an equivalent vector mask
Vec4 BoolToVecMask(bool condition)
{
	union { uint32_t ui; float flt; } select[2];
	select[0].ui = 0; select[1].ui = 0xFFFFFFFF;
	return Vec4(select[condition ? 1 : 0].flt);
}

// Returns a vector based on the input condition
Vec4 VecSelect(const Vec4& valIfTrue, const Vec4& valIfFalse, const Vec4& condition)
{
	return _mm_xor_ps(_mm_and_ps(_mm_xor_ps(valIfTrue, valIfFalse), condition), valIfFalse);
}

// Returns a vector based on the input condition
__m128 SelectVector(const __m128& valIfTrue, const __m128& valIfFalse, const __m128& condition)
{
	return _mm_xor_ps(_mm_and_ps(_mm_xor_ps(valIfTrue, valIfFalse), condition), valIfFalse);
}

// Returns the minimum of 2 vectors
Vec4 VecMin(const Vec4& a, const Vec4& b)
{
	return _mm_min_ps(a, b);
}

// Returns the maximum of 2 vectors
Vec4 VecMax(const Vec4& a, const Vec4& b)
{
	return _mm_max_ps(a, b);
}

// Clamps the input vector between a min and max vector
Vec4 VecClamp(const Vec4& v, const Vec4& min, const Vec4& max)
{
	return VecMin(VecMax(v, min), max);
}

// Returns the absolute value of the input vector
Vec4 VecAbs(Vec4& v)
{
	return _mm_max_ps(v * Vec4::kMinusOne, v);
	//return _mm_and_ps(v.simdValue, _mm_castsi128_ps(_mm_set_epi32(0, 0, 0, ~(1 << 31))));
}

// Returns the absolute value of the input vector
Vec4 VecAbs(const Vec4& v)
{
	return _mm_max_ps(v * Vec4::kMinusOne, v);
	//return _mm_and_ps(v.simdValue, _mm_castsi128_ps(_mm_set_epi32(0, 0, 0, ~(1 << 31))));
}

// Linearly interpolates between a & b using interpolator t
Vec4 VecLerp(const Vec4& a, const Vec4& b, const Vec4& t)
{
	Vec4 range = b - a;
	return range * t + a;
	//return (a * (Vec4::kOne - t)) + (b * t);
}

// Returns a [0,1] interpolator from a given input and range
Vec4 VecRamp(const Vec4& val, const Vec4& low, const Vec4& high)
{
	Vec4 range = VecMax(high - low, Vec4::kEpsilon);
	return VecClamp((val - low) / range, Vec4::kZero, Vec4::kOne);
}

// Returns the sign of the input vector
Vec4 VecSign(const Vec4& v)
{
	return _mm_or_ps(_mm_and_ps(Vec4::kZero < v, Vec4::kOne), _mm_and_ps(v < Vec4::kZero, Vec4::kMinusOne));
}

// Negates the input vector
Vec4 VecNeg(const Vec4& v)
{
	return Vec4::kZero - v;
}

#pragma endregion Misc



#pragma region Vector Math

// Returns the reciprocal of the input vector
Vec4 VecRecip(Vec4& v)
{
	return _mm_rcp_ps(v);
}

// Performs the dot product of 2 vectors
float Dot3(Vec4 const& a, Vec4 const& b)
{
	Vec4 mul = a * b;
	Vec4 dot = VecSwizzleMask(mul, k_xxxx) + VecSwizzleMask(mul, k_yyyy) + VecSwizzleMask(mul, k_zzzz);
	return _mm_cvtss_f32(dot);
}

// Performs the dot product of 2 vectors
Vec4 VecDot3(Vec4 const& a, Vec4 const& b)
{
	Vec4 mul = a * b;
	Vec4 dot = VecSwizzleMask(mul, k_xxxx) + VecSwizzleMask(mul, k_yyyy) + VecSwizzleMask(mul, k_zzzz);
	return dot;
}

// Returns the length of the input vector
Vec4 VecLength3(Vec4& v)
{
	return _mm_sqrt_ps(VecDot3(v, v));
}

// Normalizes the input vector
float VecNormal3(Vec4& v)
{
	Vec4 dp = Dot3(v, v);
	Vec4 cond = dp.simdValue == Vec4::kZero.simdValue;
	//__m128 sel   = _mm_or_ps(_mm_andnot_ps(cmp1, dp), _mm_and_ps(Vec4::kOne.simdValue, cond));
	Vec4 sel = VecSelect(Vec4::kOne, dp, cond);
	Vec4 rsqrt = _mm_rsqrt_ps(sel.simdValue);
	Vec4 tmp1 = sel * rsqrt;
	//__m128 tmp2  = _mm_andnot_ps(cond, Vec4::kHalf.simdValue * tmp1 * (Vec4::kOne.simdValue - rsqrt * tmp1) + tmp1);
	//__m128 tmp3  = _mm_or_ps(tmp2, _mm_and_ps(Vec4::kZero.simdValue, cond));
	Vec4 tmp3 = VecSelect(Vec4::kZero, Vec4::kHalf * tmp1 * (Vec4::kOne - rsqrt * tmp1) + tmp1, cond);
	return _mm_cvtss_f32(tmp3.simdValue);
}

// Performs the cross product of 2 vectors
Vec4 Cross(Vec4 const& a, Vec4 const& b)
{
	Vec4 tmp0 = _mm_shuffle_ps(a.simdValue, a.simdValue, _MM_SHUFFLE(3, 0, 2, 1));
	Vec4 tmp1 = _mm_shuffle_ps(b.simdValue, b.simdValue, _MM_SHUFFLE(3, 1, 0, 2));
	Vec4 tmp2 = tmp0 * b;
	Vec4 tmp3 = tmp0 * tmp1;
	Vec4 tmp4 = _mm_shuffle_ps(tmp2.simdValue, tmp2.simdValue, _MM_SHUFFLE(3, 0, 2, 1));
	return tmp3 - tmp4;
}

// Calculates the Taylor series cosine of the input vector
Vec4 VecCos(Vec4& in)
{
	Vec4 v1 = VecAbs(in) * kNegOneOverPi;
	const Vec4 constVal(1.2582912e7f);
	Vec4 v2 = (((v1 - constVal) + constVal) - v1) - Vec4::kHalf;

	Vec4 x = VecAbs(v2) - Vec4::kQuarter;
	Vec4 x2 = x * x;
	Vec4 accum = x * kCosXCoef;
	x *= x2;
	accum += x * kCosX3Coef;
	x *= x2;
	accum += x * kCosX5Coef;
	x *= x2;
	accum += x * kCosX7Coef;
	x *= x2;
	accum += x * kCosX9Coef;
	x *= x2;
	accum += x * kCosX11Coef;
	x *= x2;
	accum += x * kCosX13Coef;
	x *= x2;
	accum += x * kCosX15Coef;
	x *= x2;
	accum += x * kCosX17Coef;
	x *= x2;
	accum += x * kCosX19Coef;
	return _mm_max_ps(Vec4::kMinusOne, _mm_min_ps(Vec4::kOne, accum));
}

// Calculates the Taylor series sine of the input vector
Vec4 VecSin(Vec4& in)
{
	Vec4 v = in - Vec4(1.5707964f);
	return VecCos(v);
}

Vec4 VecDistanceBetween(Vec4 const& a, Vec4 const& b)
{
	Vec4 diff = VecSub(a, b);
	Vec4 dot  = VecDot3(diff, diff);
	return dot;
}

#pragma endregion Vector Math