#pragma once
#include <xmmintrin.h>
#include <emmintrin.h>
#include <intrin.h>

_forceinline __m128 operator +  (const __m128& l, const __m128& r) { return __m128(_mm_add_ps(l, r)); }
_forceinline __m128 operator -  (const __m128& l, const __m128& r) { return __m128(_mm_sub_ps(l, r)); }
_forceinline __m128 operator *  (const __m128& l, const __m128& r) { return __m128(_mm_mul_ps(l, r)); }
_forceinline __m128 operator /  (const __m128& l, const __m128& r) { return __m128(_mm_div_ps(l, r)); }
_forceinline __m128 operator &  (const __m128& l, const __m128& r) { return __m128(_mm_and_ps(l, r)); }
_forceinline __m128 operator |  (const __m128& l, const __m128& r) { return __m128(_mm_or_ps(l, r)); }
_forceinline __m128 operator ^  (const __m128& l, const __m128& r) { return __m128(_mm_xor_ps(l, r)); }
_forceinline __m128 operator <  (const __m128& l, const __m128& r) { return __m128(_mm_cmplt_ps(l, r)); }
_forceinline __m128 operator >  (const __m128& l, const __m128& r) { return __m128(_mm_cmpgt_ps(l, r)); }
_forceinline __m128 operator <= (const __m128& l, const __m128& r) { return __m128(_mm_cmple_ps(l, r)); }
_forceinline __m128 operator >= (const __m128& l, const __m128& r) { return __m128(_mm_cmpge_ps(l, r)); }
_forceinline __m128 operator == (const __m128& l, const __m128& r) { return __m128(_mm_cmpeq_ps(l, r)); }
_forceinline __m128 operator +  (const __m128& l, const float& r)  { return __m128(_mm_add_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator -  (const __m128& l, const float& r)  { return __m128(_mm_sub_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator *  (const __m128& l, const float& r)  { return __m128(_mm_mul_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator /  (const __m128& l, const float& r)  { return __m128(_mm_div_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator &  (const __m128& l, const float& r)  { return __m128(_mm_and_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator |  (const __m128& l, const float& r)  { return __m128(_mm_or_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator ^  (const __m128& l, const float& r)  { return __m128(_mm_xor_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator <  (const __m128& l, const float& r)  { return __m128(_mm_cmplt_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator >  (const __m128& l, const float& r)  { return __m128(_mm_cmpgt_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator <= (const __m128& l, const float& r)  { return __m128(_mm_cmple_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator >= (const __m128& l, const float& r)  { return __m128(_mm_cmpge_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator == (const __m128& l, const float& r)  { return __m128(_mm_cmpeq_ps(l, _mm_set1_ps(r))); }

#define MakeShuffleMask(x,y,z,w)           (x | (y<<2) | (z<<4) | (w<<6))

// vec(0, 1, 2, 3) -> (vec[x], vec[y], vec[z], vec[w])
#define VecSwizzleMask(vec, mask)          _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(vec), mask))
#define VecSwizzle(vec, x, y, z, w)        VecSwizzleMask(vec, MakeShuffleMask(x,y,z,w))
#define VecSwizzle1(vec, x)                VecSwizzleMask(vec, MakeShuffleMask(x,x,x,x))
// special swizzle
#define VecSwizzle_0022(vec)               _mm_moveldup_ps(vec)
#define VecSwizzle_1133(vec)               _mm_movehdup_ps(vec)

// return (vec1[x], vec1[y], vec2[z], vec2[w])
#define VecShuffle(v1, v2, x,y,z,w)    _mm_shuffle_ps(v1, v2, MakeShuffleMask(x,y,z,w))
// special shuffle
#define VecShuffle_0101(v1, v2)        _mm_movelh_ps(v1, v2)
#define VecShuffle_2323(v1, v2)        _mm_movehl_ps(v2, v1)

namespace
{

struct vec2
{
	static const vec2 kZero;

	float x, y;

	vec2()
	{
		x = 0.f;
		y = 0.f;
	}

	float& operator[](int index)
	{
		return (&x)[index];
	}

	vec2(const float f)
	{
		x = f;
		y = f;
	}

	vec2(const float fx, const float fy)
	{
		x = fx;
		y = fy;
	}

	vec2 operator+(const vec2& b)
	{
		vec2 v;
		v.x = this->x + b.x;
		v.y = this->y + b.y;
		return v;
	}

	vec2 operator+(const float b)
	{
		vec2 v;
		v.x = this->x + b;
		v.y = this->y + b;
		return v;
	}

	vec2 operator-(const vec2& b)
	{
		vec2 v;
		v.x = this->x - b.x;
		v.y = this->y - b.y;
		return v;
	}

	vec2 operator-(const float b)
	{
		vec2 v;
		v.x = this->x - b;
		v.y = this->y - b;
		return v;
	}

	vec2 operator*(const vec2& b)
	{
		vec2 v;
		v.x = this->x * b.x;
		v.y = this->y * b.y;
		return v;
	}

	vec2 operator*(const float b)
	{
		vec2 v;
		v.x = this->x * b;
		v.y = this->y * b;
		return v;
	}

	vec2 operator/(const vec2& b)
	{
		vec2 v;
		v.x = this->x / b.x;
		v.y = this->y / b.y;
		return v;
	}

	vec2 operator/(const float b)
	{
		vec2 v;
		v.x = this->x / b;
		v.y = this->y / b;
		return v;
	}

	vec2 operator=(const float b)
	{
		vec2 v;
		v.x = b;
		v.y = b;
		return v;
	}

};

const vec2 vec2::kZero = (0.0f);

struct vec4
{
	static const vec4 s_Zero;
	static const vec4 s_Half;
	static const vec4 s_One;
	static const vec4 s_Right;
	static const vec4 s_Up;
	static const vec4 s_Forward;

	union
	{
		struct
		{
			float x, y, z, w;
		};
		__m128 simdValue;
	};

	// Initializes this vec4 to zero
	vec4()
	{
		simdValue = _mm_setzero_ps();
	}

	// Initializes all members of this vector with the value of in
	vec4(float in)
	{
		simdValue = _mm_set1_ps(in);
	}

	// Initializes xyz and sets w to zero
	vec4(float inX, float inY, float inZ)
	{
		simdValue = _mm_setr_ps(inX, inY, inZ, 0.f);
	}

	// Initializes xyzw
	vec4(float inX, float inY, float inZ, float inW)
	{
		simdValue = _mm_setr_ps(inX, inY, inZ, inW);
	}

	// Initializes xyzw with the values of another vec4
	vec4(const vec4& v)
	{
		simdValue = v.simdValue;
	}

	// Initializes this vec4 with a SIMD value
	vec4(const __m128& simdIn)
	{
		simdValue = simdIn;
	}

	float& operator[](int i)
	{
		return (&x)[i];
	}

	vec4  operator +  (const vec4& b) { return simdValue + b.simdValue; }

	vec4  operator +  (const vec4& b) const { return simdValue + b.simdValue; }

	vec4& operator += (const vec4& b) { simdValue = simdValue + b.simdValue; return *this; }

	vec4  operator -  (const vec4& b) { return simdValue - b.simdValue; }

	vec4  operator -  (const vec4& b) const { return simdValue - b.simdValue; }

	vec4& operator -= (const vec4& b) { simdValue = simdValue - b.simdValue; return *this; }

	vec4  operator *  (const vec4& b) { return simdValue * b.simdValue; }

	vec4  operator *  (const vec4& b) const { return simdValue * b.simdValue; }

	vec4& operator *= (const vec4& b) { simdValue = simdValue * b.simdValue; return *this; }

	vec4  operator /  (const vec4& b) { return simdValue / b.simdValue; }

	vec4  operator /  (const vec4& b) const { return simdValue / b.simdValue; }

	vec4& operator /= (const vec4& b) { simdValue = simdValue / b.simdValue; return *this; }

};

const vec4 vec4::s_Zero(0.f);
const vec4 vec4::s_Half(0.5f);
const vec4 vec4::s_One(1.f);
const vec4 vec4::s_Right(1.f, 0.f, 1.f);
const vec4 vec4::s_Up(0.f, 1.f, 0.f);
const vec4 vec4::s_Forward(0.f, 0.f, 1.f);


vec4 VecAbs(vec4& v)
{
	return _mm_castsi128_ps(_mm_srli_epi32(_mm_slli_epi32(_mm_castps_si128(v.simdValue), 1), 1));
}

// Returns a vector based on the input condition
vec4 SelectVector(const vec4& valIfTrue, const vec4& valIfFalse, const vec4& condition)
{
	return _mm_xor_ps(_mm_and_ps(_mm_xor_ps(valIfTrue.simdValue, valIfFalse.simdValue), condition.simdValue), valIfFalse.simdValue);
}

// Returns a vector based on the input condition
__m128 SelectVector(const __m128& valIfTrue, const __m128& valIfFalse, const __m128& condition)
{
	return _mm_xor_ps(_mm_and_ps(_mm_xor_ps(valIfTrue, valIfFalse), condition), valIfFalse);
}

// Returns the reciprocal of the input vector
vec4 VecRecip(vec4& v)
{
	return _mm_rcp_ps(v.simdValue);
}

vec4& SimdToVec4(const __m128& simd)
{
	return *((vec4*)&simd);
}

// Performs the dot product of 2 vectors
float Dot(vec4& a, vec4& b)
{
	vec4 mul = a * b;
	vec4 dot = VecSwizzle(mul.simdValue, 1, 1, 1, 1) + VecSwizzle(mul.simdValue, 0, 0, 0, 0) + VecSwizzle(mul.simdValue, 2, 2, 2, 2);
	return _mm_cvtss_f32(dot.simdValue);
}

// Returns the magnitude of the input vector
float VecLength(vec4& v)
{
	vec4 dp = Dot(v, v);
	vec4 cond = dp.simdValue == vec4::s_Zero.simdValue;
	//__m128 sel   = _mm_or_ps(_mm_andnot_ps(cmp1, dp), _mm_and_ps(vec4::s_One.simdValue, cond));
	vec4 sel = SelectVector(vec4::s_One, dp, cond);
	vec4 rsqrt = _mm_rsqrt_ps(sel.simdValue);
	vec4 tmp1 = sel * rsqrt;
	//__m128 tmp2  = _mm_andnot_ps(cond, vec4::s_Half.simdValue * tmp1 * (vec4::s_One.simdValue - rsqrt * tmp1) + tmp1);
	//__m128 tmp3  = _mm_or_ps(tmp2, _mm_and_ps(vec4::s_Zero.simdValue, cond));
	vec4 tmp3 = SelectVector(vec4::s_Zero, vec4::s_Half * tmp1 * (vec4::s_One - rsqrt * tmp1) + tmp1, cond);
	return _mm_cvtss_f32(tmp3.simdValue);
}

// Performs the cross product of 2 vectors
vec4 Cross(vec4 const& a, vec4 const& b)
{
	vec4 tmp0 = _mm_shuffle_ps(a.simdValue, a.simdValue, _MM_SHUFFLE(3, 0, 2, 1));
	vec4 tmp1 = _mm_shuffle_ps(b.simdValue, b.simdValue, _MM_SHUFFLE(3, 1, 0, 2));
	vec4 tmp2 = tmp0 * b;
	vec4 tmp3 = tmp0 * tmp1;
	vec4 tmp4 = _mm_shuffle_ps(tmp2.simdValue, tmp2.simdValue, _MM_SHUFFLE(3, 0, 2, 1));
	return tmp3 - tmp4;
}

};
