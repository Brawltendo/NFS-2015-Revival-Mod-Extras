#pragma once
#include <emmintrin.h>
#include <stdint.h>


#pragma region __m128 Operator Overloads

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
_forceinline __m128 operator +  (const __m128& l, const float& r) { return __m128(_mm_add_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator -  (const __m128& l, const float& r) { return __m128(_mm_sub_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator *  (const __m128& l, const float& r) { return __m128(_mm_mul_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator /  (const __m128& l, const float& r) { return __m128(_mm_div_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator &  (const __m128& l, const float& r) { return __m128(_mm_and_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator |  (const __m128& l, const float& r) { return __m128(_mm_or_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator ^  (const __m128& l, const float& r) { return __m128(_mm_xor_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator <  (const __m128& l, const float& r) { return __m128(_mm_cmplt_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator >  (const __m128& l, const float& r) { return __m128(_mm_cmpgt_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator <= (const __m128& l, const float& r) { return __m128(_mm_cmple_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator >= (const __m128& l, const float& r) { return __m128(_mm_cmpge_ps(l, _mm_set1_ps(r))); }
_forceinline __m128 operator == (const __m128& l, const float& r) { return __m128(_mm_cmpeq_ps(l, _mm_set1_ps(r))); }

#pragma endregion __m128 Operator Overloads



#pragma region Vector Swizzling

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

enum EVecSwizzle
{
	k_xxxx = 0, k_xxxy = 64, k_xxxz = 128, k_xxxw = 192,
	k_xxyx = 16, k_xxyy = 80, k_xxyz = 144, k_xxyw = 208,
	k_xxzx = 32, k_xxzy = 96, k_xxzz = 160, k_xxzw = 224,
	k_xxwx = 48, k_xxwy = 112, k_xxwz = 176, k_xxww = 240,
	k_xyxx = 4, k_xyxy = 68, k_xyxz = 132, k_xyxw = 196,
	k_xyyx = 20, k_xyyy = 84, k_xyyz = 148, k_xyyw = 212,
	k_xyzx = 36, k_xyzy = 100, k_xyzz = 164, k_xyzw = 228,
	k_xywx = 52, k_xywy = 116, k_xywz = 180, k_xyww = 244,
	k_xzxx = 8, k_xzxy = 72, k_xzxz = 136, k_xzxw = 200,
	k_xzyx = 24, k_xzyy = 88, k_xzyz = 152, k_xzyw = 216,
	k_xzzx = 40, k_xzzy = 104, k_xzzz = 168, k_xzzw = 232,
	k_xzwx = 56, k_xzwy = 120, k_xzwz = 184, k_xzww = 248,
	k_xwxx = 12, k_xwxy = 76, k_xwxz = 140, k_xwxw = 204,
	k_xwyx = 28, k_xwyy = 92, k_xwyz = 156, k_xwyw = 220,
	k_xwzx = 44, k_xwzy = 108, k_xwzz = 172, k_xwzw = 236,
	k_xwwx = 60, k_xwwy = 124, k_xwwz = 188, k_xwww = 252,
	k_yxxx = 1, k_yxxy = 65, k_yxxz = 129, k_yxxw = 193,
	k_yxyx = 17, k_yxyy = 81, k_yxyz = 145, k_yxyw = 209,
	k_yxzx = 33, k_yxzy = 97, k_yxzz = 161, k_yxzw = 225,
	k_yxwx = 49, k_yxwy = 113, k_yxwz = 177, k_yxww = 241,
	k_yyxx = 5, k_yyxy = 69, k_yyxz = 133, k_yyxw = 197,
	k_yyyx = 21, k_yyyy = 85, k_yyyz = 149, k_yyyw = 213,
	k_yyzx = 37, k_yyzy = 101, k_yyzz = 165, k_yyzw = 229,
	k_yywx = 53, k_yywy = 117, k_yywz = 181, k_yyww = 245,
	k_yzxx = 9, k_yzxy = 73, k_yzxz = 137, k_yzxw = 201,
	k_yzyx = 25, k_yzyy = 89, k_yzyz = 153, k_yzyw = 217,
	k_yzzx = 41, k_yzzy = 105, k_yzzz = 169, k_yzzw = 233,
	k_yzwx = 57, k_yzwy = 121, k_yzwz = 185, k_yzww = 249,
	k_ywxx = 13, k_ywxy = 77, k_ywxz = 141, k_ywxw = 205,
	k_ywyx = 29, k_ywyy = 93, k_ywyz = 157, k_ywyw = 221,
	k_ywzx = 45, k_ywzy = 109, k_ywzz = 173, k_ywzw = 237,
	k_ywwx = 61, k_ywwy = 125, k_ywwz = 189, k_ywww = 253,
	k_zxxx = 2, k_zxxy = 66, k_zxxz = 130, k_zxxw = 194,
	k_zxyx = 18, k_zxyy = 82, k_zxyz = 146, k_zxyw = 210,
	k_zxzx = 34, k_zxzy = 98, k_zxzz = 162, k_zxzw = 226,
	k_zxwx = 50, k_zxwy = 114, k_zxwz = 178, k_zxww = 242,
	k_zyxx = 6, k_zyxy = 70, k_zyxz = 134, k_zyxw = 198,
	k_zyyx = 22, k_zyyy = 86, k_zyyz = 150, k_zyyw = 214,
	k_zyzx = 38, k_zyzy = 102, k_zyzz = 166, k_zyzw = 230,
	k_zywx = 54, k_zywy = 118, k_zywz = 182, k_zyww = 246,
	k_zzxx = 10, k_zzxy = 74, k_zzxz = 138, k_zzxw = 202,
	k_zzyx = 26, k_zzyy = 90, k_zzyz = 154, k_zzyw = 218,
	k_zzzx = 42, k_zzzy = 106, k_zzzz = 170, k_zzzw = 234,
	k_zzwx = 58, k_zzwy = 122, k_zzwz = 186, k_zzww = 250,
	k_zwxx = 14, k_zwxy = 78, k_zwxz = 142, k_zwxw = 206,
	k_zwyx = 30, k_zwyy = 94, k_zwyz = 158, k_zwyw = 222,
	k_zwzx = 46, k_zwzy = 110, k_zwzz = 174, k_zwzw = 238,
	k_zwwx = 62, k_zwwy = 126, k_zwwz = 190, k_zwww = 254,
	k_wxxx = 3, k_wxxy = 67, k_wxxz = 131, k_wxxw = 195,
	k_wxyx = 19, k_wxyy = 83, k_wxyz = 147, k_wxyw = 211,
	k_wxzx = 35, k_wxzy = 99, k_wxzz = 163, k_wxzw = 227,
	k_wxwx = 51, k_wxwy = 115, k_wxwz = 179, k_wxww = 243,
	k_wyxx = 7, k_wyxy = 71, k_wyxz = 135, k_wyxw = 199,
	k_wyyx = 23, k_wyyy = 87, k_wyyz = 151, k_wyyw = 215,
	k_wyzx = 39, k_wyzy = 103, k_wyzz = 167, k_wyzw = 231,
	k_wywx = 55, k_wywy = 119, k_wywz = 183, k_wyww = 247,
	k_wzxx = 11, k_wzxy = 75, k_wzxz = 139, k_wzxw = 203,
	k_wzyx = 27, k_wzyy = 91, k_wzyz = 155, k_wzyw = 219,
	k_wzzx = 43, k_wzzy = 107, k_wzzz = 171, k_wzzw = 235,
	k_wzwx = 59, k_wzwy = 123, k_wzwz = 187, k_wzww = 251,
	k_wwxx = 15, k_wwxy = 79, k_wwxz = 143, k_wwxw = 207,
	k_wwyx = 31, k_wwyy = 95, k_wwyz = 159, k_wwyw = 223,
	k_wwzx = 47, k_wwzy = 111, k_wwzz = 175, k_wwzw = 239,
	k_wwwx = 63, k_wwwy = 127, k_wwwz = 191, k_wwww = 255
};

#pragma endregion Vector Swizzling



struct Vec4
{
	static const Vec4 kZero;
	static const Vec4 kQuarter;
	static const Vec4 kHalf;
	static const Vec4 kOne;
	static const Vec4 kMinusOne;
	static const Vec4 kEpsilon;
	static const Vec4 kUnitX;
	static const Vec4 kUnitY;
	static const Vec4 kUnitZ;
	static const Vec4 kUnitW;

	union
	{
		struct
		{
			float x, y, z, w;
		};
		__m128 simdValue;
	};

	// Initializes this Vec4 to zero
	Vec4()
	{
		simdValue = _mm_setzero_ps();
	}

	// Initializes all members of this vector with the value of in
	Vec4(float in)
	{
		simdValue = _mm_set1_ps(in);
	}

	// Initializes xyz and sets w to zero
	Vec4(float inX, float inY, float inZ)
	{
		simdValue = _mm_setr_ps(inX, inY, inZ, 0.f);
	}

	// Initializes xyzw
	Vec4(float inX, float inY, float inZ, float inW)
	{
		simdValue = _mm_setr_ps(inX, inY, inZ, inW);
	}

	// Initializes xyzw with the values of another Vec4
	Vec4(const Vec4& v)
	{
		simdValue = v.simdValue;
	}

	// Initializes this Vec4 with a SIMD value
	Vec4(const __m128& simdIn)
	{
		simdValue = simdIn;
	}

	float& operator[](int i)
	{
		return simdValue.m128_f32[i];
	}

	Vec4  operator +  (const Vec4& b) { return simdValue + b.simdValue; }

	Vec4  operator +  (const Vec4& b) const { return simdValue + b.simdValue; }

	Vec4& operator += (const Vec4& b) { simdValue = simdValue + b.simdValue; return *this; }

	Vec4  operator -  (const Vec4& b) { return simdValue - b.simdValue; }

	Vec4  operator -  (const Vec4& b) const { return simdValue - b.simdValue; }

	Vec4& operator -= (const Vec4& b) { simdValue = simdValue - b.simdValue; return *this; }

	Vec4  operator *  (const Vec4& b) { return simdValue * b.simdValue; }

	Vec4  operator *  (const Vec4& b) const { return simdValue * b.simdValue; }

	Vec4& operator *= (const Vec4& b) { simdValue = simdValue * b.simdValue; return *this; }

	Vec4  operator /  (const Vec4& b) { return simdValue / b.simdValue; }

	Vec4  operator /  (const Vec4& b) const { return simdValue / b.simdValue; }

	Vec4& operator /= (const Vec4& b) { simdValue = simdValue / b.simdValue; return *this; }

	operator __m128() { return simdValue; }

	operator __m128() const { return simdValue; }

};



#pragma region Vector Comparisons

#define CmpMask(x,y,z,w) (x | (y << 1) | (z << 2) | (w << 3))

// Returns if the input mask is true or false (all members)
bool VecIsTrue(const Vec4& v);

// Returns if the input mask is true or false (specified members)
// Use CmpMask to create the input mask
bool VecIsTrue(const Vec4& v, uint8_t cmpMask);

// Returns if the input mask is true or false (any members)
bool VecIsTrueAny(const Vec4& v);

// Returns true if a < b (all members)
bool VecCmpLT(const Vec4& a, const Vec4& b);

// Returns true if a < b (specified members)
// Use CmpMask to create the input mask
bool VecCmpLT(const Vec4& a, const Vec4& b, uint8_t cmpMask);

// Returns true if a < b (any members)
bool VecCmpLTAny(const Vec4& a, const Vec4& b);

// Returns true if a <= b (all members)
bool VecCmpLE(const Vec4& a, const Vec4& b);

// Returns true if a <= b (specified members)
// Use CmpMask to create the input mask
bool VecCmpLE(const Vec4& a, const Vec4& b, uint8_t cmpMask);

// Returns true if a <= b (any members)
bool VecCmpLEAny(const Vec4& a, const Vec4& b);

// Returns true if a > b (all members)
bool VecCmpGT(const Vec4& a, const Vec4& b);

// Returns true if a > b (specified members)
// Use CmpMask to create the input mask
bool VecCmpGT(const Vec4& a, const Vec4& b, uint8_t cmpMask);

// Returns true if a > b (any members)
bool VecCmpGTAny(const Vec4& a, const Vec4& b);
// Returns true if a >= b (all members)
bool VecCmpGE(const Vec4& a, const Vec4& b);

// Returns true if a >= b (specified members)
// Use CmpMask to create the input mask
bool VecCmpGE(const Vec4& a, const Vec4& b, uint8_t cmpMask);

// Returns true if a >= b (any members)
bool VecCmpGEAny(const Vec4& a, const Vec4& b);

// Returns true if a == b (all members)
bool VecCmpEQ(const Vec4& a, const Vec4& b);

// Returns true if a == b (specified members)
// Use CmpMask to create the input mask
bool VecCmpEQ(const Vec4& a, const Vec4& b, uint8_t cmpMask);
// Returns true if a == b (any members)
bool VecCmpEQAny(const Vec4& a, const Vec4& b);

// Returns true if a != b (all members)
bool VecCmpNEQ(const Vec4& a, const Vec4& b);

// Returns true if a != b (specified members)
// Use CmpMask to create the input mask
bool VecCmpNEQ(const Vec4& a, const Vec4& b, uint8_t cmpMask);

// Returns true if a != b (any members)
bool VecCmpNEQAny(const Vec4& a, const Vec4& b);

#pragma endregion Vector Comparisons



#pragma region Math Operations

// Adds 2 vectors
Vec4 VecAdd(const Vec4& l, const Vec4& r);

// Subtracts 2 vectors
Vec4 VecSub(const Vec4& l, const Vec4& r);

// Multiplies 2 vectors
Vec4 VecMul(const Vec4& l, const Vec4& r);

// Divides 2 vectors
Vec4 VecDiv(const Vec4& l, const Vec4& r);

#pragma endregion Math Operations



#pragma region Logical Operations

Vec4 VecAnd(const Vec4& l, const Vec4& r);

Vec4 VecAndNot(const Vec4& l, const Vec4& r);

Vec4 VecOr(const Vec4& l, const Vec4& r);

Vec4 VecXor(const Vec4& l, const Vec4& r);

Vec4 VecNot(const Vec4& v);

#pragma endregion Logical Operations



#pragma region Misc

Vec4& SimdToVec4(const __m128& simd);

// Converts a bool input to an equivalent vector mask
Vec4 BoolToVecMask(bool condition);

// Returns a vector based on the input condition
Vec4 VecSelect(const Vec4& valIfTrue, const Vec4& valIfFalse, const Vec4& condition);

// Returns a vector based on the input condition
__m128 SelectVector(const __m128& valIfTrue, const __m128& valIfFalse, const __m128& condition);

// Returns the minimum of 2 vectors
Vec4 VecMin(const Vec4& a, const Vec4& b);

// Returns the maximum of 2 vectors
Vec4 VecMax(const Vec4& a, const Vec4& b);

// Clamps the input vector between a min and max vector
Vec4 VecClamp(const Vec4& v, const Vec4& min, const Vec4& max);

// Returns the absolute value of the input vector
Vec4 VecAbs(Vec4& v);

// Returns the absolute value of the input vector
Vec4 VecAbs(const Vec4& v);

// Linearly interpolates between a & b using interpolator t
Vec4 VecLerp(const Vec4& a, const Vec4& b, const Vec4& t);

// Returns a [0,1] interpolator from a given input and range
Vec4 VecRamp(const Vec4& val, const Vec4& low, const Vec4& high);

// Returns the sign of the input vector
Vec4 VecSign(const Vec4& v);

// Negates the input vector
Vec4 VecNeg(const Vec4& v);

#pragma endregion Misc



#pragma region Vector Math

// Returns the reciprocal of the input vector
Vec4 VecRecip(Vec4& v);

// Performs the dot product of 2 vectors
float Dot3(Vec4 const& a, Vec4 const& b);

// Performs the dot product of 2 vectors
Vec4 VecDot3(Vec4 const& a, Vec4 const& b);

// Returns the length of the input vector
Vec4 VecLength3(Vec4& v);

// Normalizes the input vector
float VecNormal3(Vec4& v);

// Performs the cross product of 2 vectors
Vec4 Cross(Vec4 const& a, Vec4 const& b);

// Calculates the Taylor series cosine of the input vector
Vec4 VecCos(Vec4& in);

// Calculates the Taylor series sine of the input vector
Vec4 VecSin(Vec4& in);

Vec4 VecDistanceBetween(Vec4 const& a, Vec4 const& b);

#pragma endregion Vector Math
