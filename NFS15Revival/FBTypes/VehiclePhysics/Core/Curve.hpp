#pragma once
#include <xmmintrin.h>
#include <math/vectormath.h>
#include <algorithm>

struct Curve
{
	float xMin;
	float xMax;
	float yMin;
	float yMax;
	float graphScale;
	float X[8];
	float Y[8];
	float Y2[8];
};

class PointGraphEval
{
	friend struct PointGraph8;

	static float Evaluate(const int numPoints, const float x[], const float y[], const float xVal);
};

struct PointGraph8
{
	enum
	{
		kNumPoints = 8
	};

	float min_x;
	float min_y;
	float max_x;
	float max_y;
	float x[kNumPoints];
	float y[kNumPoints];

	float Evaluate(float xVal) { return PointGraphEval::Evaluate(kNumPoints, x, y, xVal); }

	void operator=(PointGraph8& pg)
	{
		min_x = pg.min_x;
		min_y = pg.min_y;
		max_x = pg.max_x;
		max_y = pg.max_y;
		*reinterpret_cast<__m128*>(x) = *reinterpret_cast<__m128*>(pg.x);
		*reinterpret_cast<__m128*>(&x[4]) = *reinterpret_cast<__m128*>(&pg.x[4]);
		*reinterpret_cast<__m128*>(y) = *reinterpret_cast<__m128*>(pg.y);
		*reinterpret_cast<__m128*>(&y[4]) = *reinterpret_cast<__m128*>(&pg.y[4]);
	}
};

// Creates a PointGraph from a 4D vector array. The original arrays from the vehicle configs are never greater or less than 10 members.
// First two members of array represent min and max x/y values respectively; all following indices are for each x/y curve point, x being the input, and y being the output.
PointGraph8& initPointGraph8FromCurveData(PointGraph8& outGraph, const vec4 curveData[]);