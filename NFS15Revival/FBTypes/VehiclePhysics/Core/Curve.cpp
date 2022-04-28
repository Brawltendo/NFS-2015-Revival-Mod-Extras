#include "Curve.hpp"


float PointGraphEval::Evaluate(const int numPoints, const float x[], const float y[], const float xVal)
{
	if (xVal >= x[0] && xVal >= x[numPoints - 1])
		return y[numPoints - 1];

	if (numPoints > 1)
	{
		int i = 1;
		for (;;)
		{
			if (xVal >= x[i])
			{
				++i;
				if (i < numPoints)
					continue;
				else
					return y[0];
			}
			break;
		}

		int prevInd = i - 1;
		float deltaX = x[i] - x[prevInd];
		float deltaY = y[i] - y[prevInd];
		if (deltaX > 0.f)
			return ((deltaY / deltaX) * (xVal - x[prevInd])) + y[prevInd];
		else
			return y[i];
	}
	return y[0];
}

PointGraph8& initPointGraph8FromCurveData(PointGraph8& outGraph, const vec4 curveData[])
{
	const float xMin = curveData[0].x;
	const float yMin = curveData[0].y;
	const float xMax = curveData[1].x;
	const float yMax = curveData[1].y;

	outGraph.min_x = xMin;
	outGraph.max_x = xMax;
	outGraph.min_y = yMin;
	outGraph.max_y = yMax;

	float spanX = xMax - xMin;
	float spanY = yMax - yMin;
	const vec4* inCurve = &curveData[2];
	// inverse lerp over the set of data
	for (int i = 0; i < PointGraph8::kNumPoints; ++i)
	{
		outGraph.x[i] = spanX * inCurve[i].x + outGraph.min_x;
		outGraph.y[i] = spanY * inCurve[i].y + outGraph.min_y;
	}
	return outGraph;
}
