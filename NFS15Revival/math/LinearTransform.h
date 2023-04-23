#pragma once
#include <math/Vec4.h>


class LinearTransform
{
public:
	Vec4 left;
	Vec4 up;
	Vec4 forward;
	Vec4 trans;
};
static_assert(sizeof(LinearTransform) == 0x40, "LinearTransform");