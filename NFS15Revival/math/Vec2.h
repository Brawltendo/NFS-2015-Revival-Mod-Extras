#pragma once


struct Vec2
{
	static const Vec2 kZero;

	union
	{
		struct
		{
			float x, y;
		};
		float arr[2];
	};
	

	Vec2()
	{
		x = 0.f;
		y = 0.f;
	}

	float& operator[](int i)
	{
		return arr[i];
	}

	Vec2(const float f)
	{
		x = f;
		y = f;
	}

	Vec2(const float fx, const float fy)
	{
		x = fx;
		y = fy;
	}

	Vec2 operator+(const Vec2& b)
	{
		Vec2 v;
		v.x = this->x + b.x;
		v.y = this->y + b.y;
		return v;
	}

	Vec2 operator+(const float b)
	{
		Vec2 v;
		v.x = this->x + b;
		v.y = this->y + b;
		return v;
	}

	Vec2 operator-(const Vec2& b)
	{
		Vec2 v;
		v.x = this->x - b.x;
		v.y = this->y - b.y;
		return v;
	}

	Vec2 operator-(const float b)
	{
		Vec2 v;
		v.x = this->x - b;
		v.y = this->y - b;
		return v;
	}

	Vec2 operator*(const Vec2& b)
	{
		Vec2 v;
		v.x = this->x * b.x;
		v.y = this->y * b.y;
		return v;
	}

	Vec2 operator*(const float b)
	{
		Vec2 v;
		v.x = this->x * b;
		v.y = this->y * b;
		return v;
	}

	Vec2 operator/(const Vec2& b)
	{
		Vec2 v;
		v.x = this->x / b.x;
		v.y = this->y / b.y;
		return v;
	}

	Vec2 operator/(const float b)
	{
		Vec2 v;
		v.x = this->x / b;
		v.y = this->y / b;
		return v;
	}

	Vec2 operator=(const float b)
	{
		Vec2 v;
		v.x = b;
		v.y = b;
		return v;
	}

};