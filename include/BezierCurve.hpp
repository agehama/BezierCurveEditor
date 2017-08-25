#pragma once
#include <Siv3D.hpp>

/*
三次ベジェ曲線のクラス
*/
struct BezierCurve
{
	BezierCurve() = default;

	BezierCurve(const Vec2& p0_, const Vec2& p1_, const Vec2& p2_, const Vec2& p3_) :
		p0(p0_),
		p1(p1_),
		p2(p2_),
		p3(p3_)
	{}

	Vec2 get(double t)const
	{
		return (1 - t)*(1 - t)*(1 - t)*p0 + 3.0*(1 - t)*(1 - t)*t*p1 + 3.0*(1 - t)*t*t*p2 + t*t*t*p3;
	}

	Vec2 operator()(double t)const
	{
		return get(t);
	}

	Vec2 p0, p1, p2, p3;
};