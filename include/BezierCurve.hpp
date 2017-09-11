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

	/*
	後でちゃんと実装する
	*/
	double closestPoint(const Vec2& pos)const
	{
		double minT = 0.0;
		double minDistanceSq = get(minT).distanceFromSq(pos);

		const int divNum = 1000;
		for (int i = 0; i < divNum; ++i)
		{
			const double t = 1.0*i / (divNum - 1);
			const double distanceSq = get(t).distanceFromSq(pos);
			if (distanceSq < minDistanceSq)
			{
				minDistanceSq = distanceSq;
				minT = t;
			}
		}

		return minT;
	}

	Optional<double> closestPointOpt(const Vec2& pos, double threshold)const
	{
		double minT = 0.0;
		double minDistanceSq = get(minT).distanceFromSq(pos);

		const int divNum = 10000;
		for (int i = 0; i < divNum; ++i)
		{
			const double t = 1.0*i / (divNum - 1);
			const double distanceSq = get(t).distanceFromSq(pos);
			if (distanceSq < minDistanceSq)
			{
				minDistanceSq = distanceSq;
				minT = t;
			}
		}

		if (threshold*threshold < minDistanceSq)
		{
			return none;
		}

		return minT;
	}

	double length(double startT, double endT)const
	{
		/*
		Sqrt(Power(-3 * (1 - t)*(1 - t)*x1 + 3 * (1 - t)*(1 - t)*x2 - 6 * (1 - t)*t*x2 + 6 * (1 - t)*t*x3 - 3 * t*t*x3 + 3 * t*t*x4, 2) +
			Power(-3 * (1 - t)*(1 - t)*y1 + 3 * (1 - t)*(1 - t)*y2 - 6 * (1 - t)*t*y2 + 6 * (1 - t)*t*y3 - 3 * t*t*y3 + 3 * t*t*y4, 2))
			*/
		const auto df = [&](double t)
		{
			const double xx = -3 * (1 - t)*(1 - t)*p0.x + 3 * (1 - t)*(1 - t)*p1.x - 6 * (1 - t)*t*p1.x + 6 * (1 - t)*t*p2.x - 3 * t*t*p2.x + 3 * t*t*p3.x;
			const double yy = -3 * (1 - t)*(1 - t)*p0.y + 3 * (1 - t)*(1 - t)*p1.y - 6 * (1 - t)*t*p1.y + 6 * (1 - t)*t*p2.y - 3 * t*t*p2.y + 3 * t*t*p3.y;
			return sqrt(xx*xx + yy*yy);
		};

		double sum = 0.0;

		const double stepSize = 0.01;
		const int divNum = static_cast<int>((endT - startT) / stepSize);
		int i = 0;
		for (; i < divNum; ++i)
		{
			sum += stepSize*df(startT + stepSize*i);
		}
		sum += ((endT - startT) - stepSize*i)*df(endT);

		return sum;
	}

	void draw(int divNum = 30, const Color& color = Palette::White, double startT = 0.0, double endT = 1.0)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(get(Lerp(startT, endT, progress0)), get(Lerp(startT, endT, progress1))).draw(color);
		}
	}

	void write(Image& image, int divNum = 30, const Color& color = Palette::White, double startT = 0.0, double endT = 1.0)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(get(Lerp(startT, endT, progress0)), get(Lerp(startT, endT, progress1))).write(image, color);
		}
	}

	void drawArrow(int divNum = 30, const Color& color = Palette::White, double startT = 0.0, double endT = 1.0)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(get(Lerp(startT, endT, progress0)), get(Lerp(startT, endT, progress1))).drawArrow(1.0, {5.0,5.0}, color);
		}
	}

	void writeArrow(Image& image,int divNum = 30, const Color& color = Palette::White, double startT = 0.0, double endT = 1.0)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(get(Lerp(startT, endT, progress0)), get(Lerp(startT, endT, progress1))).writeArrow(image, 1.0, { 5.0,5.0 }, color);
		}
	}


	Vec2 p0, p1, p2, p3;
};