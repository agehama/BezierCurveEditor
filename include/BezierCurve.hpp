#pragma once
#include <Siv3D.hpp>

extern int64 profilingTime1;
extern int64 profilingTime2;

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
		//return Clamp(solveClosestPoint(pos, p0, p1, p2, p3, 0.5), 0.0, 1.0);

		/*const auto ps = solveClosestPoint(pos, p0, p1, p2, p3);

		const double fromCenter1 = Abs(ps.first - 0.5);
		const double fromCenter2 = Abs(ps.second - 0.5);
		return Clamp(fromCenter1 < fromCenter2 ? ps.first : ps.second, 0.0, 1.0);*/
		
		{
			StopwatchMicrosec watch(true);

			double minT = 0.0;
			double minDistanceSq = get(minT).distanceFromSq(pos);

			const int divNum = 500;
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

			profilingTime1 += watch.us();

			return minT;
		}
		
	}

	Optional<double> closestPointOpt(const Vec2& pos, double threshold, double startT = 0.0, double endT = 1.0)const
	{
		//const double t = Clamp(solveClosestPoint(pos, p0, p1, p2, p3, 0.5), Min(startT, endT), Max(startT, endT));

		/*
		const double m = (startT + endT)*0.5;
		const auto ts = solveClosestPoint(pos, p0, p1, p2, p3);
		const double fromCenter1 = Abs(ts.first - m);
		const double fromCenter2 = Abs(ts.second - m);
		const double t = Clamp(fromCenter1 < fromCenter2 ? ts.first : ts.second, Min(startT, endT), Max(startT, endT));
		
		const double distanceSq = get(t).distanceFromSq(pos);

		if (threshold*threshold < distanceSq)
		{
			return none;
		}

		return t;
		*/
		
		{
			StopwatchMicrosec watch(true);

			double minT = 0.0;
			double minDistanceSq = get(minT).distanceFromSq(pos);

			const int divNum = 500;
			for (int i = 0; i < divNum; ++i)
			{
				const double t = startT + (endT - startT)*i / (divNum - 1);
				const double distanceSq = get(t).distanceFromSq(pos);
				if (distanceSq < minDistanceSq)
				{
					minDistanceSq = distanceSq;
					minT = t;
				}
			}

			profilingTime1 += watch.us();

			if (threshold*threshold < minDistanceSq)
			{
				return none;
			}

			return minT;
		}
	}
	
	/*
	全体(length): 18ms
	*/
	double length(double startT, double endT)const
	{
		StopwatchMicrosec watch(true);

		const auto df = [&](double t)
		{
			const double xx = -3 * (1 - t)*(1 - t)*p0.x + 3 * (1 - t)*(1 - t)*p1.x - 6 * (1 - t)*t*p1.x + 6 * (1 - t)*t*p2.x - 3 * t*t*p2.x + 3 * t*t*p3.x;
			const double yy = -3 * (1 - t)*(1 - t)*p0.y + 3 * (1 - t)*(1 - t)*p1.y - 6 * (1 - t)*t*p1.y + 6 * (1 - t)*t*p2.y - 3 * t*t*p2.y + 3 * t*t*p3.y;
			return sqrt(xx*xx + yy*yy);
		};

		double sum = 0.0;

		//const double stepSize = 0.01;
		const double stepSize = 0.1;
		const int divNum = static_cast<int>((endT - startT) / stepSize);
		int i = 0;
		for (; i < divNum; ++i)
		{
			sum += stepSize*df(startT + stepSize*i);
		}
		sum += ((endT - startT) - stepSize*i)*df(endT);

		profilingTime2 += watch.us();

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

	void writeArrow(Image& image, int divNum = 30, const Color& color = Palette::White, double startT = 0.0, double endT = 1.0)const
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