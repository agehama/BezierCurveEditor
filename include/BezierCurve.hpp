#pragma once
#include <Siv3D.hpp>

//#include "cppoptlib/meta.h"
//#include "cppoptlib/problem.h"
//#include "cppoptlib/solver/bfgssolver.h"
//#include "cppoptlib/solver/newtondescentsolver.h"
//
//// nolintnextline
//using namespace cppoptlib;

//inline Optional<double> newtonMethod(std::function<double(double)> func, std::function<double(double)> dfunc, double initialX, double eps = 1.e-7, int maxTryCount = 300)
//{
//	double currentX = initialX;
//
//	//func(x) にニュートン法を実行する
//	for (int i = 0; i < maxTryCount; ++i)
//	{
//		double currentY = func(currentX);
//
//		//収束した
//		if (abs(currentY) < eps)
//		{
//			return currentX;
//		}
//
//		const double a = dfunc(currentX);
//		const double b = currentY - a*currentX;
//
//		//a*nextX + b = 0 を解いたものが次のx
//		//nextX = -b/a
//		currentX = -b / a;
//	}
//
//	return none;
//}

inline Optional<double> newtonMethod(std::function<double(double)> func, std::function<double(double)> dfunc, double initialX, double eps = 1.e-7, int maxTryCount = 300)
{
	double currentX = initialX;

	//func(x) にニュートン法を実行する
	for (int i = 0; i < maxTryCount; ++i)
	{
		double currentY = func(currentX);

		//収束した
		if (abs(currentY) < eps)
		{
			return currentX;
		}

		const double a = dfunc(currentX);
		const double b = currentY - a*currentX;

		const double nextX = currentX = -b / a;
		const double nextY = func(nextX);
		//極値に近づいてこれ以上収束が見込めない場合はすぐ帰る
		if (currentY < nextY)
		{
			//LOG_ERROR(L"return from newton method at iterations: ", i);
			//return currentX;
			return none;
		}

		//a*nextX + b = 0 を解いたものが次のx
		//nextX = -b/a
		currentX = -b / a;
	}

	return none;
}

//const auto func = [&](double t)
//{
//	const double dx = pos.x - p0.x*(1 - t)*(1 - t)*(1 - t) - 3.*p1.x*(1 - t)*(1 - t)*t - 3.*p2.x*(1 - t)*t*t - p3.x*t*t*t;
//	const double dy = pos.y - p0.y*(1 - t)*(1 - t)*(1 - t) - 3.*p1.y*(1 - t)*(1 - t)*t - 3.*p2.y*(1 - t)*t*t - p3.y*t*t*t;
//	return dx*dx + dy*dy;
//};

////func の微分
//const auto dfunc = [&](double t)
//{
//	return 6.*(pos.x + p0.x*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.x*(-1. + t)*(-1. + t)*t +
//		3.*p2.x*(-1. + t)*t*t - p3.x*t*t*t)*
//		(t*(-2.*p2.x + 3.*p2.x*t - 1.*p3.x*t) + p1.x*(-1. + 4.*t - 3.*t*t) +
//			p0.x*(1 - 2.*t + t*t)) +
//		6.*(pos.y + p0.y*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.y*(-1. + t)*(-1. + t)*t +
//			3.*p2.y*(-1. + t)*t*t - p3.y*t*t*t)*
//			(t*(-2.*p2.y + 3.*p2.y*t - 1.*p3.y*t) + p1.y*(-1. + 4.*t - 3.*t*t) +
//				p0.y*(1 - 2.*t + t*t));
//};

//template<typename T>
//class BezierCurveClosest : public Problem<T>
//{
//public:
//	using typename Problem<T>::TVector;
//
//	T value(const TVector &x)
//	{
//		const double t = x[0];
//		const double dx = pos.x - p0.x*(1 - t)*(1 - t)*(1 - t) - 3.*p1.x*(1 - t)*(1 - t)*t - 3.*p2.x*(1 - t)*t*t - p3.x*t*t*t;
//		const double dy = pos.y - p0.y*(1 - t)*(1 - t)*(1 - t) - 3.*p1.y*(1 - t)*(1 - t)*t - 3.*p2.y*(1 - t)*t*t - p3.y*t*t*t;
//		return dx*dx + dy*dy;
//	}
//
//	void gradient(const TVector &x, TVector &grad)
//	{
//		const double t = x[0];
//
//		grad[0] = 6.*(pos.x + p0.x*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.x*(-1. + t)*(-1. + t)*t +
//			3.*p2.x*(-1. + t)*t*t - p3.x*t*t*t)*
//			(t*(-2.*p2.x + 3.*p2.x*t - 1.*p3.x*t) + p1.x*(-1. + 4.*t - 3.*t*t) +
//				p0.x*(1 - 2.*t + t*t)) +
//			6.*(pos.y + p0.y*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.y*(-1. + t)*(-1. + t)*t +
//				3.*p2.y*(-1. + t)*t*t - p3.y*t*t*t)*
//				(t*(-2.*p2.y + 3.*p2.y*t - 1.*p3.y*t) + p1.y*(-1. + 4.*t - 3.*t*t) +
//					p0.y*(1 - 2.*t + t*t));
//	}
//
//	Vec2 p0, p1, p2, p3;
//	Vec2 pos;
//};

extern int64 profilingTime1;
extern int64 profilingTime2;

/*
全体(profilingTime): 15ms
*/
//inline double solveClosestPoint(const Vec2& pos, const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double initialX)
//{
//	StopwatchMicrosec watch(true);
//
//	BezierCurveClosest<double> f;
//	f.p0 = p0;
//	f.p1 = p1;
//	f.p2 = p2;
//	f.p3 = p3;
//	f.pos = pos;
//
//	Eigen::VectorXd x(1);
//	x << initialX;
//
//	BfgsSolver<BezierCurveClosest<double>> solver;
//
//	solver.minimize(f, x);
//	//LOG(L"BfgsSolver iterations: ", solver.criteria().iterations);
//
//	profilingTime1 += watch.us();
//
//	return x[0];
//}

//inline std::pair<double, double> solveClosestPoint(const Vec2& pos, const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3)
//{
//	StopwatchMicrosec watch(true);
//
//	BezierCurveClosest<double> f;
//	f.p0 = p0;
//	f.p1 = p1;
//	f.p2 = p2;
//	f.p3 = p3;
//	f.pos = pos;
//
//	Eigen::VectorXd xs(1), xe(1);
//	xs << 0.0;
//	xe << 1.0;
//
//	BfgsSolver<BezierCurveClosest<double>> solver;
//
//	solver.minimize(f, xs);
//	solver.minimize(f, xe);
//	//LOG(L"BfgsSolver iterations: ", solver.criteria().iterations);
//
//	profilingTime1 += watch.us();
//
//	return{ xs[0], xe[0] };
//}

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
		//const auto func = [&](double t)
		//{
		//	const double dx = pos.x - p0.x*(1 - t)*(1 - t)*(1 - t) - 3.*p1.x*(1 - t)*(1 - t)*t - 3.*p2.x*(1 - t)*t*t - p3.x*t*t*t;
		//	const double dy = pos.y - p0.y*(1 - t)*(1 - t)*(1 - t) - 3.*p1.y*(1 - t)*(1 - t)*t - 3.*p2.y*(1 - t)*t*t - p3.y*t*t*t;
		//	return dx*dx + dy*dy;
		//};

		////func の微分
		//const auto dfunc = [&](double t)
		//{
		//	return 6.*(pos.x + p0.x*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.x*(-1. + t)*(-1. + t)*t +
		//		3.*p2.x*(-1. + t)*t*t - p3.x*t*t*t)*
		//		(t*(-2.*p2.x + 3.*p2.x*t - 1.*p3.x*t) + p1.x*(-1. + 4.*t - 3.*t*t) +
		//			p0.x*(1 - 2.*t + t*t)) +
		//		6.*(pos.y + p0.y*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.y*(-1. + t)*(-1. + t)*t +
		//			3.*p2.y*(-1. + t)*t*t - p3.y*t*t*t)*
		//			(t*(-2.*p2.y + 3.*p2.y*t - 1.*p3.y*t) + p1.y*(-1. + 4.*t - 3.*t*t) +
		//				p0.y*(1 - 2.*t + t*t));
		//};

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

	/*Optional<double> closestPointOpt(const Vec2& pos, double threshold)const
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
	}*/

	Optional<double> closestPointOpt(const Vec2& pos, double threshold, double startT = 0.0, double endT = 1.0)const
	{
		//const auto func = [&](double t)
		//{
		//	const double dx = pos.x - p0.x*(1 - t)*(1 - t)*(1 - t) - 3.*p1.x*(1 - t)*(1 - t)*t - 3.*p2.x*(1 - t)*t*t - p3.x*t*t*t;
		//	const double dy = pos.y - p0.y*(1 - t)*(1 - t)*(1 - t) - 3.*p1.y*(1 - t)*(1 - t)*t - 3.*p2.y*(1 - t)*t*t - p3.y*t*t*t;
		//	return dx*dx + dy*dy;
		//};

		////func の微分
		//const auto dfunc = [&](double t)
		//{
		//	return 6.*(pos.x + p0.x*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.x*(-1. + t)*(-1. + t)*t +
		//		3.*p2.x*(-1. + t)*t*t - p3.x*t*t*t)*
		//		(t*(-2.*p2.x + 3.*p2.x*t - 1.*p3.x*t) + p1.x*(-1. + 4.*t - 3.*t*t) +
		//			p0.x*(1 - 2.*t + t*t)) +
		//		6.*(pos.y + p0.y*(-1 + t)*(-1 + t)*(-1 + t) - 3.*p1.y*(-1. + t)*(-1. + t)*t +
		//			3.*p2.y*(-1. + t)*t*t - p3.y*t*t*t)*
		//			(t*(-2.*p2.y + 3.*p2.y*t - 1.*p3.y*t) + p1.y*(-1. + 4.*t - 3.*t*t) +
		//				p0.y*(1 - 2.*t + t*t));
		//};

		//if (auto tOpt = newtonMethod(func, dfunc, 0.5, threshold, 100))
		
		//LOG(L"closestPointOpt: newton method succeed");

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
		
		//LOG_ERROR(L"closestPointOpt: newton method failed");
		
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