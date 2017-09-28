#pragma once
#include <Siv3D.hpp>
#include "SegmentTree.hpp"
#include "DMat3x2.hpp"
#include "clipper/clipper.hpp"

static const double scaleInt = 100000.0;

struct ClipVertex
{
	ClipVertex() = default;
	ClipVertex(int x, int y, ClipperLib::cInt z) :
		m_pos(1.0*x / scaleInt, 1.0*y / scaleInt),
		m_Z(z)
	{}

	ClipVertex(const ClipperLib::IntPoint& p) :
		m_pos(1.0*p.X / scaleInt, 1.0*p.Y / scaleInt),
		m_Z(p.Z)
	{}

	bool samePos(const ClipVertex& other)const
	{
		return m_pos == other.m_pos;
	}

	Vec2 m_pos;
	ClipperLib::cInt m_Z;
};

/*
三次ベジェ曲線で構成されたパスの1セグメントを表すクラス
他のセグメントとの交差判定はSegmentTreeを使って二分探索で行う
自分との交差判定は記号解を用いて行う
*/
class CurveSegment
{
public:

	CurveSegment() = default;

	CurveSegment(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double startT = 0.0, double endT = 1.0) :
		m_curve(p0, p1, p2, p3),
		m_startT(startT),
		m_endT(endT)
	{
		init();
	}

	CurveSegment(const Line& beginHandle, const Line& endHandle, double startT = 0.0, double endT = 1.0) :
		m_curve(beginHandle.begin, beginHandle.end, endHandle.begin, endHandle.end),
		m_startT(startT),
		m_endT(endT)
	{
		init();
	}

	CurveSegment(const BezierCurve& bezierCurve, double startT = 0.0, double endT = 1.0) :
		m_curve(bezierCurve),
		m_startT(startT),
		m_endT(endT)
	{
		init();
	}

	CurveSegment(const CurveSegment& segment)
	{
		*this = segment;
		init();
	}

	double eval_(const Vec2& pos, double t)const
	{
		/*
		(-3*p0x*(1 - t)*(1 - t) + 3.*p1x*(1 - t)*(1 - t) - 6.*p1x*(1 - t)*t + 6.*p2x*(1 - t)*t -
		3.*p2x*t*t + 3*p3x*t*t)*(-(p0x*(1 - t)*(1 - t)*(1 - t)) - 3.*p1x*(1 - t)*(1 - t)*t -
		3.*p2x*(1 - t)*t*t - p3x*t*t*t + vx) +
		(-3*p0y*(1 - t)*(1 - t) + 3.*p1y*(1 - t)*(1 - t) - 6.*p1y*(1 - t)*t + 6.*p2y*(1 - t)*t -
		3.*p2y*t*t + 3*p3y*t*t)*(-(p0y*(1 - t)*(1 - t)*(1 - t)) - 3.*p1y*(1 - t)*(1 - t)*t -
		3.*p2y*(1 - t)*t*t - p3y*t*t*t + vy)
		*/

		const auto& p0 = m_curve.p0;
		const auto& p1 = m_curve.p1;
		const auto& p2 = m_curve.p2;
		const auto& p3 = m_curve.p3;

		return (-3 * p0.x*(1 - t)*(1 - t) + 3.*p1.x*(1 - t)*(1 - t) - 6.*p1.x*(1 - t)*t + 6.*p2.x*(1 - t)*t -
			3.*p2.x*t*t + 3 * p3.x*t*t)*(-(p0.x*(1 - t)*(1 - t)*(1 - t)) - 3.*p1.x*(1 - t)*(1 - t)*t -
				3.*p2.x*(1 - t)*t*t - p3.x*t*t*t + pos.x) +
				(-3 * p0.y*(1 - t)*(1 - t) + 3.*p1.y*(1 - t)*(1 - t) - 6.*p1.y*(1 - t)*t + 6.*p2.y*(1 - t)*t -
					3.*p2.y*t*t + 3 * p3.y*t*t)*(-(p0.y*(1 - t)*(1 - t)*(1 - t)) - 3.*p1.y*(1 - t)*(1 - t)*t -
						3.*p2.y*(1 - t)*t*t - p3.y*t*t*t + pos.y);
	}

	double eval(const Vec2& pos, double t)const
	{
		const double u = 1.0 - t;
		const double t2 = t*t;
		const double u2 = u*u;

		const double p0x = m_curve.p0.x;
		const double p0y = m_curve.p0.y;
		const double p1x = m_curve.p1.x;
		const double p1y = m_curve.p1.y;
		const double p2x = m_curve.p2.x;
		const double p2y = m_curve.p2.y;
		const double p3x = m_curve.p3.x;
		const double p3y = m_curve.p3.y;

		return
			(-3 * p0x*u2 + 3.*p1x*u2 - 6.*p1x*u*t + 6.*p2x*u*t - 3.*p2x*t2 + 3 * p3x*t2)*(-(p0x*u2*u) - 3.*p1x*u2*t - 3.*p2x*u*t2 - p3x*t2*t + pos.x) +
			(-3 * p0y*u2 + 3.*p1y*u2 - 6.*p1y*u*t + 6.*p2y*u*t - 3.*p2y*t2 + 3 * p3y*t2)*(-(p0y*u2*u) - 3.*p1y*u2*t - 3.*p2y*u*t2 - p3y*t2*t + pos.y);
	}

	double eval2_(const Vec2& pos,double t)const
	{
		/*
		(3*p0x*(1 - t)^2 - 3.*p1x*(1 - t)^2 + 6.*p1x*(1 - t)*t - 6.*p2x*(1 - t)*t +
		3.*p2x*t^2 - 3*p3x*t^2)*(-3*p0x*(1 - t)^2 + 3.*p1x*(1 - t)^2 -
		6.*p1x*(1 - t)*t + 6.*p2x*(1 - t)*t - 3.*p2x*t^2 + 3*p3x*t^2) +
		(3*p0y*(1 - t)^2 - 3.*p1y*(1 - t)^2 + 6.*p1y*(1 - t)*t - 6.*p2y*(1 - t)*t +
		3.*p2y*t^2 - 3*p3y*t^2)*(-3*p0y*(1 - t)^2 + 3.*p1y*(1 - t)^2 -
		6.*p1y*(1 - t)*t + 6.*p2y*(1 - t)*t - 3.*p2y*t^2 + 3*p3y*t^2) +
		(6*p0x*(1 - t) - 12.*p1x*(1 - t) + 6.*p2x*(1 - t) + 6.*p1x*t - 12.*p2x*t +
		6*p3x*t)*(-(p0x*(1 - t)^3) - 3.*p1x*(1 - t)^2*t - 3.*p2x*(1 - t)*t^2 -
		p3x*t^3 + vx) + (6*p0y*(1 - t) - 12.*p1y*(1 - t) + 6.*p2y*(1 - t) +
		6.*p1y*t - 12.*p2y*t + 6*p3y*t)*(-(p0y*(1 - t)^3) - 3.*p1y*(1 - t)^2*t -
		3.*p2y*(1 - t)*t^2 - p3y*t^3 + vy)
		*/

		const auto& p0 = m_curve.p0;
		const auto& p1 = m_curve.p1;
		const auto& p2 = m_curve.p2;
		const auto& p3 = m_curve.p3;

		return (3*p0.x*(1 - t)*(1 - t) - 3.*p1.x*(1 - t)*(1 - t) + 6.*p1.x*(1 - t)*t - 6.*p2.x*(1 - t)*t +
		3.*p2.x*t*t - 3*p3.x*t*t)*(-3*p0.x*(1 - t)*(1 - t) + 3.*p1.x*(1 - t)*(1 - t) -
		6.*p1.x*(1 - t)*t + 6.*p2.x*(1 - t)*t - 3.*p2.x*t*t + 3*p3.x*t*t) +
		(3*p0.y*(1 - t)*(1 - t) - 3.*p1.y*(1 - t)*(1 - t) + 6.*p1.y*(1 - t)*t - 6.*p2.y*(1 - t)*t +
		3.*p2.y*t*t - 3*p3.y*t*t)*(-3*p0.y*(1 - t)*(1 - t) + 3.*p1.y*(1 - t)*(1 - t) -
		6.*p1.y*(1 - t)*t + 6.*p2.y*(1 - t)*t - 3.*p2.y*t*t + 3*p3.y*t*t) +
		(6*p0.x*(1 - t) - 12.*p1.x*(1 - t) + 6.*p2.x*(1 - t) + 6.*p1.x*t - 12.*p2.x*t +
		6*p3.x*t)*(-(p0.x*(1 - t)*(1 - t)*(1 - t)) - 3.*p1.x*(1 - t)*(1 - t)*t - 3.*p2.x*(1 - t)*t*t -
		p3.x*t*t*t + pos.x) + (6*p0.y*(1 - t) - 12.*p1.y*(1 - t) + 6.*p2.y*(1 - t) +
		6.*p1.y*t - 12.*p2.y*t + 6*p3.y*t)*(-(p0.y*(1 - t)*(1 - t)*(1 - t)) - 3.*p1.y*(1 - t)*(1 - t)*t -
		3.*p2.y*(1 - t)*t*t - p3.y*t*t*t + pos.y);
	}

	double eval2(const Vec2& pos, double t)const
	{
		const double u = 1.0 - t;
		const double t2 = t*t;
		const double u2 = u*u;

		const double p0x = m_curve.p0.x;
		const double p0y = m_curve.p0.y;
		const double p1x = m_curve.p1.x;
		const double p1y = m_curve.p1.y;
		const double p2x = m_curve.p2.x;
		const double p2y = m_curve.p2.y;
		const double p3x = m_curve.p3.x;
		const double p3y = m_curve.p3.y;

		const double A =(3*p0x*u2 - 3.*p1x*u2 + 6.*p1x*u*t - 6.*p2x*u*t + 3.*p2x*t2 - 3*p3x*t2);
		const double B =(3*p0y*u2 - 3.*p1y*u2 + 6.*p1y*u*t - 6.*p2y*u*t + 3.*p2y*t2 - 3*p3y*t2);

		return -A*A - B*B +
			(6 * p0x*u - 12.*p1x*u + 6.*p2x*u + 6.*p1x*t - 12.*p2x*t + 6 * p3x*t)*(-(p0x*u2*u) - 3.*p1x*u2*t - 3.*p2x*u*t2 - p3x*t2*t + pos.x) +
			(6 * p0y*u - 12.*p1y*u + 6.*p2y*u + 6.*p1y*t - 12.*p2y*t + 6 * p3y*t)*(-(p0y*u2*u) - 3.*p1y*u2*t - 3.*p2y*u*t2 - p3y*t2*t + pos.y);

		/*
		u = (1 - t)

		A = (3*p0x*u^2 - 3.*p1x*u^2 + 6.*p1x*u*t - 6.*p2x*u*t + 3.*p2x*t^2 - 3*p3x*t^2);
		B = (3*p0y*u^2 - 3.*p1y*u^2 + 6.*p1y*u*t - 6.*p2y*u*t + 3.*p2y*t^2 - 3*p3y*t^2);

		A*-A +
		B*-B +
		(6*p0x*u - 12.*p1x*u + 6.*p2x*u + 6.*p1x*t - 12.*p2x*t + 6*p3x*t)*(-(p0x*u^3) - 3.*p1x*u^2*t - 3.*p2x*u*t^2 - p3x*t^3 + vx) +
		(6*p0y*u - 12.*p1y*u + 6.*p2y*u + 6.*p1y*t - 12.*p2y*t + 6*p3y*t)*(-(p0y*u^3) - 3.*p1y*u^2*t - 3.*p2y*u*t^2 - p3y*t^3 + vy);
		*/
	}

	void setLine(const Vec2& p0, const Vec2& p1)
	{
		m_curve = BezierCurve(p0, p0, p1, p1);
	}

	void drawSegmentTree()const
	{
		m_segmentTree->draw();
	}

	std::vector<SegmentTree::CurvePoint> intersectionPoints(CurveSegment& other)
	{
		std::vector<SegmentTree::CurvePoint> result;

		return m_segmentTree->intersectionPoints(m_curve, other.m_segmentTree, other.m_curve);
	}

	Optional<SegmentTree::CurvePoint> selfIntersection()const
	{
		/*
		Reduce[(1 - t)^3 x1 + 3(1 - t)^2  t  x2 + 3(1 - t) t^2 x3 + t^3 x4 == (1 - s)^3 x1 + 3(1 - s)^2 s x2 + 3(1 - s) s^2 x3 + s^3 x4 && (1 - t)^3 y1 + 3(1 - t)^2  t  y2 + 3(1 - t) t^2 y3 + t^3 y4 == (1 - s)^3 y1 + 3(1 - s)^2 s y2 + 3(1 - s) s^2 y3 + s^3 y4 && s != t, {s, t}]
		をFactor,Simplifyを使って整形したもの
		*/

		double x1 = m_curve.p0.x, x2 = m_curve.p1.x, x3 = m_curve.p2.x, x4 = m_curve.p3.x;
		double y1 = m_curve.p0.y, y2 = m_curve.p1.y, y3 = m_curve.p2.y, y4 = m_curve.p3.y;

		const double a = 2 * x2*y1 - 3 * x3*y1 + x4*y1 - 2 * x1*y2 + 3 * x3*y2 - x4*y2 + 3 * x1*y3 - 3 * x2*y3 - x1*y4 + x2*y4;
		const double b = x2*y1 - 2 * x3*y1 + x4*y1 - x1*y2 + 3 * x3*y2 - 2 * x4*y2 + 2 * x1*y3 - 3 * x2*y3 + x4*y3 - x1*y4 + 2 * x2*y4 - x3*y4;
		const double c = a*b;
		const double d = x4*(y1 - 2 * y2 + y3) - x3*(2 * y1 - 3 * y2 + y4) - x1*(y2 - 2 * y3 + y4) + x2*(y1 - 3 * y3 + 2 * y4);

		const double s = (
			c
			- Sqrt(
				c*c
				- 4 * (x4*x4*y1*y1 - x1*x4*y1*y2 - 2 * x4*x4*y1*y2 +
					x1*x1*y2*y2 - 2 * x1*x4*y2*y2 + x4*x4*y2*y2 +
					3 * x1*x4*y1*y3 - 3 * x1*x1*y2*y3 + 3 * x1*x4*y2*y3 +
					3 * x1*x1*y3*y3 - 3 * x1*x4*y3*y3 +
					3 * x3*x3*(y1 - y2)*(y1 - y4) + x2*x2*(y1 - y4)*(y1 - y4) -
					2 * x1*x4*y1*y4 + x1*x1*y2*y4 + 2 * x1*x4*y2*y4 - 3 * x1*x1*y3*y4 +
					x1*x1*y4*y4 + x2*
					(-3 * x3*(y1 - y3)*(y1 - y4) +
						x4*(y1*y1 + 3 * y3*y3 - 2 * y2*y4 + 2 * y1*(y2 - 3 * y3 + y4)) -
						x1*(3 * y3*y3 - 3 * y3*y4 + 2 * y4*(-y2 + y4) + y1*(2 * y2 - 3 * y3 + y4)))
					+ 3 * x3*(-(x4*(y1 - y2)*(y1 - y3)) +
						x1*(y2*y3 - 2 * y2*y4 + y3*y4 + y1*(y2 - 2 * y3 + y4))))
				* b*b
			)
			)
			/
			(2.*d*d);

		const double t =
			(3 * (-((x3 - x4)*(y1 - y2)) + (x1 - x2)*(y3 - y4)) +
				3 * s*(-(x4*(y1 - 2 * y2 + y3)) + x3*(2 * y1 - 3 * y2 + y4) +
					x1*(y2 - 2 * y3 + y4) - x2*(y1 - 3 * y3 + 2 * y4)) +
				2 * s*s*(x4*(y1 - 2 * y2 + y3) - x3*(2 * y1 - 3 * y2 + y4) -
					x1*(y2 - 2 * y3 + y4) + x2*(y1 - 3 * y3 + 2 * y4))) /
					(x4*(y1 - 4 * y2 + 3 * y3) - x2*(y1 + 3 * y3 - 4 * y4) + (x1 + 3 * x3)*(y2 - y4));

		if (0.0 < s&&s < 1.0&&0.0 < t&&t < 1.0)
		{
			return SegmentTree::CurvePoint(s, m_curve(s));
		}

		return none;
	}

	void print()const
	{
		Println(m_segmentTree->boundingRect().toString());
	}

	void lengthProfile()
	{
		double max_error1 = 0.0, max_error2 = 0.0;
		int64 time1 = 0, time2 = 0;
		for (int i = 0; i < 10000; ++i)
		{
			m_curve.p0 = RandomVec2(Window::ClientRect());
			m_curve.p1 = RandomVec2(Window::ClientRect());
			m_curve.p2 = RandomVec2(Window::ClientRect());
			m_curve.p3 = RandomVec2(Window::ClientRect());

			const double lengthCorrect = m_curve.lengthCorrect(m_startT, m_endT);
			
			{
				StopwatchMicrosec watch1(true);
				const double length1 = m_curve.length(m_startT, m_endT);
				time1 += watch1.us();
				const double error1 = abs(length1 - lengthCorrect);
				if (max_error1 < error1)
				{
					max_error1 = error1;
				}
			}
			
			{
				StopwatchMicrosec watch2(true);
				const double length2 = m_curve.lengthBySimpsonsRule(m_startT, m_endT);
				time2 += watch2.us();
				const double error2 = abs(length2 - lengthCorrect);
				if (max_error2 < error2)
				{
					max_error2 = error2;
				}
			}
		}
		
		LOG(L"Old method: ");
		LOG(L"____Whole time: ", time1, L"[us], ", time1 / 1000.0, L"[ms]");
		LOG(L"____Max error: ", max_error1);
		
		LOG(L"Simpson method: ");
		LOG(L"____Whole time: ", time2, L"[us], ", time2 / 1000.0, L"[ms]");
		LOG(L"____Max error: ", max_error2);

		//Window::SetTitle(L"length1: ", length1, L", length2: ", length2);
	}

	void closestPointProfile()
	{
		double max_error1 = 0.0, max_error2 = 0.0, max_error3 = 0.0;
		int64 time1 = 0, time2 = 0, time3 = 0;
		for (int i = 0; i < 10000; ++i)
		{
			m_curve.p0 = RandomVec2(Window::ClientRect());
			m_curve.p1 = RandomVec2(Window::ClientRect());
			m_curve.p2 = RandomVec2(Window::ClientRect());
			m_curve.p3 = RandomVec2(Window::ClientRect());

			const Vec2 pos = RandomVec2(Window::ClientRect());

			StopwatchMicrosec watch2(true);
			auto resultOpt = calcClosestByStrumOpt(pos, 0.01);
			const int64 currentTime2 = watch2.us();

			StopwatchMicrosec watch3(true);
			auto resultOpt3 = calcClosestByStrumAndNewtonOpt(pos, 0.01);
			const int64 currentTime3 = watch3.us();

			//外側にある場合は誤差を測る意味がないので除外
			if (!resultOpt || !resultOpt3)
			{
				continue;
			}
			const double closestTCorrect = m_curve.closestPointCorrect(pos);
			const Vec2 closestPointCorrect = curve(closestTCorrect);

			const Vec2 closestPoint2 = curve(resultOpt.value());
			time2 += currentTime2;
			if (max_error2 < closestPointCorrect.distanceFrom(closestPoint2))
			{
				max_error2 = closestPointCorrect.distanceFrom(closestPoint2);
				/*if (2500 < i)
				{
					Image image(640, 480, Palette::White);
					write(image, 30, Palette::Black);
					Circle(closestPointCorrect, 5).write(image, Palette::Blue);
					Circle(closestPoint2, 5).write(image, Palette::Red);
					Circle(pos, 5).writeFrame(image, 1.0, 0, Palette::Gray);
					image.savePNG(Format(L"closestPoint_", i, L".png"));
				}*/
			}

			const Vec2 closestPoint3 = curve(resultOpt3.value());
			time3 += currentTime3;
			if (max_error3 < closestPointCorrect.distanceFrom(closestPoint3))
			{
				max_error3 = closestPointCorrect.distanceFrom(closestPoint3);
			}
			
			StopwatchMicrosec watch1(true);
			const double closestT1 = m_curve.closestPoint(pos);
			time1 += watch1.us();
			const Vec2 closestPoint1 = curve(closestT1);

			if (max_error1 < closestPointCorrect.distanceFrom(closestPoint1))
			{
				max_error1 = closestPointCorrect.distanceFrom(closestPoint1);
			}
		}

		LOG(L"Old method: ");
		LOG(L"____Whole time: ", time1, L"[us], ", time1 / 1000.0, L"[ms]");
		LOG(L"____Max error: ", max_error1);

		LOG(L"Strum method: ");
		LOG(L"____Whole time: ", time2, L"[us], ", time2 / 1000.0, L"[ms]");
		LOG(L"____Max error: ", max_error2);

		LOG(L"Strum and newton method: ");
		LOG(L"____Whole time: ", time3, L"[us], ", time3 / 1000.0, L"[ms]");
		LOG(L"____Max error: ", max_error3);
	}

	/*const BezierCurve& curve()const
	{
		return m_curve;
	}

	Vec2 curve(double t)const
	{
		return m_curve(t);
	}*/

	Vec2 curve(double t)const
	{
		return m_transform.transform(m_curve(t));
	}

	double closestPoint(const Vec2& pos)const
	{
		return m_curve.closestPoint(m_transformInv.transform(pos));
	}

	Optional<double> closestPointOpt(const Vec2& pos, double threshold)const
	{
		const double scaling = m_transformInv.transform(Vec2(0, 0)).distanceFrom(m_transformInv.transform(Vec2(1, 0)));
		//LOG(__FILEW__,L", scaling: ", scaling);
		
		return m_curve.closestPointOpt(m_transformInv.transform(pos), threshold*scaling, m_startT, m_endT);
	}

	void draw(int divNum = 30, const Color& color = Palette::White)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(curve(Lerp(m_startT, m_endT, progress0)), curve(Lerp(m_startT, m_endT, progress1))).draw(color);
		}
	}

	/*void write(Image& image, int divNum = 30, const Color& color = Palette::White, double startT = 0.0, double endT = 1.0)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(curve(Lerp(startT, endT, progress0)), curve(Lerp(startT, endT, progress1))).write(image, color);
		}
	}*/

	void write(Image& image, int divNum = 30, const Color& color = Palette::White)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(curve(Lerp(m_startT, m_endT, progress0)), curve(Lerp(m_startT, m_endT, progress1))).write(image, color);
		}
	}
	
	void drawArrow(int divNum = 30, const Color& color = Palette::White)const
	{
		for (int i = 0; i < divNum; ++i)
		{
			const double progress0 = 1.0*i / divNum;
			const double progress1 = 1.0*(i + 1) / divNum;
			Line(curve(Lerp(m_startT, m_endT, progress0)), curve(Lerp(m_startT, m_endT, progress1))).drawArrow(1.0, { 5.0,5.0 }, color);
		}
	}

	void setRange(double startT, double endT)
	{
		m_startT = startT;
		m_endT = endT;
	}

	void setBegin(double t)
	{
		m_startT = t;
	}

	void setEnd(double t)
	{
		m_endT = t;
	}

	std::pair<double, double> range()const
	{
		return{ m_startT,m_endT };
	}

	void getSpecificPathHighPrecision(ClipperLib::Path& output, size_t zIndex, double interval, double permissibleError = 1.0)const
	{
		const auto adder = [&](const Vec2& pos_, size_t zIndex)
		{
			const Vec2 pos = m_transform.transform(pos_);
			//LOG(__LINE__, L", transform: ", m_transform);
			//LOG(__LINE__, L", pos:", pos_, L" -> ", pos);
			output << ClipperLib::IntPoint(pos.x * scaleInt, pos.y * scaleInt, zIndex);
		};

		getSpecificPathHighPrecisionImpl(adder, zIndex, interval, permissibleError);
	}

	void getSpecificPathHighPrecision(std::vector<Vec2>& output, double interval, double permissibleError = 1.0)const
	{
		const auto adder = [&](const Vec2& pos, size_t zIndex)
		{
			output.push_back(m_transform.transform(pos));
		};

		getSpecificPathHighPrecisionImpl(adder, 0, interval, permissibleError);
	}

	/*Optional<double> closestPointOpt(const Vec2& pos, double threshold)const
	{
		return m_curve.closestPointOpt(pos, threshold, m_startT, m_endT);
	}*/

	void setTransform(const DMat3x2& transform, const DMat3x2& transformInv)
	{
		m_transform = transform;
		m_transformInv = transformInv;
	}

	BoundingRect boundingRect()const
	{
		BoundingRect rect;

		const auto& p0 = m_curve.p0;
		const auto& p1 = m_curve.p1;
		const auto& p2 = m_curve.p2;
		const auto& p3 = m_curve.p3;

		/*
		https://stackoverflow.com/questions/24809978/calculating-the-bounding-box-of-cubic-bezier-curve
		*/
		const Vec2 a = (-3 * p0 + 9 * p1 - 9 * p2 + 3 * p3);
		const Vec2 b = (6 * p0 - 12 * p1 + 6 * p2);
		const Vec2 c = 3 * (p1 - p0);
		const Vec2 d = b*b - 4 * a*c;

		//rect.add(p0);
		rect.add(curve(0.0));

		if (0 < d.x)
		{
			const double dSqrt = sqrt(d.x);
			const double t1 = (-b.x + dSqrt) / (2 * a.x);
			const double t2 = (-b.x - dSqrt) / (2 * a.x);
			if (0 < t1&&t1 < 1) {
				//rect.add(m_curve(t1));
				rect.add(curve(t1));
			}
			if (0 < t2&&t2 < 1) {
				//rect.add(m_curve(t2));
				rect.add(curve(t2));
			}
		}
		if (0 < d.y)
		{
			const double dSqrt = sqrt(d.y);
			const double t1 = (-b.y + dSqrt) / (2 * a.y);
			const double t2 = (-b.y - dSqrt) / (2 * a.y);
			if (0 < t1&&t1 < 1) {
				//rect.add(m_curve(t1));
				rect.add(curve(t1));
			}
			if (0 < t2&&t2 < 1) {
				//rect.add(m_curve(t2));
				rect.add(curve(t2));
			}
		}

		//rect.add(p3);
		rect.add(curve(1.0));

		return rect;
	}

	// 1 < flatnessFactor
	void constructLinearApproximation(double flatnessFactor = 1.1)
	{
		m_linearApprox.clear();

		m_linearApprox.push_back(m_curve.p0);
		constructLinearApproximationImpl(flatnessFactor, m_curve.p0, m_curve.p1, m_curve.p2, m_curve.p3);
		m_linearApprox.push_back(m_curve.p3);
	}

	void drawLinear(const Color& c)const
	{
		LineString(m_linearApprox).draw(c);
		Window::SetTitle(m_linearApprox.size());
	}

	double calcClosestByStrum(const Vec2& pos, double eps = 0.01)const
	{
		if (auto tOpt = calcClosestByStrumOpt(pos, eps))
		{
			return tOpt.value();
		}

		return curve(m_startT).distanceFromSq(pos) < curve(m_endT).distanceFromSq(pos) ? m_startT : m_endT;
	}

	Optional<double> calcClosestByStrumOpt(const Vec2& pos, double eps = 0.01)const
	{
		std::vector<double> result;
		calcClosestByStrumImpl(pos, result, Min(m_startT, m_endT), Max(m_startT, m_endT), eps);

		if (result.empty())
		{
			return none;
		}

		const auto minIt = std::min_element(result.cbegin(), result.cend(), [&](double a,double b)
		{
			return m_curve(a).distanceFromSq(pos) < m_curve(b).distanceFromSq(pos);
		});

		return *minIt;
	}

	Optional<double> calcClosestByStrumAndNewtonOpt(const Vec2& pos, double eps = 0.01)const
	{
		std::vector<double> result;
		calcClosestByStrumAndNewtonImpl(pos, result, Min(m_startT, m_endT), Max(m_startT, m_endT), eps);

		if (result.empty())
		{
			return none;
		}

		const auto minIt = std::min_element(result.cbegin(), result.cend(), [&](double a, double b)
		{
			return m_curve(a).distanceFromSq(pos) < m_curve(b).distanceFromSq(pos);
		});

		return *minIt;
	}

	/*
	int calcClosestByStrumImpl(const Vec2& pos, double leftT, double rightT)const
	{
		const double midT = (leftT + rightT)*0.5;

		const auto numOfRoots = [&](double t)
		{
			const double f0 = eval(pos, t);
			const char s0 = 0.0 < f0 ? 1 : 0;

			const double f1 = eval2(pos, t);
			const char s1 = 0.0 < f1 ? 1 : 0;

			const double q0 = fmod(f1, f0);
			const double f2 = q0*f1 - f0;
			const char s2 = 0.0 < f2 ? 1 : 0;

			const double q1 = fmod(f2, f1);
			const double f3 = q1*f2 - f1;
			const char s3 = 0.0 < f3 ? 1 : 0;

			const double q2 = fmod(f3, f2);
			const double f4 = q2*f3 - f2;
			const char s4 = 0.0 < f4 ? 1 : 0;

			const double q3 = fmod(f4, f3);
			const double f5 = q3*f4 - f3;
			const char s5 = 0.0 < f5 ? 1 : 0;

			int c = 0;
			c += s0 != s1 ? 1 : 0;
			c += s1 != s2 ? 1 : 0;
			c += s2 != s3 ? 1 : 0;
			c += s3 != s4 ? 1 : 0;
			c += s4 != s5 ? 1 : 0;

			return c;
		};

		const int leftNum = numOfRoots(leftT);
		//numOfRoots(midT);
		const int rightNum = numOfRoots(rightT);

		return leftNum - rightNum;


		// [leftT, midT)
		{

		}


		// [midT, rightT]
	}
	*/

private:

	void constructLinearApproximationImpl(double flatnessFactor, const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3)
	{
		const Vec2 p01 = (p0 + p1)*0.5;
		const Vec2 p12 = (p1 + p2)*0.5;
		const Vec2 p23 = (p2 + p3)*0.5;

		const Vec2 p012 = (p01 + p12)*0.5;
		const Vec2 p123 = (p12 + p23)*0.5;

		const Vec2 p0123 = (p012 + p123)*0.5;

		if ((p0 - p1).length() + (p1 - p2).length() + (p2 - p3).length() < flatnessFactor * (p0 - p3).length())
		{
			m_linearApprox.push_back(p0123);
			return;
		}

		constructLinearApproximationImpl(flatnessFactor, p0, p01, p012, p0123);
		constructLinearApproximationImpl(flatnessFactor, p0123, p123, p23, p3);
	}

	void calcClosestByStrumImpl(const Vec2& pos, std::vector<double>& result, double leftT, double rightT, double eps)const
	{
		const double midT = (leftT + rightT)*0.5;

		const double width = rightT - leftT;
		if (width < eps)
		{
			result.push_back(midT);
			return;
		}
		
		const int leftRoots = strumMethod(pos, leftT);
		const int midRoots = strumMethod(pos, midT);
		const int rightRoots = strumMethod(pos, rightT);

		const int leftNum = leftRoots - midRoots;
		const int rightNum = midRoots - rightRoots;

		if (1 <= leftNum)
		{
			calcClosestByStrumImpl(pos, result, leftT, midT, eps);
		}
		if (1 <= rightNum)
		{
			calcClosestByStrumImpl(pos, result, midT, rightT, eps);
		}
	}

	void calcClosestByStrumAndNewtonImpl(const Vec2& pos, std::vector<double>& result, double leftT, double rightT, double eps)const
	{
		const double midT = (leftT + rightT)*0.5;

		/*const double width = rightT - leftT;
		if (width < eps)
		{
			result.push_back(midT);
			return;
		}*/

		const int leftRoots = strumMethod(pos, leftT);
		const int rightRoots = strumMethod(pos, rightT);

		const int wholeNum = leftRoots - rightRoots;
		if (wholeNum == 1)
		{
			//距離の極小値はこのケースのみ
			if (0.0 < eval(pos, leftT) && eval(pos, rightT) < 0.0)
			{
				double x = midT;
				//100回やってみてだめだったらもう一度スツルムの方法に戻る
				for (int i = 0; i < 100; ++i)
				{
					const double f = eval(pos, x);
					const double df = eval2(pos, x);
					x = x - f / df;

					const double dx = abs(f / df);
					if (dx < eps || abs(f) < eps)
					{
						result.push_back(x);
						return;
					}
				}
				LOG(L"Failed");
			}
			else
			{
				return;
			}
		}

		const int midRoots = strumMethod(pos, midT);
		const int leftNum = leftRoots - midRoots;
		const int rightNum = midRoots - rightRoots;

		if (1 <= leftNum)
		{
			calcClosestByStrumImpl(pos, result, leftT, midT, eps);
		}
		if (1 <= rightNum)
		{
			calcClosestByStrumImpl(pos, result, midT, rightT, eps);
		}
	}

	int strumMethod(const Vec2& pos, double t)const
	{
		const double posx = pos.x;
		const double posy = pos.y;

		const double t_2 = t*t;
		const double t_3 = t_2*t;
		const double t_4 = t_3*t;
		const double t_5 = t_4*t;

		const double p0x = m_curve.p0.x;
		const double p0y = m_curve.p0.y;
		const double p1x = m_curve.p1.x;
		const double p1y = m_curve.p1.y;
		const double p2x = m_curve.p2.x;
		const double p2y = m_curve.p2.y;
		const double p3x = m_curve.p3.x;
		const double p3y = m_curve.p3.y;

		const double p0x_2 = p0x*p0x;
		const double p0y_2 = p0y*p0y;
		const double p1x_2 = p1x*p1x;
		const double p1y_2 = p1y*p1y;
		const double p2x_2 = p2x*p2x;
		const double p2y_2 = p2y*p2y;
		const double p3x_2 = p3x*p3x;
		const double p3y_2 = p3y*p3y;

		const double a0 = 3 * p0x_2 + 3 * p0y_2 - 3.*p0x*p1x - 3.*p0y*p1y - 3 * p0x*posx + 3.*p1x*posx - 3 * p0y*posy + 3.*p1y*posy;
		const double a1 = -15 * p0x_2 - 15 * p0y_2 + 30.*p0x*p1x - 9.*p1x_2 + 30.*p0y*p1y - 9.*p1y_2 - 6.*p0x*p2x - 6.*p0y*p2y +
			6 * p0x*posx - 12.*p1x*posx + 6.*p2x*posx + 6 * p0y*posy - 12.*p1y*posy + 6.*p2y*posy;
		const double a2 = 30 * p0x_2 + 30 * p0y_2 - 90.*p0x*p1x + 54.*p1x_2 - 90.*p0y*p1y + 54.*p1y_2 + 36.*p0x*p2x - 27.*p1x*p2x +
			36.*p0y*p2y - 27.*p1y*p2y - 3 * p0x*p3x - 3 * p0y*p3y - 3 * p0x*posx + 9.*p1x*posx - 9.*p2x*posx + 3 * p3x*posx -
			3 * p0y*posy + 9.*p1y*posy - 9.*p2y*posy + 3 * p3y*posy;
		const double a3 = -30 * p0x_2 - 30 * p0y_2 + 120.*p0x*p1x - 108.*p1x_2 + 120.*p0y*p1y - 108.*p1y_2 - 72.*p0x*p2x + 108.*p1x*p2x -
			18.*p2x_2 - 72.*p0y*p2y + 108.*p1y*p2y - 18.*p2y_2 + 12 * p0x*p3x - 12.*p1x*p3x + 12 * p0y*p3y - 12.*p1y*p3y;
		const double a4 = 15 * p0x_2 + 15 * p0y_2 - 75.*p0x*p1x + 90.*p1x_2 - 75.*p0y*p1y + 90.*p1y_2 + 60.*p0x*p2x - 135.*p1x*p2x +
			45.*p2x_2 + 60.*p0y*p2y - 135.*p1y*p2y + 45.*p2y_2 - 15 * p0x*p3x + 30.*p1x*p3x - 15.*p2x*p3x - 15 * p0y*p3y +
			30.*p1y*p3y - 15.*p2y*p3y;
		const double a5 = -3 * p0x_2 - 3 * p0y_2 + 18.*p0x*p1x - 27.*p1x_2 + 18.*p0y*p1y - 27.*p1y_2 - 18.*p0x*p2x + 54.*p1x*p2x - 27.*p2x_2 -
			18.*p0y*p2y + 54.*p1y*p2y - 27.*p2y_2 + 6 * p0x*p3x - 18.*p1x*p3x + 18.*p2x*p3x - 3 * p3x_2 + 6 * p0y*p3y - 18.*p1y*p3y +
			18.*p2y*p3y - 3 * p3y_2;

		const double a0_2 = a0*a0;
		const double a1_2 = a1*a1;
		const double a2_2 = a2*a2;
		const double a3_2 = a3*a3;
		const double a4_2 = a4*a4;
		const double a5_2 = a5*a5;

		const double a0_3 = a0_2*a0;
		const double a1_3 = a1_2*a1;
		const double a2_3 = a2_2*a2;
		const double a3_3 = a3_2*a3;
		const double a4_3 = a4_2*a4;
		const double a5_3 = a5_2*a5;

		const double f0 = a0 + a1*t + a2*t_2 + a3*t_3 + a4*t_4 + a5*t_5;
		const double f1 = a1 + 2 * a2*t + 3 * a3*t_2 + 4 * a4*t_3 + 5 * a5*t_4;
		const double f2 = (-25 * a0*a5 + a1*(a4 - 20 * a5*t) + t*(a2*(2 * a4 - 15 * a5*t) + t*(3 * a3*a4 + 4 * a4*a4 * t - 10 * a3*a5*t))) / (25 * a5);

		const double temp3_1 = (2 * a4_2 - 5 * a3*a5);
		const double f3 = (25 * a5*(a0*(-16 * a4_3 + 55 * a3*a4*a5 - 20 * a4_2*a5*t + 25 * a5_2*(-3 * a2 + 2 * a3*t)) + a1*(-4 * a3_2*a5 + 3 * a2*a5*(a4 - 20 * a5*t) - 4 * a4_2*t*(3 * a4 + 4 * a5*t) + a3*(a4_2 + 42 * a4*a5*t + 40 * a5_2*t_2)) + t*(3 * a3_2*(a4_2 - 4 * a3*a5)*t + 3 * a2_2*a5*(2 * a4 - 15 * a5*t) + a2*(-8 * a3_2*a5 - 8 * a4_3*t + 2 * a3*a4*(a4 + 19 * a5*t))))) / (4 * temp3_1*temp3_1);

		const double temp4_1 = (2 * a4_2 - 5 * a3*a5);
		const double temp4_2 = (a4_2*(-3 * a3_2 + 8 * a2*a4) + 2 * (6 * a3_3 - 19 * a2*a3*a4 + 8 * a1*a4_2)*a5 + 5 * (9 * a2_2 - 8 * a1*a3)*a5_2);
		const double f4 = (-4 * temp4_1*temp4_1*(2 * a2_2*(-(a3_2*a4_2) + 4 * a3_3*a5 - 18 * a2*a3*a4*a5 + a2*(4 * a4_3 + 27 * a2*a5_2))*t +
			16 * a1_3*a5_2*(a4 - 20 * a5*t) + 5 * a0_2*a5*(16 * a4_3 - 55 * a3*a4*a5 + 20 * a4_2*a5*t +
				25 * a5_2*(3 * a2 - 2 * a3*t)) + a1_2*(-400 * a0*a5_3 + 36 * a4_2*a4_2*t +
					4 * a3_2*a5*(3 * a4 + 44 * a5*t) + a2*a4*a5*(7 * a4 + 264 * a5*t) -
					a3*(3 * a4_3 + 48 * a2*a5_2 + 194 * a4_2*a5*t)) +
			a0*(-36 * a3_2*a3_2*a5 + 3 * a3_3*a4*(3 * a4 - 16 * a5*t) +
				a2*a3*(-195 * a2*a5_2 + 4 * a4_2*(-8 * a4 + 33 * a5*t)) +
				2 * a3_2*(6 * a4_3*t + a2*a5*(73 * a4 + 40 * a5*t)) -
				4 * a2*a4*(8 * a4_3*t + a2*a5*(a4 + 60 * a5*t))) +
			a1*(27 * a2_3*a5_2 + 6 * a3_3*(a4_2 - 4 * a3*a5)*t +
				2 * a2_2*(2 * a4_3 - 9 * a3*a4*a5 + 6 * a4_2*a5*t - 117 * a3*a5_2*t) +
				a0*(48 * a4_2*a4_2 - 266 * a3*a4_2*a5 + 260 * a3_2*a5_2 + 56 * a4_3*a5*t -
					260 * a3*a4*a5_2*t) + a2*(4 * a3_3*a5 - 28 * a3*a4_3*t -
						a3_2*a4*(a4 - 124 * a5*t) + 10 * a0*a5_2*(29 * a4 + 60 * a5*t))))) / (25 * a5*temp4_2*temp4_2);

		const double temp5_1 = (a4_2*(3 * a3_2 - 8 * a2*a4) - 2 * (6 * a3_3 - 19 * a2*a3*a4 + 8 * a1*a4_2)*a5 + 5 * (-9 * a2_2 + 8 * a1*a3)*a5_2);
		const double temp5_2 = (2 * a4_2 - 5 * a3*a5);
		const double temp5_3 = (a4_2*(-(a2_2*a3_2) + 4 * a2_3*a4 - 2 * a2*a4*(7 * a1*a3 + 8 * a0*a4) +
			3 * (a1*a3_3 + 2 * a0*a3_2*a4 + 6 * a1_2*a4_2)) +
			(4 * a3_3*(a2_2 - 3 * a1*a3) - 2 * a3*(9 * a2_3 - 31 * a1*a2*a3 + 12 * a0*a3_2)*a4 +
			(6 * a1*a2_2 - 97 * a1_2*a3 + 66 * a0*a2*a3)*a4_2 + 28 * a0*a1*a4_3)*a5 +
				(27 * a2_2*a2_2 + 88 * a1_2*a3_2 - 130 * a0*a1*a3*a4 + 50 * a0_2*a4_2 -
					3 * a2_2*(39 * a1*a3 + 40 * a0*a4) + 4 * a2*(10 * a0*a3_2 + 33 * a1_2*a4))*a5_2 -
			5 * (32 * a1_3 - 60 * a0*a1*a2 + 25 * a0_2*a3)*a5_3);

		const double f5 = (25 * a5*temp5_1*temp5_1*
			(a4_2*(a3_2*(a1_2*a2_2 - 4 * a1_3*a3 + 18 * a0*a1*a2*a3 -
				a0*(4 * a2_3 + 27 * a0*a3_2)) + 2 * (-2 * a2_3*(a1_2 - 4 * a0*a2) +
					a1*a2*(9 * a1_2 - 40 * a0*a2)*a3 - 3 * a0*(a1_2 - 24 * a0*a2)*a3_2)*a4 -
					(27 * a1_2*a1_2 - 144 * a0*a1_2*a2 + 128 * a0_2*a2_2 + 192 * a0_2*a1*a3)*a4_2 +
				256 * a0_3*a4_3) + 2 * (2 * a3_3*(-(a1_2*a2_2) + 4 * a1_3*a3 - 18 * a0*a1*a2*a3 +
					a0*(4 * a2_3 + 27 * a0*a3_2)) + a3*(9 * a2_3*(a1_2 - 4 * a0*a2) +
						2 * a1*a2*(-20 * a1_2 + 89 * a0*a2)*a3 + 3 * a0*(4 * a1_2 - 105 * a0*a2)*a3_2)*
					a4 + (-3 * a1_3*a2_2 + 72 * a1_2*a1_2*a3 - 373 * a0*a1_2*a2*a3 +
						280 * a0_2*a2_2*a3 + 6 * a0*a1*(2 * a2_3 + 85 * a0*a3_2))*a4_2 -
					2 * a0*(9 * a1_3 - 40 * a0*a1*a2 + 400 * a0_2*a3)*a4_3)*a5 +
					(16 * a1_3*a3*(9 * a2_2 + 10 * a0*a4) - 64 * a1_2*a1_2*(2 * a3_2 + 3 * a2*a4) -
						10 * a0*a1*a3*(63 * a2_3 + 90 * a0*a3_2 + 205 * a0*a2*a4) +
						a1_2*(-27 * a2_2*a2_2 + 560 * a0*a2*a3_2 + 1020 * a0*a2_2*a4 - 50 * a0_2*a4_2) +
						a0*(108 * a2_3*a2_2 + 825 * a0*a2_2*a3_2 - 900 * a0*a2_3*a4 + 2250 * a0_2*a3_2*a4 +
							2000 * a0_2*a2*a4_2))*a5_2 + 2 * (128 * a1_3*a1_2 - 800 * a0*a1_3*a2 +
								1000 * a0_2*a1_2*a3 - 1875 * a0_3*a2*a3 + 125 * a0_2*a1*(9 * a2_2 - 10 * a0*a4))*
				a5_3 + 3125 * a0_2*a0_2*a5_2*a5_2)) / (16 * temp5_2*temp5_2*temp5_3*temp5_3);

		const char s0 = 0.0 < f0 ? 1 : 0;
		const char s1 = 0.0 < f1 ? 1 : 0;
		const char s2 = 0.0 < f2 ? 1 : 0;
		const char s3 = 0.0 < f3 ? 1 : 0;
		const char s4 = 0.0 < f4 ? 1 : 0;
		const char s5 = 0.0 < f5 ? 1 : 0;

		int c = 0;
		c += s0 != s1 ? 1 : 0;
		c += s1 != s2 ? 1 : 0;
		c += s2 != s3 ? 1 : 0;
		c += s3 != s4 ? 1 : 0;
		c += s4 != s5 ? 1 : 0;

		return c;
	}

	void getSpecificPathHighPrecisionImpl(std::function<void(const Vec2& pos, size_t zIndex)> adder, size_t zIndex, double interval, double permissibleError = 1.0)const
	{
		const Vec2 startPos = m_curve(m_startT);
		adder(startPos, zIndex);

		const auto binarySearchStep = [&](double prevT, double leftBound, double rightBound)->double
		{
			//const bool toRight = prevT < (leftBound + rightBound)*0.5;

			for (int count = 0; count < 100; ++count)
			{
				double newT = (leftBound + rightBound)*0.5;

				const double distance = m_curve.length(Min(prevT, newT), Max(prevT, newT));
				if ((interval - permissibleError) <= distance && distance <= (interval + permissibleError))//OK!
				{
					const Vec2 newPos = m_curve(newT);
					adder(newPos, zIndex);
					return newT;
				}
				else if (distance < interval - permissibleError)//進みが足りない
				{
					//(toRight ? leftBound : rightBound) = newT;
					leftBound = newT;
				}
				else//進みすぎ
				{
					//(toRight ? rightBound : leftBound) = newT;
					rightBound = newT;
				}
			}

			LOG_ERROR(L"収束に失敗");

			return (leftBound + rightBound)*0.5;
		};

		double currentT = m_startT;

		/*
		CurveSegmentは方向を持っているので逆もあり得る
		*/
		if (m_startT < m_endT)
		{
			for (; interval + permissibleError < m_curve.length(currentT, m_endT);)
			{
				currentT = binarySearchStep(currentT, currentT, m_endT);
			}
		}
		else
		{
			for (; interval + permissibleError < m_curve.length(m_endT, currentT);)
			{
				//currentT = binarySearchStep(currentT, m_endT, currentT);
				currentT = binarySearchStep(currentT, currentT, m_endT);
			}
		}
	}

	void init()
	{
		std::deque<SegmentTree::CurvePoint> curvePoints;

		const auto& p0 = m_curve.p0;
		const auto& p1 = m_curve.p1;
		const auto& p2 = m_curve.p2;
		const auto& p3 = m_curve.p3;

		/*
		https://stackoverflow.com/questions/24809978/calculating-the-bounding-box-of-cubic-bezier-curve
		*/
		const Vec2 a = (-3 * p0 + 9 * p1 - 9 * p2 + 3 * p3);
		const Vec2 b = (6 * p0 - 12 * p1 + 6 * p2);
		const Vec2 c = 3 * (p1 - p0);
		const Vec2 d = b*b - 4 * a*c;

		curvePoints.emplace_back(0.0, p0);

		if (0 < d.x)
		{
			const double dSqrt = sqrt(d.x);
			const double t1 = (-b.x + dSqrt) / (2 * a.x);
			const double t2 = (-b.x - dSqrt) / (2 * a.x);
			if (0 < t1&&t1 < 1) {
				curvePoints.emplace_back(t1, m_curve(t1));
			}
			if (0 < t2&&t2 < 1) {
				curvePoints.emplace_back(t2, m_curve(t2));
			}
		}
		if (0 < d.y)
		{
			const double dSqrt = sqrt(d.y);
			const double t1 = (-b.y + dSqrt) / (2 * a.y);
			const double t2 = (-b.y - dSqrt) / (2 * a.y);
			if (0 < t1&&t1 < 1) {
				curvePoints.emplace_back(t1, m_curve(t1));
			}
			if (0 < t2&&t2 < 1) {
				curvePoints.emplace_back(t2, m_curve(t2));
			}
		}

		curvePoints.emplace_back(1.0, p3);

		std::sort(curvePoints.begin(), curvePoints.end(), [](const SegmentTree::CurvePoint& a, const SegmentTree::CurvePoint& b) {return a.first < b.first; });

		m_segmentTree = std::make_shared<SegmentTree>(0.0, 1.0, curvePoints);
	}

	DMat3x2 m_transform = DMat3x2::Identity(), m_transformInv = DMat3x2::Identity();

	std::vector<Vec2> m_linearApprox;

	BezierCurve m_curve;
	std::shared_ptr<SegmentTree> m_segmentTree;
	double m_startT = 0.0, m_endT = 1.0;
};