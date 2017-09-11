#pragma once
#include <Siv3D.hpp>
#include "SegmentTree.hpp"
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

	CurveSegment(const BezierCurve& bezierCurve, double startT = 0.0, double endT = 1.0) :
		m_curve(bezierCurve),
		m_startT(startT),
		m_endT(endT)
	{
		init();
	}

	void setLine(const Vec2& p0, const Vec2& p1)
	{
		m_curve = BezierCurve(p0, p0, p1, p1);
	}

	void draw()const
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

	const BezierCurve& curve()const
	{
		return m_curve;
	}

	void setRange(double startT, double endT)
	{
		m_startT = startT;
		m_endT = endT;
	}

	std::pair<double, double> range()const
	{
		return{ m_startT,m_endT };
	}

	void getSpecificPathHighPrecision(ClipperLib::Path& output, size_t zIndex, double interval, double permissibleError = 1.0)const
	{
		const auto adder = [&output](const Vec2& pos, size_t zIndex)
		{
			output << ClipperLib::IntPoint(pos.x * scaleInt, pos.y * scaleInt, zIndex);
		};

		getSpecificPathHighPrecisionImpl(adder, zIndex, interval, permissibleError);
	}

	void getSpecificPathHighPrecision(std::vector<Vec2>& output, double interval, double permissibleError = 1.0)const
	{
		const auto adder = [&output](const Vec2& pos, size_t zIndex)
		{
			output.push_back(pos);
		};

		getSpecificPathHighPrecisionImpl(adder, 0, interval, permissibleError);
	}

	/*
	void getSpecificPathHighPrecision(ClipperLib::Path& output, size_t zIndex, double interval, double permissibleError = 1.0)const
	{	
		const Vec2 startPos = m_curve(m_startT);
		output << ClipperLib::IntPoint(startPos.x * scaleInt, startPos.y * scaleInt, zIndex);

		const auto binarySearchStep = [&](double prevT, double leftBound, double rightBound)->double
		{
			for (int count = 0; count < 100; ++count)
			{
				double newT = (leftBound + rightBound)*0.5;

				const double distance = m_curve.length(prevT, newT);
				if ((interval - permissibleError) <= distance && distance <= (interval + permissibleError))//OK!
				{
					const Vec2 newPos = m_curve(newT);
					output << ClipperLib::IntPoint(newPos.x * scaleInt, newPos.y * scaleInt, zIndex);
					return newT;
				}
				else if (distance < interval - permissibleError)//足りない
				{
					leftBound = newT;
				}
				else//進みすぎ
				{
					rightBound = newT;
				}
			}

			LOG_ERROR(L"収束に失敗");

			return (leftBound + rightBound)*0.5;
		};
		
		double currentT = m_startT;
		for (; interval + permissibleError < m_curve.length(currentT, m_endT);)
		{
			currentT = binarySearchStep(currentT, currentT, m_endT);
		}
	}
	*/

private:

	void getSpecificPathHighPrecisionImpl(std::function<void(const Vec2& pos, size_t zIndex)> adder, size_t zIndex, double interval, double permissibleError = 1.0)const
	{
		const Vec2 startPos = m_curve(m_startT);
		adder(startPos, zIndex);

		const auto binarySearchStep = [&](double prevT, double leftBound, double rightBound)->double
		{
			for (int count = 0; count < 100; ++count)
			{
				double newT = (leftBound + rightBound)*0.5;

				const double distance = m_curve.length(prevT, newT);
				if ((interval - permissibleError) <= distance && distance <= (interval + permissibleError))//OK!
				{
					const Vec2 newPos = m_curve(newT);
					adder(newPos, zIndex);
					return newT;
				}
				else if (distance < interval - permissibleError)//足りない
				{
					leftBound = newT;
				}
				else//進みすぎ
				{
					rightBound = newT;
				}
			}

			LOG_ERROR(L"収束に失敗");

			return (leftBound + rightBound)*0.5;
		};

		double currentT = m_startT;
		for (; interval + permissibleError < m_curve.length(currentT, m_endT);)
		{
			currentT = binarySearchStep(currentT, currentT, m_endT);
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

	BezierCurve m_curve;
	std::shared_ptr<SegmentTree> m_segmentTree;
	double m_startT = 0.0, m_endT = 1.0;
};