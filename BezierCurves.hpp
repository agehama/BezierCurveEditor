#pragma once
#include <queue>
#include <Siv3D.hpp> // August 2016 v2
#include "include/AnchorPoint.hpp"
#include "include/BoundingRect.hpp"

struct ControlPoints
{
	ControlPoints() = default;

	ControlPoints(const Vec2& p0_, const Vec2& p1_, const Vec2& p2_, const Vec2& p3_) :
		p0(p0_), 
		p1(p1_), 
		p2(p2_), 
		p3(p3_)
	{}

	Vec2 p0, p1, p2, p3;
};

Vec2 curve(const ControlPoints& ps, double t)
{
	return (1 - t)*(1 - t)*(1 - t)*ps.p0 + 3.0*(1 - t)*(1 - t)*t*ps.p1 + 3.0*(1 - t)*t*t*ps.p2 + t*t*t*ps.p3;
}

class SegementTree
{
public:

	using CurvePoint = std::pair<double, Vec2>;

	SegementTree() = default;
	
	SegementTree(double t0, double t1, const std::deque<CurvePoint>& extremums) :
		m_t0(t0),
		m_t1(t1),
		m_extremums(extremums)
	{
		calcBoundingRect();
	}

	const BoundingRect& boundingRect()const
	{
		return m_boundingRect;
	}

	void makeChildren(const ControlPoints& ps, int maxDepth, int currentDepth)
	{
		if (maxDepth <= currentDepth)
		{
			return;
		}

		const double t_m = (m_t0 + m_t1)*0.5;
		const Vec2 pos_m = curve(ps, t_m);
		const auto it = std::upper_bound(m_extremums.cbegin(), m_extremums.cend(), CurvePoint(t_m, Vec2::Zero), [](const CurvePoint& a, const CurvePoint& b) {return a.first < b.first; });

		{
			std::deque<CurvePoint> fronts(m_extremums.cbegin(), it);
			fronts.emplace_back(t_m, pos_m);
			m_childFront = std::make_shared<SegementTree>(m_t0, t_m, std::move(fronts));
		}

		{
			std::deque<CurvePoint> backs(it, m_extremums.cend());
			backs.emplace_front(t_m, pos_m);
			m_childBack = std::make_shared<SegementTree>(t_m, m_t1, std::move(backs));
		}		

		m_childFront->makeChildren(ps, maxDepth, currentDepth + 1);
		m_childBack->makeChildren(ps, maxDepth, currentDepth + 1);
	}

	void draw()const
	{
		m_boundingRect.get().drawFrame();

		if (m_childFront)
		{
			m_childFront->draw();
		}

		if (m_childBack)
		{
			m_childBack->draw();
		}
	}

private:

	void calcBoundingRect()
	{
		m_boundingRect = BoundingRect();

		for (const auto& extremum : m_extremums)
		{
			m_boundingRect.add(extremum.second);
		}
	}

	double m_t0, m_t1;
	std::deque<CurvePoint> m_extremums;
	BoundingRect m_boundingRect;

	std::shared_ptr<SegementTree> m_childFront, m_childBack;
};

class CurveSegment
{
public:

	CurveSegment() = default;

	CurveSegment(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3) :
		ps(p0, p1, p2, p3)
	{
		init();
	}

	/*bool intersects(const CurveSegment& other)const
	{
		return m_boundingRect.intersects(other.m_boundingRect);
	}*/

	void draw()const
	{
		m_segmentTree.draw();
	}

	//ÉÅÉÇâªÇ∑ÇÈÇÃÇ≈constÇ…ÇÕÇ≈Ç´Ç»Ç¢
	std::vector<SegementTree::CurvePoint> intersectionPoints(const CurveSegment& other)
	{
		std::vector<SegementTree::CurvePoint> result;

		const double permissibleError = 1.e-1;

		return result;
	}

private:

	void init()
	{
		std::deque<SegementTree::CurvePoint> curvePoints;

		const auto& p0 = ps.p0;
		const auto& p1 = ps.p1;
		const auto& p2 = ps.p2;
		const auto& p3 = ps.p3;

		const auto getPos = [&](double t) {return (1 - t)*(1 - t)*(1 - t)*p0 + 3.0*(1 - t)*(1 - t)*t*p1 + 3.0*(1 - t)*t*t*p2 + t*t*t*p3; };

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
			const double t1 = (-b.x + sqrt(d.x)) / (2 * a.x);
			const double t2 = (-b.x - sqrt(d.x)) / (2 * a.x);
			if (0 < t1&&t1 < 1) {
				curvePoints.emplace_back(t1, getPos(t1));
			}
			if (0 < t2&&t2 < 1) {
				curvePoints.emplace_back(t2, getPos(t2));
			}
		}
		if (0 < d.y)
		{
			const double t1 = (-b.y + sqrt(d.y)) / (2 * a.y);
			const double t2 = (-b.y - sqrt(d.y)) / (2 * a.y);
			if (0 < t1&&t1 < 1) {
				curvePoints.emplace_back(t1, getPos(t1));
			}
			if (0 < t2&&t2 < 1) {
				curvePoints.emplace_back(t2, getPos(t2));
			}
		}

		curvePoints.emplace_back(1.0, p3);

		std::sort(curvePoints.begin(), curvePoints.end(), [](const SegementTree::CurvePoint& a, const SegementTree::CurvePoint& b) {return a.first < b.first; });

		//m_segmentTree.push_back(Segement(0.0, 1.0, curvePoints));

		//for (int depth = 0; depth < 5; ++depth)
		//{
		//	const size_t currentDepthSize = m_segmentTree.size();
		//	for (size_t i = 0; i < currentDepthSize; ++i)
		//	{
		//		/*const Segement top = m_segmentTree.front();
		//		m_segmentTree.pop();

		//		top.addChild(m_segmentTree, p0, p1, p2, p3);*/

		//		const Segement top = m_segmentTree.front();
		//		m_segmentTree.pop_front();

		//		top.addChild(m_segmentTree, ps);
		//	}
		//}

		m_segmentTree = SegementTree(0.0, 1.0, curvePoints);

		m_segmentTree.makeChildren(ps, 3, 0);
	}

	ControlPoints ps;
	SegementTree m_segmentTree;
};

class BezierCurves
{
public:

	void update()
	{
		if (Input::MouseR.clicked)
		{
			if (2 <= m_anchorPoints.size() && Circle(m_anchorPoints.front().anchorPoint(), AnchorPointRadius()).mouseOver)
			{
				m_isClosed = true;
			}
			
			if (!m_isClosed)
			{
				m_anchorPoints.emplace_back(Mouse::PosF());
			}
		}

		if (Input::MouseR.pressed)
		{
			if (!m_isClosed)
			{
				m_anchorPoints.back().setSymmetricallyB(Mouse::PosF());
			}
		}

		for (auto i : step(m_anchorPoints.size()))
		{
			if (m_anchorPoints[i].updateControlPoint(AnchorPointRadius()))
			{
				break;
			}
		}

		reconstractSegmentInfo();
	}

	void draw()const
	{
		for (const auto& p : m_anchorPoints)
		{
			p.draw(AnchorPointRadius());
		}

		std::vector<Vec2> pp;
		for (size_t i = 0; i + 1 < m_anchorPoints.size() || (m_isClosed && i < m_anchorPoints.size()); ++i)
		{
			const auto l0 = m_anchorPoints[i].outerHandle();
			const auto l1 = m_anchorPoints[(i + 1) % m_anchorPoints.size()].innerHandle();

			const auto& p0 = l0.begin;
			const auto& p1 = l0.end;
			const auto& p2 = l1.begin;
			const auto& p3 = l1.end;

			const auto getPos = [&](double t) {return (1 - t)*(1 - t)*(1 - t)*p0 + 3.0*(1 - t)*(1 - t)*t*p1 + 3.0*(1 - t)*t*t*p2 + t*t*t*p3; };

			//const Vec2 a = (3 * p3 - 9 * p2 + 9 * p1 - 3 * p0);
			const Vec2 a = (-3 * p0 + 9 * p1 - 9 * p2 + 3 * p3);
			const Vec2 b = (6 * p0 - 12 * p1 + 6 * p2);
			const Vec2 c = 3 * (p1 - p0);

			const Vec2 d = b*b - 4 * a*c;

			/*
			https://stackoverflow.com/questions/24809978/calculating-the-bounding-box-of-cubic-bezier-curve
			*/
			BoundingRect boundingRect;
			boundingRect.add(p0);
			boundingRect.add(p3);

			if (0 < d.x)
			{
				const double t1 = (-b.x + sqrt(d.x)) / (2 * a.x);
				const double t2 = (-b.x - sqrt(d.x)) / (2 * a.x);
				if (0 < t1&&t1 < 1) {
					boundingRect.add(getPos(t1));
					//Circle(getPos(t1), controllPointRadius).draw(Color(255, 0, 0, 128));
				}
				if (0 < t2&&t2 < 1) {
					boundingRect.add(getPos(t2));
					//Circle(getPos(t2), controllPointRadius).draw(Color(0, 255, 0, 128));
				}
			}
			if (0 < d.y)
			{
				const double t1 = (-b.y + sqrt(d.y)) / (2 * a.y);
				const double t2 = (-b.y - sqrt(d.y)) / (2 * a.y);
				if (0 < t1&&t1 < 1) {
					boundingRect.add(getPos(t1));
					//Circle(getPos(t1), controllPointRadius).draw(Color(0, 0, 255, 128));
				}
				if (0 < t2&&t2 < 1) {
					boundingRect.add(getPos(t2));
					//Circle(getPos(t2), controllPointRadius).draw(Color(255, 255, 0, 128));
				}
			}
			//boundingRect.get().drawFrame();

			const int divNum = 30;
			for (int p = 0; p < divNum; ++p)
			{
				const double t = 1.0*p / divNum;
				pp.push_back(getPos(t));
			}
		}
				
		if (!pp.empty())
		{
			if (!m_isClosed)
			{
				pp.push_back(m_anchorPoints.back().anchorPoint());
			}

			LineString(pp).draw(Palette::Yellow, m_isClosed);

			//Polygon(pp).draw(Color(Palette::White, 64));

			for (const auto& curveSegment : m_curveSegments)
			{
				curveSegment.draw();
			}
		}
	}

	bool calcIntersection()
	{
		std::vector<BoundingRect> boundingRects;
		std::vector<char> intersections;
		for (size_t i = 0; i + 1 < m_anchorPoints.size() || (m_isClosed && i < m_anchorPoints.size()); ++i)
		{
			const auto l0 = m_anchorPoints[i].outerHandle();
			const auto l1 = m_anchorPoints[(i + 1) % m_anchorPoints.size()].innerHandle();

			const auto& p0 = l0.begin;
			const auto& p1 = l0.end;
			const auto& p2 = l1.begin;
			const auto& p3 = l1.end;

			const auto getPos = [&](double t) {return (1 - t)*(1 - t)*(1 - t)*p0 + 3.0*(1 - t)*(1 - t)*t*p1 + 3.0*(1 - t)*t*t*p2 + t*t*t*p3; };

			const Vec2 a = (-3 * p0 + 9 * p1 - 9 * p2 + 3 * p3);
			const Vec2 b = (6 * p0 - 12 * p1 + 6 * p2);
			const Vec2 c = 3 * (p1 - p0);

			const Vec2 d = b*b - 4 * a*c;

			/*
			https://stackoverflow.com/questions/24809978/calculating-the-bounding-box-of-cubic-bezier-curve
			*/
			BoundingRect boundingRect;
			boundingRect.add(p0);
			boundingRect.add(p3);

			if (0 < d.x)
			{
				const double t1 = (-b.x + sqrt(d.x)) / (2 * a.x);
				const double t2 = (-b.x - sqrt(d.x)) / (2 * a.x);
				if (0 < t1&&t1 < 1) {
					boundingRect.add(getPos(t1));
				}
				if (0 < t2&&t2 < 1) {
					boundingRect.add(getPos(t2));
				}
			}
			if (0 < d.y)
			{
				const double t1 = (-b.y + sqrt(d.y)) / (2 * a.y);
				const double t2 = (-b.y - sqrt(d.y)) / (2 * a.y);
				if (0 < t1&&t1 < 1) {
					boundingRect.add(getPos(t1));
				}
				if (0 < t2&&t2 < 1) {
					boundingRect.add(getPos(t2));
				}
			}
			boundingRect.get().drawFrame();

			boundingRects.push_back(boundingRect);
			intersections.push_back(false);
		}

		bool anyIntersection = false;
		for (int i = 0; i < boundingRects.size(); ++i)
		{
			for (int j = i + 1; j < boundingRects.size(); ++j)
			{
				if (boundingRects[i].intersects(boundingRects[j]))
				{
					intersections[i] = true;
					intersections[j] = true;
					anyIntersection = true;
				}
			}
		}
		Window::SetTitle(anyIntersection);

		ClearPrint();

		for (int i = 0; i < boundingRects.size(); ++i)
		{
			Println(i, L" : ", boundingRects[i].toString());
			
			boundingRects[i].get().drawFrame(1.0, 0.0, intersections[i] ? Palette::Red : Palette::White);
		}
	}

private:

	void reconstractSegmentInfo()
	{
		m_curveSegments.clear();

		std::vector<char> intersections;
		for (size_t i = 0; i + 1 < m_anchorPoints.size() || (m_isClosed && i < m_anchorPoints.size()); ++i)
		{
			const auto l0 = m_anchorPoints[i].outerHandle();
			const auto l1 = m_anchorPoints[(i + 1) % m_anchorPoints.size()].innerHandle();

			m_curveSegments.emplace_back(l0.begin, l0.end, l1.begin, l1.end);

			intersections.push_back(false);
		}

		/*bool anyIntersection = false;
		for (int i = 0; i < m_curveSegments.size(); ++i)
		{
			for (int j = i + 1; j < m_curveSegments.size(); ++j)
			{
				if (m_curveSegments[i].intersects(m_curveSegments[j]))
				{
					intersections[i] = true;
					intersections[j] = true;
					anyIntersection = true;
				}
			}
		}
		Window::SetTitle(anyIntersection);

		ClearPrint();*/

		/*for (int i = 0; i < m_curveSegments.size(); ++i)
		{
			Println(i, L" : ", boundingRects[i].toString());

			boundingRects[i].get().drawFrame(1.0, 0.0, intersections[i] ? Palette::Red : Palette::White);
		}*/
	}

	static double AnchorPointRadius()
	{
		return 5.0;
	}

	std::vector<AnchorPoint> m_anchorPoints;
	std::vector<CurveSegment> m_curveSegments;
	bool m_isClosed = false;
};