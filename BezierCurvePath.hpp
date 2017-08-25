#pragma once
#include <queue>
#include <Siv3D.hpp> // August 2016 v2
#include "include/AnchorPoint.hpp"
#include "include/BoundingRect.hpp"
#include "include/BezierCurve.hpp"
#include "include/CurveSegment.hpp"

class BezierCurvePath
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
			if (m_anchorPoints[i].updateControlPoint(AnchorPointRadius()*2))
			{
				break;
			}
		}

		constractCurveSegments();
		calcIntersections();
	}

	void draw()const
	{
		for (const auto& p : m_anchorPoints)
		{
			p.draw(AnchorPointRadius());
		}

		std::vector<Vec2> pp;
		for (const auto& segment : m_curveSegments)
		{
			const auto& curve = segment.curve();

			const int divNum = 30;
			for (int p = 0; p < divNum; ++p)
			{
				const double t = 1.0*p / divNum;
				pp.push_back(curve(t));
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

private:

	void calcIntersections()
	{
		int intersectionCount = 0;

		for (size_t i = 0; i < m_curveSegments.size(); ++i)
		{
			for (size_t j = i + 1; j < m_curveSegments.size(); ++j)
			{
				const auto result = m_curveSegments[i].intersectionPoints(m_curveSegments[j]);
				
				intersectionCount += result.size();

				for (const auto& point : result)
				{
					Circle(point.second, 5).draw(Palette::Red);
				}
			}

			if (auto t = m_curveSegments[i].selfIntersection())
			{
				++intersectionCount;

				const Vec2 pos = m_curveSegments[i].curve().get(t.value().first);
				Circle(pos, AnchorPointRadius()).draw(Color(255, 255, 0));
			}
		}

		Window::SetTitle(intersectionCount);
	}

	/*
	アンカーポイント -> セグメントへの変換
	*/
	void constractCurveSegments()
	{
		m_curveSegments.clear();

		for (size_t i = 0; i + 1 < m_anchorPoints.size() || (m_isClosed && i < m_anchorPoints.size()); ++i)
		{
			const auto l0 = m_anchorPoints[i].outerHandle();
			const auto l1 = m_anchorPoints[(i + 1) % m_anchorPoints.size()].innerHandle();

			m_curveSegments.emplace_back(l0.begin, l0.end, l1.begin, l1.end);
		}
	}

	static double AnchorPointRadius()
	{
		return 5.0;
	}

	std::vector<AnchorPoint> m_anchorPoints;
	std::vector<CurveSegment> m_curveSegments;
	bool m_isClosed = false;
};