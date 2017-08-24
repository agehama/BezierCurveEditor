#pragma once
#include <Siv3D.hpp> // August 2016 v2
#include "include/AnchorPoint.hpp"
#include "include/BoundingRect.hpp"

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

			const Vec2 a = (3 * p3 - 9 * p2 + 9 * p1 - 3 * p0);
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
				const double t = 1.0*p / (divNum - 1);
				pp.push_back(getPos(t));
			}
		}

		if (!pp.empty())
		{
			LineString(pp).draw(Palette::Yellow);
		}
	}

private:

	static double AnchorPointRadius()
	{
		return 5.0;
	}

	std::vector<AnchorPoint> m_anchorPoints;
	bool m_isClosed = false;
};