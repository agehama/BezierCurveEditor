# include <Siv3D.hpp> // August 2016 v2

class BoundingRect
{
public:

	void add(const Vec2& v)
	{
		if (v.x < m_min.x)
		{
			m_min.x = v.x;
		}
		if (v.y < m_min.y)
		{
			m_min.y = v.y;
		}
		if (m_max.x < v.x)
		{
			m_max.x = v.x;
		}
		if (m_max.y < v.y)
		{
			m_max.y = v.y;
		}
	}

	RectF get()const
	{
		return RectF(m_min, m_max - m_min);
	}

private:

	Vec2 m_min = Vec2(DBL_MAX, DBL_MAX);
	Vec2 m_max = Vec2(-DBL_MAX, -DBL_MAX);
};

class ControlPoint
{
public:

	ControlPoint(const Vec2& pos = Mouse::PosF()) :
		m_points({ pos, pos, pos })
	{}

	void setSymmetricallyA(const Vec2& pos)
	{
		controlPointA() = pos;
		controlPointB() = anchorPoint().movedBy(anchorPoint() - controlPointA());
	}

	void setSymmetricallyB(const Vec2& pos)
	{
		controlPointB() = pos;
		controlPointA() = anchorPoint().movedBy(anchorPoint() - controlPointB());
	}
	
	Vec2& controlPointA() { return m_points[0]; }

	const Vec2& controlPointA()const { return m_points[0]; }

	Vec2& anchorPoint() { return m_points[1]; }

	const Vec2& anchorPoint()const { return m_points[1]; }

	Vec2& controlPointB() { return m_points[2]; }

	const Vec2& controlPointB()const { return m_points[2]; }

	Vec2& operator[](size_t i) { return m_points[i]; }
	
	const Vec2& operator[](size_t i)const { return m_points[i]; }

	size_t size()const { return m_points.size(); }

	const Line innerHandle()const
	{
		return Line(controlPointA(), anchorPoint());
	}

	const Line outerHandle()const
	{
		return Line(anchorPoint(), controlPointB());
	}

	bool updateControlPoint(double radius)
	{
		if (!m_grabbingIndex && !Input::MouseL.clicked)
		{
			return false;
		}

		if (!m_grabbingIndex)
		{
			m_grabbingIndex = mouseOverIndex(radius);
			if (!m_grabbingIndex)
			{
				return false;
			}
		}
		else
		{
			if (m_grabbingIndex)
			{
				if (m_grabbingIndex.value() == 1)
				{
					m_points[0] += Mouse::DeltaF();
					m_points[1] += Mouse::DeltaF();
					m_points[2] += Mouse::DeltaF();
				}
				//制御点を個別に動かす
				else if (Input::KeyAlt.pressed)
				{
					m_points[m_grabbingIndex.value()] = Mouse::PosF();
				}
				//制御点を対称に動かす
				else
				{
					m_grabbingIndex.value() == 0 ? setSymmetricallyA(Mouse::PosF()) : setSymmetricallyB(Mouse::PosF());
				}
			}

			if (Input::MouseL.released)
			{
				m_grabbingIndex = none;
			}
		}		

		return true;
	}

	Optional<size_t> mouseOverIndex(double radius)const
	{
		const Vec2 mousePos = Mouse::PosF();
		const double radius2 = radius*radius;

		for (auto i : step(m_points.size()))
		{
			if (mousePos.distanceFromSq(m_points[i]) < radius2)
			{
				return i;
			}
		}

		return none;
	}

	void draw(double radius)const
	{
		innerHandle().draw();
		outerHandle().draw();
		Circle(controlPointA(), radius*0.5).draw(Palette::Cyan);
		Circle(anchorPoint(), radius).drawFrame(1.0, 1.0, Palette::Yellow);
		Circle(controlPointB(), radius*0.5).draw(Palette::Cyan);
	}

private:

	std::array<Vec2, 3> m_points;
	Optional<size_t> m_grabbingIndex;
};

class BezierCurve
{
public:

	void update()
	{
		if (Input::MouseR.clicked)
		{
			m_controllPoints.emplace_back(Mouse::PosF());
		}

		if (Input::MouseR.pressed)
		{
			m_controllPoints.back().setSymmetricallyB(Mouse::PosF());
		}

		for (auto i : step(m_controllPoints.size()))
		{
			if (m_controllPoints[i].updateControlPoint(ControllPointRadius()))
			{
				break;
			}
		}
	}

	void draw()const
	{
		for (const auto& p : m_controllPoints)
		{
			p.draw(ControllPointRadius());
		}

		std::vector<Vec2> pp;
		for (size_t i = 0; i + 1 < m_controllPoints.size(); ++i)
		{
			const auto l0 = m_controllPoints[i].outerHandle();
			const auto l1 = m_controllPoints[i + 1].innerHandle();

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

	static double ControllPointRadius()
	{
		return 5.0;
	}

	std::vector<ControlPoint> m_controllPoints;
};

void Main()
{
	BezierCurve curve;

	while (System::Update())
	{		
		curve.update();
		curve.draw();
	}
}
