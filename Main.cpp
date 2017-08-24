# include <Siv3D.hpp>

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

void Main()
{
	const double controllPointRadius = 15.0;
	std::vector<Vec2> controllPoints;
	Optional<size_t> grabbing;

	while (System::Update())
	{
		if (Input::MouseR.clicked)
		{
			controllPoints.push_back(Mouse::Pos());
		}

		{
			if (Input::MouseL.clicked)
			{
				for (auto i : step(controllPoints.size()))
				{
					if (Circle(controllPoints[i], controllPointRadius).intersects(Mouse::Pos()))
					{
						grabbing = i;
						break;
					}
				}
			}

			if (Input::MouseL.released)
			{
				grabbing = none;
			}

			if (grabbing)
			{
				controllPoints[grabbing.value()].moveBy(Mouse::DeltaF());
			}
		}
		
		for (const auto& p : controllPoints)
		{
			Circle(p, controllPointRadius).draw(Color(Palette::Cyan, 64));
		}
		if (!controllPoints.empty())
		{
			LineString(controllPoints).draw();
		}

		
		std::vector<Vec2> pp;
		for (size_t i = 0; i + 3 < controllPoints.size(); i += 3)
		{
			auto& p0 = controllPoints[i];
			auto& p1 = controllPoints[i + 1];
			auto& p2 = controllPoints[i + 2];
			auto& p3 = controllPoints[i + 3];
			if (i != 0)
			{
				p0 = (controllPoints[i - 1] + p1)*0.5;
			}
			if (i + 4 < controllPoints.size())
			{
				p3 = (controllPoints[i + 2] + controllPoints[i + 4])*0.5;
			}

			const auto getPos = [&](double t) {return (1 - t)*(1 - t)*(1 - t)*p0 + 3.0*(1 - t)*(1 - t)*t*p1 + 3.0*(1 - t)*t*t*p2 + t*t*t*p3; };

			const Vec2 a = (3 * p3 - 9 * p2 + 9 * p1 - 3 * p0);
			const Vec2 b = (6*p0 - 12*p1 + 6*p2);
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
			boundingRect.get().drawFrame();

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
}
