#pragma once
#include <Siv3D.hpp>

class AnchorPoint
{
public:

	AnchorPoint(const Vec2& pos = Mouse::PosF()) :
		m_points({ pos, pos, pos })
	{}

	AnchorPoint(const Vec2& posA, const Vec2& posAnchor, const Vec2& posB) :
		m_points({ posA, posAnchor, posB })
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

	Line innerHandle()const
	{
		return Line(controlPointA(), anchorPoint());
	}

	Line outerHandle()const
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
				//êßå‰ì_Çå¬ï Ç…ìÆÇ©Ç∑
				else if (Input::KeyAlt.pressed)
				{
					m_points[m_grabbingIndex.value()] = Mouse::PosF();
				}
				//êßå‰ì_ÇëŒèÃÇ…ìÆÇ©Ç∑
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

	void draw(double radius, int alpha)const
	{
		/*
		innerHandle().draw();
		outerHandle().draw();
		Circle(controlPointA(), radius*0.5).draw(Palette::Cyan);
		Circle(anchorPoint(), radius).drawFrame(1.0, 1.0, Palette::Yellow);
		Circle(controlPointB(), radius*0.5).draw(Palette::Cyan);
		*/

		/*innerHandle().draw(Color(237, 237, 245));
		outerHandle().draw(Color(237, 237, 245));
		Circle(controlPointA(), radius*0.5).draw(Color(112, 86, 151));
		Circle(anchorPoint(), radius).drawFrame(1.0, 1.0, Color(112, 86, 151));
		Circle(controlPointB(), radius*0.5).draw(Color(112, 86, 151));*/

		innerHandle().draw(Color(170, 170, 170, alpha));
		outerHandle().draw(Color(170, 170, 170, alpha));
		Circle(controlPointA(), radius*0.5).draw(Color(112, 86, 151, alpha));
		Circle(anchorPoint(), radius).drawFrame(1.0, 1.0, Color(112, 86, 151, alpha));
		Circle(controlPointB(), radius*0.5).draw(Color(112, 86, 151, alpha));
	}

private:

	std::array<Vec2, 3> m_points;
	Optional<size_t> m_grabbingIndex;
};