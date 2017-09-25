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

	void setSymmetricallyA(const Vec2& screenPos)
	{
		controlPointA_Raw() = m_transformInv.transform(screenPos);
		controlPointB_Raw() = anchorPoint_Raw().movedBy(anchorPoint_Raw() - controlPointA_Raw());
	}

	void setSymmetricallyB(const Vec2& screenPos)
	{
		controlPointB_Raw() = m_transformInv.transform(screenPos);
		controlPointA_Raw() = anchorPoint_Raw().movedBy(anchorPoint_Raw() - controlPointB_Raw());
	}


	Vec2& controlPointA_Raw() { return m_points[0]; }

	Vec2& anchorPoint_Raw() { return m_points[1]; }

	Vec2& controlPointB_Raw() { return m_points[2]; }

	const Vec2& controlPointA_Raw()const { return m_points[0]; }

	const Vec2& anchorPoint_Raw()const { return m_points[1]; }

	const Vec2& controlPointB_Raw()const { return m_points[2]; }

	Vec2 controlPointA_Screen()const { return m_transform.transform(m_points[0]); }

	Vec2 anchorPoint_Screen()const { return m_transform.transform(m_points[1]); }

	Vec2 controlPointB_Screen()const { return m_transform.transform(m_points[2]); }


	Line innerHandle_Raw()const { return Line(controlPointA_Raw(), anchorPoint_Raw()); }

	Line outerHandle_Raw()const { return Line(anchorPoint_Raw(), controlPointB_Raw()); }

	Line innerHandle_Screen()const { return Line(controlPointA_Screen(), anchorPoint_Screen()); }

	Line outerHandle_Screen()const { return Line(anchorPoint_Screen(), controlPointB_Screen()); }


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
					const Vec2 absDelta = directionScrToAbs(Mouse::DeltaF());
					m_points[0] += absDelta;
					m_points[1] += absDelta;
					m_points[2] += absDelta;
				}
				//êßå‰ì_Çå¬ï Ç…ìÆÇ©Ç∑
				else if (Input::KeyAlt.pressed)
				{
					m_points[m_grabbingIndex.value()] = m_transformInv.transform(Mouse::PosF());
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
		const Vec2 mousePos = m_transformInv.transform(Mouse::PosF());

		const double acsScale = scaleScrToAbs(radius);
		const double radius2 = acsScale*acsScale;

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

		innerHandle_Screen().draw(Color(170, 170, 170, alpha));
		outerHandle_Screen().draw(Color(170, 170, 170, alpha));
		Circle(controlPointA_Screen(), radius*0.5).draw(Color(112, 86, 151, alpha));
		Circle(anchorPoint_Screen(), radius).drawFrame(1.0, 1.0, Color(112, 86, 151, alpha));
		Circle(controlPointB_Screen(), radius*0.5).draw(Color(112, 86, 151, alpha));
	}

	const Mat3x2& transform()const { return m_transform; }
	const Mat3x2& transformInv()const { return m_transformInv; }

	void setTransform(const Mat3x2& transform, const Mat3x2& transformInv)
	{
		m_transform = transform;
		m_transformInv = transformInv;
	}

private:

	Vec2 directionScrToAbs(const Vec2& screenDirection)const
	{
		const Vec2 p0 = m_transformInv.transform(Vec2(0, 0));
		const Vec2 p1 = m_transformInv.transform(Vec2(0, 0) + screenDirection);
		return p1 - p0;
	}

	double scaleScrToAbs(double screenScale)const
	{
		const Vec2 p0 = m_transformInv.transform(Vec2(0, 0));
		const Vec2 p1 = m_transformInv.transform(Vec2(1, 0));
		return (p1 - p0).length()*screenScale;
	}
	
	//size_t size()const { return m_points.size(); }

	//const Vec2& operator[](size_t i)const { return m_transform.transform(m_points[i]); }

	Mat3x2 m_transform = Mat3x2::Identity(), m_transformInv = Mat3x2::Identity();

	std::array<Vec2, 3> m_points;
	Optional<size_t> m_grabbingIndex;
};