#pragma once
#include <Siv3D.hpp>

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