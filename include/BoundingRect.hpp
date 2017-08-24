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

	/*
	‹«ŠEüã‚Å‚ÍŒð·‚µ‚Ä‚¢‚È‚¢‚Æ‚¢‚¤”»’è‚ðŒµ–§‚É‚µ‚½‚¢‚½‚ßAŽl‘¥‰‰ŽZ‚ðŽg‚í‚¸‚ÉÕ“Ë”»’è‚ðs‚¤
	*/
	bool intersects(const BoundingRect& other)const
	{
		return Max(m_min.x, other.m_min.x) < Min(m_max.x, other.m_max.x)
			&& Max(m_min.y, other.m_min.y) < Min(m_max.y, other.m_max.y);
	}

	bool includes(const Vec2& point)const
	{
		return m_min.x < point.x && point.x < m_max.x
			&& m_min.y < point.y && point.y < m_max.y;
	}

	String toString()const
	{
		return Format(m_min, L", ", m_max);
	}

private:

	Vec2 m_min = Vec2(DBL_MAX, DBL_MAX);
	Vec2 m_max = Vec2(-DBL_MAX, -DBL_MAX);
};