#pragma once
#include <Siv3D.hpp>
#include "BezierCurve.hpp"

/*
三次ベジェ曲線で構成されたパスの1セグメントについて、他のセグメントとの交差判定を計算するためのクラス
*/
class SegmentTree
{
public:

	using CurvePoint = std::pair<double, Vec2>;

	SegmentTree() = default;

	SegmentTree(double t0, double t1, const std::deque<CurvePoint>& extremums) :
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

	void makeChildRec(const BezierCurve& curve, int maxDepth, int currentDepth = 0)
	{
		if (maxDepth <= currentDepth)
		{
			return;
		}

		makeChild(curve);

		m_childFront->makeChildRec(curve, maxDepth, currentDepth + 1);
		m_childBack->makeChildRec(curve, maxDepth, currentDepth + 1);
	}

	void makeChild(const BezierCurve& curve)
	{
		const double t_m = (m_t0 + m_t1)*0.5;
		const Vec2 pos_m = curve(t_m);
		const auto it = std::upper_bound(m_extremums.cbegin(), m_extremums.cend(), CurvePoint(t_m, Vec2::Zero), [](const CurvePoint& a, const CurvePoint& b) {return a.first < b.first; });

		std::deque<CurvePoint> fronts(m_extremums.cbegin(), it);
		fronts.emplace_back(t_m, pos_m);
		m_childFront = std::make_shared<SegmentTree>(m_t0, t_m, std::move(fronts));

		std::deque<CurvePoint> backs(it, m_extremums.cend());
		backs.emplace_front(t_m, pos_m);
		m_childBack = std::make_shared<SegmentTree>(t_m, m_t1, std::move(backs));
	}

	void draw()const
	{
		m_boundingRect.get().drawFrame(1.0, 0.0, Color(Palette::White, 32));

		if (m_childFront)
		{
			m_childFront->draw();
		}

		if (m_childBack)
		{
			m_childBack->draw();
		}
	}

	std::vector<CurvePoint> intersectionPoints(const BezierCurve& curve, std::shared_ptr<SegmentTree> other, const BezierCurve& other_curve)
	{
		if (!m_boundingRect.intersects(other->m_boundingRect))
		{
			return{};
		}

		std::vector<CurvePoint> result;

		{
			const double permissibleError = 0.1;
			const auto rect1 = m_boundingRect.get();
			const auto rect2 = other->m_boundingRect.get();
			if (rect1.w < permissibleError && rect1.h < permissibleError
				&&rect2.w < permissibleError && rect2.h < permissibleError)
			{
				result.push_back(getCenter(curve));

				//ブーリアン演算には恐らくother側の交差点も使う必要があるはず
				//other->getCenter(other_curve);

				return result;
			}
		}

		if (!m_childFront)
		{
			makeChild(curve);
		}
		if (!other->m_childFront)
		{
			other->makeChild(other_curve);
		}

		const auto result1 = m_childFront->intersectionPoints(curve, other->m_childFront, other_curve);
		const auto result2 = m_childFront->intersectionPoints(curve, other->m_childBack, other_curve);
		const auto result3 = m_childBack->intersectionPoints(curve, other->m_childFront, other_curve);
		const auto result4 = m_childBack->intersectionPoints(curve, other->m_childBack, other_curve);

		result.reserve(result1.size() + result2.size() + result3.size() + result4.size());

		result.insert(result.end(), result1.cbegin(), result1.cend());
		result.insert(result.end(), result2.cbegin(), result2.cend());
		result.insert(result.end(), result3.cbegin(), result3.cend());
		result.insert(result.end(), result4.cbegin(), result4.cend());

		return result;
	}

	CurvePoint getCenter(const BezierCurve& curve)const
	{
		const double t_m = (m_t0 + m_t1)*0.5;
		const Vec2 pos_m = curve(t_m);
		return{ t_m,pos_m };
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

	std::shared_ptr<SegmentTree> m_childFront, m_childBack;
};
