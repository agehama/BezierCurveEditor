#pragma once
#include <fstream>
#include "clipper/clipper.hpp"
#include "CurveSegment.hpp"

enum ClippingRelation {
	AdjacentInner, AdjacentOuter, Contain, Distant
};

ClippingRelation CalcRetation(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB);

struct AdditionalInfo
{
	std::vector<CurveSegment> additionalSegments;
};

//polygonA - polygonB
std::pair<ClipperLib::Paths, Optional<AdditionalInfo>> PolygonSubtract(const ClipperLib::Path& polygonA, const ClipperLib::Path& polygonB, size_t& newSegmentIndex);
ClipperLib::Paths PolygonSubtract2(const ClipperLib::Path& polygonA, const ClipperLib::Path& polygonB);

bool IsAppliableGraph(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB);

#define OUTPUT_LOG

#ifdef OUTPUT_LOG
#define TEST_LOG(...) LOG(__VA_ARGS__)
#else
#define TEST_LOG(...)
#endif

struct GraphNode
{
	ClipperLib::IntPoint m_pos;
	char m_isEnabledVertex;
};

inline char True()
{
	return 1;
}

inline char False()
{
	return 0;
}

inline bool IsClockWise(const std::vector<ClipperLib::IntPoint>& closedPath)
{
	double sum = 0;

	for (int i = 0; i < closedPath.size(); ++i)
	{
		const auto& p1 = closedPath[i];
		const auto& p2 = closedPath[(i + 1) % closedPath.size()];

		sum += (p2.X - p1.X)*(p2.Y + p1.Y);
	}

	return sum < 0.0;
}

inline Vec2 ToVec2(const ClipperLib::IntPoint& v)
{
	return Vec2(static_cast<double>(v.X), static_cast<double>(v.Y));
}

inline Line ToLine(const ClipperLib::IntPoint& pA, const ClipperLib::IntPoint& pB)
{
	return Line(pA.X, pA.Y, pB.X, pB.Y);
}

inline Polygon ToPoly(const std::vector<ClipperLib::IntPoint>& poly)
{
	std::vector<Vec2> polyVec(poly.size());

	if (poly.empty())
	{
		return Polygon();
	}

	if (IsClockWise(poly))
	{
		for (size_t i = 0; i < poly.size(); ++i)
		{
			polyVec[i] = ToVec2(poly[i]);
		}
	}
	else
	{
		for (int i = static_cast<int>(poly.size()) - 1; 0 <= i; --i)
		{
			polyVec[i] = ToVec2(poly[i]);
		}
	}

	return Polygon(polyVec);
}

inline bool IsSamePos(const ClipperLib::IntPoint& pA, const ClipperLib::IntPoint& pB)
{
	return pA.X == pB.X && pA.Y == pB.Y;
}

inline bool Includes(const std::vector<ClipperLib::IntPoint>& poly, const ClipperLib::IntPoint& pA)
{
	for (auto& point : poly)
	{
		if (IsSamePos(point, pA))
		{
			return true;
		}
	}

	return false;
}

inline ClipperLib::IntPoint ToIntPoint(const Vec2& v, ClipperLib::cInt z)
{
	//return{ v.x,v.y,z };
	return{ static_cast<ClipperLib::cInt>(v.x),static_cast<ClipperLib::cInt>(v.y),z };
}

inline ClipperLib::cInt MergeIndex(ClipperLib::cInt a, ClipperLib::cInt b)
{
	ClipperLib::cInt c = 0;
	c += a;
	c += (b << 32);
	return c;
}

inline void ZFillFunc(const ClipperLib::IntPoint& e1bot, const ClipperLib::IntPoint& e1top, const ClipperLib::IntPoint& e2bot, const ClipperLib::IntPoint& e2top, ClipperLib::IntPoint& pt)
{
	//intersectionPoints.push_back(pt);
	pt.Z = 0;
	pt.Z += e1bot.Z;
	pt.Z += (e2bot.Z << 32);
}

inline ClipperLib::IntPoint MidPoint(const ClipperLib::IntPoint& p1, const ClipperLib::IntPoint& p2)
{
	if (p1.Z == p2.Z)
	{
		return ClipperLib::IntPoint((p1.X + p2.X)*0.5, (p1.Y + p2.Y)*0.5, p1.Z);
	}
	LOG_ERROR(L"L(", __LINE__, L"): p1.Z != p2.Z");
	return ClipperLib::IntPoint((p1.X + p2.X)*0.5, (p1.Y + p2.Y)*0.5, p1.Z);
}

class ShortestPathTree;
using SPTree = std::shared_ptr<ShortestPathTree>;

//　ShortestPathTreeの中で、最小パスを表すノードのマップ
//　ShortestPathTreeに同じインデックスが二度現れないようにするために使う
using ShortestNodeMap = std::map<size_t, SPTree>;

class ShortestPathTree
{
public:

	void initRoot(size_t index, std::shared_ptr<ShortestNodeMap> shortestNodeMap)
	{
		m_index = index;
		m_shortestNodeMap = shortestNodeMap;
		m_parent = std::weak_ptr<ShortestPathTree>();
	}

	void init(size_t index, std::shared_ptr<ShortestNodeMap> shortestNodeMap, std::weak_ptr<ShortestPathTree> parent, SPTree smartThis)
	{
		m_index = index;
		m_shortestNodeMap = shortestNodeMap;
		m_parent = parent;

		(*m_shortestNodeMap)[m_index] = smartThis;
	}

	void calcCandidates(const std::vector<char>& currentMovableList, std::function<double(size_t, size_t)> costFunction)
	{
		TEST_LOG(L"calcCandidates: ");
		for (auto nextIndex : step(currentMovableList.size()))
		{
			if (currentMovableList[nextIndex] == True())
			{
				m_toSearchIndices.push_back(nextIndex);
				m_toSearchTotalCost.push_back(costFunction(m_index, nextIndex));

				TEST_LOG(L"cost of P(", m_index, L") -> P(", m_toSearchIndices.back(), L"): ", m_toSearchTotalCost.back());
			}
		}
	}

	SPTree addChild(size_t toSearchIndex, SPTree smartThis)
	{
		m_pTree.push_back(std::make_shared<ShortestPathTree>());
		auto& child = *m_pTree.back();

		//child.init(m_toSearchIndices[toSearchIndex], m_shortestNodeMap, smartThis, m_pTree.back());
		//インテリセンスの赤線回避
		child.init(m_toSearchIndices[toSearchIndex], m_shortestNodeMap, static_cast<std::weak_ptr<ShortestPathTree>>(smartThis), m_pTree.back());

		return m_pTree.back();
	}

	std::pair<SPTree, std::pair<size_t, double>> globalMinimumCostPath(double parentCost, SPTree smartThis)
	{
		if (m_pTree.empty())
		{
			TEST_LOG(L"globalMinimumCostPath: ");
			if (auto pOpt = minimumCostPath())
			{
#ifdef OUTPUT_LOG
				//debugPrint();
#endif
				TEST_LOG(L"m_shortestNodeMap has ", m_toSearchIndices[pOpt.value().first], L" ?");
				if (m_shortestNodeMap->find(m_toSearchIndices[pOpt.value().first]) == m_shortestNodeMap->end())
				{
					TEST_LOG(L"minimumCostPath is: ", Vec2(pOpt.value().first, parentCost + pOpt.value().second));
					return std::pair<SPTree, std::pair<size_t, double>>(smartThis, { pOpt.value().first, parentCost + pOpt.value().second });
				}
			}
			TEST_LOG(L"minimumCostPath not found: ");

			return std::pair<SPTree, std::pair<size_t, double>>(nullptr, { 0, DBL_MAX });
		}
		else
		{
			SPTree resultTree = nullptr;
			size_t resultIndex = 0;
			double resultCost = DBL_MAX;

			for (auto pTree : m_pTree)
			{
				auto current = pTree->globalMinimumCostPath(parentCost, pTree);

				if (current.second.second < resultCost)
				{
					resultTree = current.first;
					resultIndex = current.second.first;
					resultCost = current.second.second;
				}
			}

			return std::pair<SPTree, std::pair<size_t, double>>(resultTree, { resultIndex, resultCost });
		}
	}

	size_t index()const
	{
		return m_index;
	}

	std::vector<size_t> toRoot()const
	{
		std::vector<size_t> result;

		toRootImpl(result);

		return result;
	}

private:

	void debugPrint()
	{
		TEST_LOG(L"debugPrint: ");
		for (auto p : *m_shortestNodeMap)
		{
			TEST_LOG(p.first, L", ", p.second->index());
		}
	}

	void toRootImpl(std::vector<size_t>& path)const
	{
		path.push_back(m_index);

		if (auto parent = m_parent.lock())
		{
			parent->toRootImpl(path);
		}
	}

	Optional<std::pair<size_t, double>> minimumCostPath()const
	{
		TEST_LOG(L"minimumCostPath: ");
		if (m_toSearchTotalCost.empty())
		{
			TEST_LOG(L"____result = none: m_toSearchTotalCost.empty()");
			return none;
		}

		size_t minCostIndex = 0;
		double minCost = DBL_MAX;
		for (size_t i = 0; i < m_toSearchTotalCost.size(); ++i)
		{
			if (auto parent = m_parent.lock())
			{
				//後戻り禁止
				if (m_toSearchIndices[i] == parent->index())
				{
					continue;
				}
			}

			if (m_toSearchTotalCost[i] < minCost)
			{
				minCost = m_toSearchTotalCost[i];
				minCostIndex = i;
			}
		}

		if (minCost != DBL_MAX)
		{
			TEST_LOG(L"____result = ", Vec2(minCostIndex, minCost));
			return std::pair<size_t, double>(minCostIndex, minCost);
		}

		TEST_LOG(L"____result = none: minCost == DBL_MAX");
		return none;
	}

	std::shared_ptr<ShortestNodeMap> m_shortestNodeMap;
	std::vector<SPTree> m_pTree;
	std::weak_ptr<ShortestPathTree> m_parent;
	size_t m_index;
	std::vector<size_t> m_toSearchIndices;
	std::vector<double> m_toSearchTotalCost;
};

class VerticesGraph
{
public:

	size_t addNode(const GraphNode& node)
	{
		m_nodes.push_back(node);

		for (auto& line : m_adjacencyMatrix)
		{
			line.push_back(False());
		}

		m_adjacencyMatrix.push_back(std::vector<char>(m_nodes.size(), False()));

		//自分へのリンクは常に無効とする（ダイクストラ法でループを探すため）
		m_adjacencyMatrix.back().back() = False();

		return m_adjacencyMatrix.size() - 1;
	}

	size_t copyNodeWithLink(size_t nodeIndex)
	{
		m_nodes.push_back(m_nodes[nodeIndex]);

		for (auto& line : m_adjacencyMatrix)
		{
			line.push_back(line[nodeIndex]);
		}

		m_adjacencyMatrix.push_back(m_adjacencyMatrix[nodeIndex]);

		m_adjacencyMatrix.back().back() = False();

		return m_adjacencyMatrix.size() - 1;
	}

	size_t copyNodeWithoutLink(size_t nodeIndex)
	{
		m_nodes.push_back(m_nodes[nodeIndex]);

		for (auto& line : m_adjacencyMatrix)
		{
			line.push_back(False());
		}

		m_adjacencyMatrix.push_back(std::vector<char>(m_nodes.size(), False()));

		m_adjacencyMatrix.back().back() = False();

		return m_adjacencyMatrix.size() - 1;
	}

	void addLink(int indexA, int indexB)
	{
		m_adjacencyMatrix[indexB][indexA] = True();
		//m_adjacencyMatrix[indexA][indexB] = True();
	}

	void addBidirectionalLink(int indexA, int indexB)
	{
		m_adjacencyMatrix[indexB][indexA] = True();
		m_adjacencyMatrix[indexA][indexB] = True();
	}

	void removeLink(int indexA, int indexB)
	{
		m_adjacencyMatrix[indexB][indexA] = False();
		//m_adjacencyMatrix[indexA][indexB] = False();
	}

	void removeAllLinkByIndex(int index)
	{
		for (auto& line : m_adjacencyMatrix)
		{
			line[index] = False();
		}
		for (auto& x : m_adjacencyMatrix[index])
		{
			x = False();
		}
	}

	//poly1 - poly2
	std::vector<std::vector<ClipperLib::IntPoint>> setPolygons(const std::vector<ClipperLib::IntPoint>& poly1, const std::vector<ClipperLib::IntPoint>& poly2)
	{
		static int passCount = 0;

		if (passCount == 3)
		{
			const auto baseName = FileSystem::BaseName(FileSystem::UniquePath());
			std::ofstream ofs1(CharacterSet::Narrow(
				baseName + L"_poly1.txt"
			));
			std::ofstream ofs2(CharacterSet::Narrow(
				baseName + L"_poly2.txt"
			));

			for (const auto& p : poly1)
			{
				ofs1 << p.X << " " << p.Y << " " << p.Z << "\n";
			}
			for (const auto& p : poly2)
			{
				ofs2 << p.X << " " << p.Y << " " << p.Z << "\n";
			}

			return{};
		}

		++passCount;

		const Polygon subjectPoly = ToPoly(poly1);
		const Polygon clipPoly = ToPoly(poly2);

		std::vector<size_t> removeIndices;

		//ノードの初期化
		{
			for (auto i : step(poly1.size()))
			{
				//m_nodes.push_back({ poly1[i], clipPoly.contains(poly1[i]) ? False() : True() });
				if (clipPoly.contains(ToVec2(poly1[i])) || Includes(poly2, poly1[i]))
				{
					m_nodes.push_back({ poly1[i], False() });
					removeIndices.push_back(i);
				}
				else
				{
					m_nodes.push_back({ poly1[i], True() });
				}
				//loopPoints1.push_back(i);
			}

			for (auto i : step(poly2.size()))
			{
				m_nodes.push_back({ poly2[i], False() });
			}
		}

		//隣接行列の初期化
		{
			m_adjacencyMatrix = std::vector<std::vector<char>>(poly1.size() + poly2.size(), std::vector<char>(poly1.size() + poly2.size(), False()));

			size_t offset = 0;
			for (size_t i = 0; i < poly1.size(); ++i)
			{
				addLink(offset + i, offset + (i + 1) % poly1.size());
			}

			offset = poly1.size();
			for (size_t i = 0; i < poly2.size(); ++i)
			{
				//クリップポリゴンの周上では探索方向に制限を設けない（どちらが時計回りかわからないため）
				//その代わり、探索の過程では後戻りを禁止する
				addBidirectionalLink(offset + i, offset + (i + 1) % poly2.size());
			}
		}

		//debugPrint();

		/*
		中点挿入により解決する方法
		共有点がある場合にうまく動かない
		*/
		/*
		{
		std::vector<std::vector<std::pair<ClipperLib::IntPoint, size_t>>> intersectionListLines2(poly2.size());

		size_t currentInsertPos = 1;
		for (size_t p1 = 0; p1 < poly1.size(); ++p1)
		{
		const size_t l1Begin = p1;
		const size_t l1End = (p1 + 1) % poly1.size();

		const Line line1 = ToLine(poly1[l1Begin], poly1[l1End]);
		std::vector<std::pair<ClipperLib::IntPoint, size_t>> intersectionList;

		for (size_t p2 = 0; p2 < poly2.size(); ++p2)
		{
		auto& intersectionListLine2 = intersectionListLines2[p2];

		const size_t l2Begin = p2;
		const size_t l2End = (p2 + 1) % poly2.size();

		const Line line2 = ToLine(poly2[l2Begin], poly2[l2End]);

		if (auto crossPointOpt = line1.intersectsAt(line2))
		{
		//          l2.p0
		//            |
		//            v
		// l1.p0 -> cross -> l1.p1
		//            |
		//            v
		//          l2.p1

		ClipperLib::IntPoint intersectionPos = ToIntPoint(crossPointOpt.value(), 0);
		ZFillFunc(poly1[l1Begin], poly1[l1End], poly2[l2Begin], poly2[l2End], intersectionPos);

		//交点は有効な頂点とする
		const size_t crossPoint = addNode({ intersectionPos, True() });
		TEST_LOG(L"detect cross point at: ", crossPointOpt.value());

		intersectionList.emplace_back(intersectionPos, crossPoint);
		intersectionListLine2.emplace_back(intersectionPos, crossPoint);

		removeLink(l1Begin, l1End);

		removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
		removeLink(poly1.size() + l2End, poly1.size() + l2Begin);
		}
		else if (IsSamePos(poly1[l1Begin], poly2[l2Begin]) && IsSamePos(poly1[l1End], poly2[l2End])
		|| IsSamePos(poly1[l1Begin], poly2[l2End]) && IsSamePos(poly1[l1End], poly2[l2Begin]))
		{
		//完全に同一な線であれば、その上は行き来できないものとする

		removeLink(l1Begin, l1End);
		removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
		removeLink(poly1.size() + l2End, poly1.size() + l2Begin);
		}
		//1つの共有点を持つ場合は、リンクを追加する
		// l1.p0 -> l1.p1(l2.p0)
		//            |
		//            v
		//          l2.p1
		else if (IsSamePos(poly1[l1Begin], poly2[l2Begin]))
		{
		addBidirectionalLink(l1Begin, poly1.size() + l2Begin);
		}
		else if (IsSamePos(poly1[l1Begin], poly2[l2End]))
		{
		addBidirectionalLink(l1Begin, poly1.size() + l2End);
		}
		else if (IsSamePos(poly1[l1End], poly2[l2Begin]))
		{
		addBidirectionalLink(l1End, poly1.size() + l2Begin);
		}
		else if (IsSamePos(poly1[l1End], poly2[l2End]))
		{
		addBidirectionalLink(l1End, poly1.size() + l2End);
		}
		}

		if (!intersectionList.empty())
		{
		//line1に沿ってソート
		std::sort(intersectionList.begin(), intersectionList.end(),
		[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
		{
		return line1.begin.distanceFromSq(ToVec2(a.first)) < line1.begin.distanceFromSq(ToVec2(b.first));
		});

		//交点は遮蔽されていないものとみなす
		//この場合、隣接した交点の間が遮蔽されているにもかかわらず、パスがつながる可能性がある
		//したがって、交点の間にもう一つ点を挿入する（この点が遮蔽されるかどうかで繋がるかどうかが決まる）

		for (int i = 0; i + 1 < intersectionList.size(); i += 2)
		{
		const ClipperLib::IntPoint a = intersectionList[i].first;
		const ClipperLib::IntPoint b = intersectionList[i + 1].first;
		const ClipperLib::IntPoint m = MidPoint(a, b);

		size_t midPointIndex;

		if (clipPoly.contains(ToVec2(m)))
		{
		midPointIndex = addNode({ m, False() });
		removeIndices.push_back(midPointIndex);
		}
		else
		{
		midPointIndex = addNode({ m, True() });
		}

		intersectionList.insert(intersectionList.begin() + i + 1, std::pair<ClipperLib::IntPoint, size_t>(m, midPointIndex));
		}

		//リンクの再貼り付け
		addLink(l1Begin, intersectionList.front().second);
		for (int i = 0; i + 1 < intersectionList.size(); ++i)
		{
		addLink(intersectionList[i].second, intersectionList[i + 1].second);
		}
		addLink(intersectionList.back().second, l1End);

		//loop1の順路作り
		std::vector<size_t> intersectionIndices(intersectionList.size());
		for (auto i : step(intersectionList.size()))
		{
		intersectionIndices[i] = intersectionList[i].second;
		}
		loopPoints1.insert(loopPoints1.end(), intersectionIndices.begin(), intersectionIndices.end());
		}

		loopPoints1.push_back(l1End);
		}

		for (size_t p2 = 0; p2 < poly2.size(); ++p2)
		{
		auto& intersectionListLine2 = intersectionListLines2[p2];

		const size_t l2Begin = p2;
		const size_t l2End = (p2 + 1) % poly2.size();

		const Line line2 = ToLine(poly2[l2Begin], poly2[l2End]);

		if (!intersectionListLine2.empty())
		{
		//line2に沿ってソート
		std::sort(intersectionListLine2.begin(), intersectionListLine2.end(),
		[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
		{
		return line2.begin.distanceFromSq(ToVec2(a.first)) < line2.begin.distanceFromSq(ToVec2(b.first));
		});

		//リンクの再貼り付け
		addBidirectionalLink(poly1.size() + l2Begin, intersectionListLine2.front().second);
		for (int i = 0; i + 1 < intersectionListLine2.size(); ++i)
		{
		addBidirectionalLink(intersectionListLine2[i].second, intersectionListLine2[i + 1].second);
		}
		addBidirectionalLink(intersectionListLine2.back().second, poly1.size() + l2End);
		}
		}

		//不要な連結を削除
		for (size_t p2 = 0; p2 < poly2.size(); ++p2)
		{
		if (!subjectPoly.contains(ToVec2(poly2[p2])))
		{
		const size_t removeIndex = poly1.size() + p2;
		removeAllLinkByIndex(removeIndex);
		}
		}

		for (auto removeIndex : removeIndices)
		{
		removeAllLinkByIndex(removeIndex);
		}
		}
		*/

		/*
		エッジコピー＋連結削除により解決する方法
		分割線をまたぐリンク自体を削除するので、中点の挿入は必要ない

		追記：やっぱり中点の挿入は必要（差分後のポリゴンが poly1 の頂点を一つも含まないケースでは中点を挿入しないと全て False になってしまう）
		*/
		{
			std::vector<std::vector<std::pair<ClipperLib::IntPoint, size_t>>> intersectionListLines2(poly2.size());

			struct TransitionPoint
			{
				//共有点の場合はpoly1とpoly2のインデックスをそれぞれ持ってくる
				TransitionPoint(size_t index2, size_t index1) :
					poly2Index(index2),
					poly1Index_1(index1)
				{}

				//交点の場合はnodeのインデックスをpoly2のものとして持ってきて、それを挟む前後のpoly1のインデックスも持ってくる
				TransitionPoint(size_t index2, size_t index1_prev, size_t index1_post) :
					poly2Index(index2),
					poly1Index_1(index1_prev),
					poly1Index_2(index1_post)
				{}

				bool isSharedPoint()const
				{
					return !isCrossPoint();
				}

				bool isCrossPoint()const
				{
					return static_cast<bool>(poly1Index_2);
				}

				size_t poly2Index;
				size_t poly1Index_1;
				Optional<size_t> poly1Index_2;
			};

			//std::vector<std::pair<ClipperLib::IntPoint, size_t>> transitionPoints;
			//std::vector<TransitionPoint> transitionPointsInPoly2;

			enum TransitionLineType { Inner, Outer, Equal, Unknown };

			class TransitionLines
			{
			public:

				//TransitionPointは交点または共有点
				//つまり、各TransitionPointの間はpoly1の内側・外側・共有線上のどれかである
				//共有線上かどうかは追加する時に既に分かっており、内側か外側かはTransitionLineの中点がpoly1の内側かどうかで判定できる

				void addCrossPoint(size_t indexNode, size_t index1_prev, size_t index1_post)
				{
					m_transitionPoints.emplace_back(TransitionPoint(indexNode, index1_prev, index1_post), Unknown);
				}

				void addSharedPoint(size_t index2, size_t index1)
				{
					m_transitionPoints.emplace_back(TransitionPoint(index2, index1), Unknown);
				}

				void addSharedLine(size_t index2_A, size_t index1_A, size_t index2_B, size_t index1_B)
				{
					m_transitionPoints.emplace_back(TransitionPoint(index2_A, index1_A), Unknown);

					m_transitionPoints.emplace_back(TransitionPoint(index2_B, index1_B), Equal);
				}

				void solve(const std::vector<GraphNode>& nodes, const std::vector<size_t>& loopPoints2, const Polygon& poly1)
				{
					const size_t poly1Size = poly1.outer().size();

					//m_transitionPoints の各点を loopPoints2 に沿う形（つまり poly2 に沿って時計回り）にソート	
					std::sort(m_transitionPoints.begin(), m_transitionPoints.end(),
						[&](const TransitionPointLine& a, const TransitionPointLine& b)
					{
						const auto aIndex = a.first.poly2Index + (a.first.isCrossPoint() ? 0 : poly1Size);
						const auto bIndex = b.first.poly2Index + (b.first.isCrossPoint() ? 0 : poly1Size);
						return indexOfLoopPoints2(aIndex, loopPoints2).value() < indexOfLoopPoints2(bIndex, loopPoints2).value();
					});

					//std::unique(m_transitionPoints.begin(), m_transitionPoints.end(),
					//	[&](const TransitionPointLine& a, const TransitionPointLine& b)
					//{
					//	
					//	//a.first.poly1Index_1 == b.first.poly1Index_1 && a.first.poly2Index == b.first.poly2Index && a.first.poly1Index_2 == b.first.poly1Index_2;
					//	const auto aIndex = a.first.poly2Index + (a.first.isCrossPoint() ? 0 : poly1Size);
					//	const auto bIndex = b.first.poly2Index + (b.first.isCrossPoint() ? 0 : poly1Size);
					//	return indexOfLoopPoints2(aIndex, loopPoints2).value() < indexOfLoopPoints2(bIndex, loopPoints2).value();
					//});



					solveLoop(nodes, loopPoints2, poly1);
					solveInOut(nodes, loopPoints2, poly1);
				}

				const TransitionPoint& beginOfLine(int lineIndex)const
				{
					return m_transitionPoints[(lineIndex + size() - 1) % size()].first;
				}

				const TransitionPoint& endOfLine(int lineIndex)const
				{
					return m_transitionPoints[lineIndex].first;
				}

				int size()const
				{
					return static_cast<int>(m_transitionPoints.size());
				}

				//loopPoints2 は交点も含みうる（交点のインデックスはpoly1,poly2より後ろにある）ことからソートすることができないので線形探索を行う
				Optional<size_t> indexOfLoopPoints2(size_t value, const std::vector<size_t>& loopPoints2)const
				{
					for (auto i : step(loopPoints2.size()))
					{
						if (loopPoints2[i] == value)
						{
							return i;
						}
					}

					return none;
				}

				size_t findBeginOfLoopPoints2(int lineIndex, const std::vector<size_t>& loopPoints2, size_t poly1Size)const
				{
					//const size_t beginIndex = beginOfLine(lineIndex).poly2Index;
					const size_t beginIndex = getNodeIndex(beginOfLine(lineIndex), poly1Size);
					const size_t beginLoopIndex2 = indexOfLoopPoints2(beginIndex, loopPoints2).value();

					return beginLoopIndex2;
				}

				//必ず findBeginOfLoopPoints2 < findEndOfLoopPoints2 となる結果を返す
				//インデックスに変換するには loopPoints2.size() で剰余を取る
				size_t findEndOfLoopPoints2(int lineIndex, const std::vector<size_t>& loopPoints2, size_t poly1Size)const
				{
					//const size_t endIndex = endOfLine(lineIndex).poly2Index;
					const size_t endIndex = getNodeIndex(endOfLine(lineIndex), poly1Size);
					const size_t endLoopIndex2 = indexOfLoopPoints2(endIndex, loopPoints2).value() + loopPoints2.size();

					return endLoopIndex2;
				}

				TransitionLineType getType(int lineIndex)const
				{
					return m_transitionPoints[lineIndex].second;
				}

				size_t getNodeIndex(const TransitionPoint & a, size_t poly1Size)const
				{
					return a.poly2Index + (a.isCrossPoint() ? 0 : poly1Size);
				};


			private:

				/*
				連続したTransitionPointについては、その間に含まれる線が内側か外側かを別に判定する必要がある

				対応する：
				・連続した二つの同一なTransitionPoint

				対応しない：
				・共有点1 - 交点1 - 交点2 - 共有点1　のように間に別の点を挟む閉路
				・連続した三つ以上の同一なTransitionPoint

				solveInOutで代替可能
				*/
				void solveLoop(const std::vector<GraphNode>& nodes, const std::vector<size_t>& loopPoints2, const Polygon& poly1) {}
				/*void solveLoop(const std::vector<GraphNode>& nodes, const std::vector<size_t>& loopPoints2, const Polygon& poly1)
				{
				for (int i = 0; i < size(); ++i)
				{
				if (m_transitionPoints[i].second != Unknown)
				{
				continue;
				}

				const size_t beginIndex = beginOfLine(i).poly2Index;
				const size_t endIndex = endOfLine(i).poly2Index;

				const size_t beginLoopIndex2 = indexOfLoopPoints2(beginIndex, loopPoints2).value();
				const size_t endLoopIndex2 = indexOfLoopPoints2(endIndex, loopPoints2).value() + loopPoints2.size();
				const size_t numOfPoints = (endLoopIndex2 - beginLoopIndex2) % loopPoints2.size();

				if (nodes[loopPoints2[beginLoopIndex2]].m_pos == nodes[loopPoints2[endLoopIndex2]].m_pos)
				{
				const auto samplePoint = nodes[loopPoints2[(beginLoopIndex2 + 1) % loopPoints2.size()]].m_pos;
				m_transitionPoints[i].second = transitionLineType(samplePoint);
				}
				}
				}*/

				/*
				各TransitionPoint間の線がpoly1の内側か外側かを判定する
				*/
				void solveInOut(const std::vector<GraphNode>& nodes, const std::vector<size_t>& loopPoints2, const Polygon& poly1)
				{
					const size_t poly1Size = poly1.outer().size();

					for (int i = 0; i < size(); ++i)
					{
						if (m_transitionPoints[i].second != Unknown)
						{
							continue;
						}

						/*const size_t beginIndex = beginOfLine(i).poly2Index;
						const size_t endIndex = endOfLine(i).poly2Index;*/
						const size_t beginIndex = getNodeIndex(beginOfLine(i), poly1Size);
						const size_t endIndex = getNodeIndex(endOfLine(i), poly1Size);

						const size_t beginLoopIndex2 = indexOfLoopPoints2(beginIndex, loopPoints2).value();
						const size_t endLoopIndex2 = indexOfLoopPoints2(endIndex, loopPoints2).value() + loopPoints2.size();
						const size_t numOfPoints = (endLoopIndex2 - beginLoopIndex2) % loopPoints2.size();

						const auto transitionLineType = [&](const ClipperLib::IntPoint& pos)
						{
							return poly1.contains(ToVec2(pos)) ? Inner : Outer;
						};

						//beginLoopIndex2 と endLoopIndex2 で囲まれる線が poly1 の内側か外側かを判定する
						if (numOfPoints == 2)
						{
							//１つも点がない場合：つまり単なる線分である場合は、単純に中点を取って内側か外側か判定すればよい
							TEST_LOG(L"L(", __LINE__, L"): <A");
							const auto samplePoint = MidPoint(nodes[loopPoints2[beginLoopIndex2]].m_pos, nodes[loopPoints2[endLoopIndex2 % loopPoints2.size()]].m_pos);
							TEST_LOG(L"L(", __LINE__, L"): B>");
							m_transitionPoints[i].second = transitionLineType(samplePoint);
						}
						else
						{
							//１つ以上点がある場合：begin/end の間では内側/外側は変化しないので、どれが一つの点を持ってきて判定すればよい
							const auto samplePoint = nodes[loopPoints2[(beginLoopIndex2 + 1) % loopPoints2.size()]].m_pos;
							m_transitionPoints[i].second = transitionLineType(samplePoint);
						}
					}
				}

				using TransitionPointLine = std::pair<TransitionPoint, TransitionLineType>;

				std::vector<TransitionPointLine> m_transitionPoints;

				//std::vector<TransitionPoint> m_transitionPoints;

				//(-1,0), (0,1), (1,2), ..., (-2,-1)
				//std::vector<Type> m_transitionLines;
			};

			TransitionLines transitionLines;

			size_t currentInsertPos = 1;
			for (size_t p1 = 0; p1 < poly1.size(); ++p1)
			{
				const size_t l1Begin = p1;
				const size_t l1End = (p1 + 1) % poly1.size();

				const Line line1 = ToLine(poly1[l1Begin], poly1[l1End]);
				std::vector<std::pair<ClipperLib::IntPoint, size_t>> intersectionList;

				for (size_t p2 = 0; p2 < poly2.size(); ++p2)
				{
					auto& intersectionListLine2 = intersectionListLines2[p2];

					const size_t l2Begin = p2;
					const size_t l2End = (p2 + 1) % poly2.size();

					const Line line2 = ToLine(poly2[l2Begin], poly2[l2End]);

					if (IsSamePos(poly1[l1Begin], poly2[l2Begin]) && IsSamePos(poly1[l1End], poly2[l2End])
						|| IsSamePos(poly1[l1Begin], poly2[l2End]) && IsSamePos(poly1[l1End], poly2[l2Begin]))
					{
						//完全に同一な線であれば、その上は行き来できないものとする

						// l1.p0(l2.p0) -> l1.p1(l2.p1)

						removeLink(l1Begin, l1End);
						removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
						removeLink(poly1.size() + l2End, poly1.size() + l2Begin);

						if (IsSamePos(poly1[l1Begin], poly2[l2Begin]))
						{
							transitionLines.addSharedLine(l2Begin, l1Begin, l2End, l1End);
						}
						else
						{
							transitionLines.addSharedLine(l2Begin, l1End, l2End, l1Begin);
						}
					}
					//1つの共有点を持つ場合は、リンクを追加する
					// l1.p0 -> l1.p1(l2.p0)
					//            |
					//            v
					//          l2.p1
					else if (IsSamePos(poly1[l1Begin], poly2[l2Begin]))
					{
						TEST_LOG(L"L(", __LINE__, L"): Shared Point ", Point(l2Begin, l1Begin));
						transitionLines.addSharedPoint(l2Begin, l1Begin);
						addBidirectionalLink(l1Begin, poly1.size() + l2Begin);
					}
					else if (IsSamePos(poly1[l1Begin], poly2[l2End]))
					{
						//transitionLines.addSharedPoint(l2End, l1Begin);
						//addBidirectionalLink(l1Begin, poly1.size() + l2End);
						continue;
					}
					else if (IsSamePos(poly1[l1End], poly2[l2Begin]))
					{
						//transitionLines.addSharedPoint(l2Begin, l1End);
						//addBidirectionalLink(l1End, poly1.size() + l2Begin);
						continue;
					}
					else if (IsSamePos(poly1[l1End], poly2[l2End]))
					{
						//TEST_LOG(L"L(", __LINE__, L"): Shared Point ", Point(l1End, l2End));
						//transitionLines.addSharedPoint(l2End, l1End);
						//addBidirectionalLink(l1End, poly1.size() + l2End);
						continue;
					}
					else if (auto crossPointOpt = line1.intersectsAt(line2))
					{
						//          l2.p0
						//            |
						//            v
						// l1.p0 -> cross -> l1.p1
						//            |
						//            v
						//          l2.p1

						ClipperLib::IntPoint intersectionPos = ToIntPoint(crossPointOpt.value(), 0);
						ZFillFunc(poly1[l1Begin], poly1[l1End], poly2[l2Begin], poly2[l2End], intersectionPos);

						//交点は無効な頂点とする（左右にコピーを作るためここを通る必要はないので）
						const size_t crossPoint = addNode({ intersectionPos, False() });
						TEST_LOG(L"detect cross point at: ", crossPointOpt.value());

						transitionLines.addCrossPoint(crossPoint, l1Begin, l1End);

						//transitionPointsInPoly2.emplace_back(crossPoint);

						intersectionList.emplace_back(intersectionPos, crossPoint);
						intersectionListLine2.emplace_back(intersectionPos, crossPoint);

						removeLink(l1Begin, l1End);

						removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
						removeLink(poly1.size() + l2End, poly1.size() + l2Begin);
					}
				}

				if (!intersectionList.empty())
				{
					//line1に沿ってソート
					std::sort(intersectionList.begin(), intersectionList.end(),
						[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
					{
						return line1.begin.distanceFromSq(ToVec2(a.first)) < line1.begin.distanceFromSq(ToVec2(b.first));
					});

					//リンクの再貼り付け
					addLink(l1Begin, intersectionList.front().second);
					for (int i = 0; i + 1 < intersectionList.size(); ++i)
					{
						addLink(intersectionList[i].second, intersectionList[i + 1].second);
					}
					addLink(intersectionList.back().second, l1End);

					//loop1の順路作り
					std::vector<size_t> intersectionIndices(intersectionList.size());
					for (auto i : step(intersectionList.size()))
					{
						intersectionIndices[i] = intersectionList[i].second;
					}
					loopPoints1.insert(loopPoints1.end(), intersectionIndices.begin(), intersectionIndices.end());
				}

				loopPoints1.push_back(l1End);
			}

			for (size_t p2 = 0; p2 < poly2.size(); ++p2)
			{
				auto& intersectionListLine2 = intersectionListLines2[p2];

				const size_t l2Begin = p2;
				const size_t l2End = (p2 + 1) % poly2.size();

				const Line line2 = ToLine(poly2[l2Begin], poly2[l2End]);

				if (!intersectionListLine2.empty())
				{
					//line2に沿ってソート
					std::sort(intersectionListLine2.begin(), intersectionListLine2.end(),
						[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
					{
						return line2.begin.distanceFromSq(ToVec2(a.first)) < line2.begin.distanceFromSq(ToVec2(b.first));
					});

					//リンクの再貼り付け
					addBidirectionalLink(poly1.size() + l2Begin, intersectionListLine2.front().second);
					for (int i = 0; i + 1 < intersectionListLine2.size(); ++i)
					{
						addBidirectionalLink(intersectionListLine2[i].second, intersectionListLine2[i + 1].second);
					}
					addBidirectionalLink(intersectionListLine2.back().second, poly1.size() + l2End);

					//loop2の順路作り
					std::vector<size_t> intersectionIndices(intersectionListLine2.size());
					for (auto i : step(intersectionListLine2.size()))
					{
						intersectionIndices[i] = intersectionListLine2[i].second;
					}
					loopPoints2.insert(loopPoints2.end(), intersectionIndices.begin(), intersectionIndices.end());
				}

				loopPoints2.push_back(poly1.size() + l2End);
			}

			/*
			全ての交点・共有点間の折れ線について、line1にその折れ線のコピーを二つ作り、
			一つは外側のインデックスとリンクを張り、もう一つは内側のインデックスとリンクを張るようにする。
			そして、外側と内側の間についてのリンクを切り離し行き来できないようにする。
			*/
			{
				/*
				TODO : solveよりも前に、連続した共有線の間を取り除き、始点と終点のみ残す
				*/

				std::set<size_t> processedNodes;

				transitionLines.solve(m_nodes, loopPoints2, subjectPoly);

				for (size_t line = 0; line < transitionLines.size(); ++line)
				{
					//TEST_LOG(__LINE__, L": for (size_t line = 0; line < transitionLines.size(); ++line)");
					const auto beginOfLine = transitionLines.beginOfLine(line);
					const auto endOfLine = transitionLines.beginOfLine(line);

					const size_t beginIndex = transitionLines.findBeginOfLoopPoints2(line, loopPoints2, poly1.size());
					const size_t endIndex = transitionLines.findEndOfLoopPoints2(line, loopPoints2, poly1.size()) % loopPoints2.size();

					TEST_LOG(__LINE__, L": about transitionLines[", line, L"]: P(", beginIndex, L")=", ClipVertex(m_nodes[loopPoints2[beginIndex]].m_pos).m_pos, L", P(", endIndex, L")=", ClipVertex(m_nodes[loopPoints2[endIndex]].m_pos).m_pos);

					//beginとendが示すインデックスが同じ => TransitionLineが閉路である場合は、その間の線が内側の時のみリンクを削除する
					//if (m_nodes[loopPoints2[beginIndex]].m_pos == m_nodes[loopPoints2[endIndex]].m_pos)
					if (beginIndex == endIndex)
					{
						TEST_LOG(__LINE__, L": beginIndex == endIndex");
						if (transitionLines.getType(line) == TransitionLineType::Inner)
						{
							TEST_LOG(__LINE__, L": transitionLines.getType(line) == TransitionLineType::Inner");
							const size_t nodeIndexPoly1 = beginOfLine.poly1Index_1;
							const size_t nodeIndexPoly2 = poly1.size() + beginOfLine.poly2Index;

							const size_t nodesID = std::hash<size_t>()(nodeIndexPoly1) + std::hash<size_t>()(nodeIndexPoly2);

							TEST_LOG(__LINE__, L": About node ", nodeIndexPoly1, L" and node ", nodeIndexPoly2, L" :");

							if (processedNodes.count(nodesID) != 0)
							{
								TEST_LOG(__LINE__, L": Already processed skip ...");
								continue;
							}

							processedNodes.insert(nodesID);

							assert(nodeIndexPoly2 == loopPoints2[beginIndex]);

							const size_t cloneIndexPoly1 = copyNodeWithoutLink(nodeIndexPoly1);
							const size_t cloneIndexPoly2 = copyNodeWithoutLink(nodeIndexPoly2);

							const size_t nextNodeIndexPoly1 = (nodeIndexPoly1 + 1) % poly1.size();

							const size_t postFrontNodeIndexPoly2 = loopPoints2[beginIndex + 1];
							const size_t prevBackNodeIndexPoly2 = loopPoints2[(beginIndex + loopPoints2.size() - 1) % loopPoints2.size()];

							removeLink(prevBackNodeIndexPoly2, nodeIndexPoly2);
							removeLink(nodeIndexPoly2, prevBackNodeIndexPoly2);
							addBidirectionalLink(prevBackNodeIndexPoly2, cloneIndexPoly2);

							removeLink(nodeIndexPoly1, nextNodeIndexPoly1);
							removeLink(nextNodeIndexPoly1, nodeIndexPoly1);
							addBidirectionalLink(cloneIndexPoly1, nextNodeIndexPoly1);

							addBidirectionalLink(cloneIndexPoly1, cloneIndexPoly2);
						}
					}
					else
					{
						TEST_LOG(__LINE__, L": beginIndex != endIndex");
						if (transitionLines.getType(line) == TransitionLineType::Inner)
						{
							std::vector<size_t> copyNodeListPoly2;
							for (size_t subIndex = beginIndex; subIndex != endIndex; subIndex = (subIndex + 1) % loopPoints2.size())
							{
								copyNodeListPoly2.push_back(loopPoints2[subIndex]);
							}
							copyNodeListPoly2.push_back(loopPoints2[endIndex]);

							/*
							poly2 の部分線を取り出し、poly1 側に二つ複製を作る（線で区切られる内側用と外側用）
							*/
							std::array<size_t, 2> cloneLineBegins;
							std::array<size_t, 2> cloneLineEnds;
							for (size_t it = 0; it < 2; ++it)
							{
								std::vector<size_t> cloneIndices;
								for (size_t nodeIndex = 0; nodeIndex < copyNodeListPoly2.size(); ++nodeIndex)
								{
									cloneIndices.push_back(copyNodeWithoutLink(copyNodeListPoly2[nodeIndex]));
								}

								for (size_t cloneIndex = 0; cloneIndex + 1 < cloneIndices.size(); ++cloneIndex)
								{
									addBidirectionalLink(cloneIndices[cloneIndex], cloneIndices[cloneIndex + 1]);
								}

								cloneLineBegins[it] = cloneIndices.front();
								cloneLineEnds[it] = cloneIndices.back();
							}

							const size_t beginNodeIndexPoly1 = beginOfLine.poly1Index_1;
							const size_t beginNodeIndexPoly2 = poly1.size() + beginOfLine.poly2Index;

							const size_t endNodeIndexPoly1 = endOfLine.poly1Index_1;
							const size_t endNodeIndexPoly2 = poly1.size() + endOfLine.poly2Index;

							/*
							poly1 に複製を作るので、poly1 と poly2 のリンクは解除しておく
							*/
							removeLink(beginNodeIndexPoly1, beginNodeIndexPoly2);
							removeLink(beginNodeIndexPoly2, beginNodeIndexPoly1);
							removeLink(endNodeIndexPoly1, endNodeIndexPoly2);
							removeLink(endNodeIndexPoly2, endNodeIndexPoly1);

							const size_t prevBeginNodeIndexPoly1 = (beginNodeIndexPoly1 + poly1.size() - 1) % poly1.size();
							const size_t postBeginNodeIndexPoly1 = (beginNodeIndexPoly1 + 1) % poly1.size();
							const size_t prevEndNodeIndexPoly1 = (endNodeIndexPoly1 + poly1.size() - 1) % poly1.size();
							const size_t postEndNodeIndexPoly1 = (endNodeIndexPoly1 + 1) % poly1.size();

							/*
							まず poly1 の TransitionPoint の前後でリンクを切る
							*/
							removeLink(prevBeginNodeIndexPoly1, beginNodeIndexPoly1);
							removeLink(beginNodeIndexPoly1, postBeginNodeIndexPoly1);
							removeLink(prevEndNodeIndexPoly1, endNodeIndexPoly1);
							removeLink(endNodeIndexPoly1, postEndNodeIndexPoly1);

							/*
							左側を繋げる
							*/
							addBidirectionalLink(prevBeginNodeIndexPoly1, cloneLineBegins[0]);
							addBidirectionalLink(cloneLineEnds[0], postEndNodeIndexPoly1);

							/*
							右側を繋げる
							*/
							addBidirectionalLink(postBeginNodeIndexPoly1, cloneLineBegins[1]);
							addBidirectionalLink(cloneLineBegins[1], prevEndNodeIndexPoly1);
						}
					}
					/*else
					{
					for (size_t it = beginIndex; it <= endIndex; ++it)
					{
					const size_t index = it % loopPoints2.size();
					copyNodeWithoutLink(loopPoints2[index]);

					}
					}*/

				}
			}

			//不要な連結を削除
			/*for (size_t p2 = 0; p2 < poly2.size(); ++p2)
			{
			if (!subjectPoly.contains(ToVec2(poly2[p2])))
			{
			const size_t removeIndex = poly1.size() + p2;
			removeAllLinkByIndex(removeIndex);
			}
			}

			for (auto removeIndex : removeIndices)
			{
			removeAllLinkByIndex(removeIndex);
			}*/
		}

		m_adjacencyMatrix = transposed();

#ifdef OUTPUT_LOG
		debugPrint();
#endif

		//連続した有効要素を見つける
		{
			for (auto i : step(loopPoints1.size()))
			{
				const size_t currentIndex = loopPoints1[i];
				const size_t nextIndex = loopPoints1[(i + 1) % loopPoints1.size()];


				TEST_LOG(L"index(", currentIndex, L"): ", m_nodes[currentIndex].m_isEnabledVertex);

				if (m_nodes[currentIndex].m_isEnabledVertex == True() && m_nodes[nextIndex].m_isEnabledVertex == False())
				{
					endPoints.push_back(currentIndex);
					TEST_LOG(L"new end point index: ", currentIndex);
				}
			}
		}

		std::vector<std::vector<ClipperLib::IntPoint>> resultPolygons;
		std::vector<std::pair<size_t, size_t>> resultIndicesNumAndFrontIndex;
		{
			for (auto i : step(endPoints.size()))
			{
				auto routeIndices = findShortestLoop(endPoints[i]);

				if (routeIndices.empty())
				{
					LOG_ERROR(L"閉路の計算に失敗");
					continue;
				}

				if (!isClockWise(routeIndices))
				{
					std::reverse(routeIndices.begin(), routeIndices.end());
				}

				bool isNewLoop = true;
				//この閉路が過去に計算したものと同じか調べる
				for (const auto& previousComputed : resultIndicesNumAndFrontIndex)
				{
					//インデックスの個数が違えば異なる閉路であるはず
					if (previousComputed.first != routeIndices.size())
					{
						continue;
					}

					for (auto index : routeIndices)
					{
						//1つでも同じ共有点があれば同じ閉路だとみなす
						if (index == previousComputed.second)
						{
							isNewLoop = false;
						}
					}
				}

				if (isNewLoop)
				{
					resultPolygons.emplace_back();

					resultIndicesNumAndFrontIndex.emplace_back();
					auto& indicesInfo = resultIndicesNumAndFrontIndex.back();
					indicesInfo.first = routeIndices.size();
					indicesInfo.second = routeIndices.front();

					for (auto p : routeIndices)
					{
						resultPolygons.back().push_back(m_nodes[p].m_pos);
					}
				}
			}
		}

		{
			for (const auto& p : m_nodes)
			{
				pp.push_back(RandomVec2(1.0));
			}
		}

		return resultPolygons;
	}

	void debugDraw()
	{
		const auto ToDrawVec2 = [](const ClipperLib::IntPoint& p)
		{
			return ClipVertex(p).m_pos;
		};
		const auto ToDrawLine = [&](const ClipperLib::IntPoint& p0, const ClipperLib::IntPoint& p1)
		{
			return Line(ClipVertex(p0).m_pos, ClipVertex(p1).m_pos);
		};

		//const double radius = 10.0;
		const double radius = 6.0 / Graphics2D::GetMaxScaling();

		if (Input::KeyShift.pressed)
		{
			for (auto i : step(loopPoints1.size()))
			{
				const size_t currentIndex = loopPoints1[i];
				const size_t nextIndex = loopPoints1[(i + 1) % loopPoints1.size()];

				ToDrawLine(m_nodes[currentIndex].m_pos, m_nodes[nextIndex].m_pos).drawArrow(1.0, { 7.5,7.5 }, Palette::Yellow);
			}

			/*for (auto p : endPoints)
			{
			Circle(ToDrawVec2(m_nodes[p].m_pos), radius).draw(Palette::Purple);
			}*/
		}
		else
		{
			for (const auto y : step(m_adjacencyMatrix.size()))
			{
				const auto& line = m_adjacencyMatrix[y];
				for (const auto x : step(line.size()))
				{
					//y -> x
					if (line[x] == True())
					{
						const Vec2 dir = ToDrawLine(m_nodes[y].m_pos, m_nodes[x].m_pos).vector().normalized();
						Line(ToDrawVec2(m_nodes[y].m_pos) + dir*radius, ToDrawVec2(m_nodes[x].m_pos) - dir*radius).drawArrow(1.0 / Graphics2D::GetMaxScaling(), Vec2(3, 3) / Graphics2D::GetMaxScaling(), Palette::Skyblue);
						//Line(ToDrawVec2(m_nodes[y].m_pos), ToDrawVec2(m_nodes[x].m_pos)).drawArrow(1.0, { 5.0,5.0 }, Palette::Skyblue);
					}
				}
			}

			//for (const auto& node : m_nodes)
			for (size_t i = 0; i < m_nodes.size(); ++i)
			{
				const auto& node = m_nodes[i];
				Circle(ToDrawVec2(node.m_pos), radius).drawFrame(1.0 / Graphics2D::GetMaxScaling(), 0.0, node.m_isEnabledVertex == True() ? Palette::Orange : Palette::Gray);

				//const String nodeName = Format(L"node", i, ClipVertex(m_nodes[i].m_pos).m_pos);
				//m_debugFont(nodeName).drawAt(ToDrawVec2(m_nodes[i].m_pos));
			}
		}
	}

	void debugDraw2(const Mat3x2& mat)
	{
		const auto ToDrawVec2 = [&](const ClipperLib::IntPoint& p)
		{
			return mat.transform(ClipVertex(p).m_pos);
		};

		//const double d = 30.0 / Graphics2D::GetMaxScaling();
		const double d = 10.0;
		for (size_t i = 0; i < m_nodes.size(); ++i)
		{
			const auto& node = m_nodes[i];
			
			const String nodeName = Format(L"n", i, ClipVertex(m_nodes[i].m_pos).m_pos.asPoint());
			
			m_debugFont(nodeName).drawAt(ToDrawVec2(m_nodes[i].m_pos) + pp[i]*d);
		}
	}

	void dumpGraphViz(const String& filename)
	{
		std::ofstream ofs(filename.c_str());

		const auto nodeLabel = [](size_t i, const ClipperLib::IntPoint& p)
		{
			return CharacterSet::Narrow(Format(L"node", i, ClipVertex(p).m_pos));
			//return std::string("(") + std::to_string(p.X) + ", " + std::to_string(p.Y) + ", " + std::to_string(p.Z) + ")";
		};

		ofs << "digraph G {\n";
		//ノード書き出し
		{
			for (size_t i = 0; i < m_nodes.size(); ++i)
			{
				//ofs << "node" << i << " [label=\"" << nodeLabel(m_nodes[i].m_pos) << "\"];\n";
				ofs << "node" << i << " [label=\"" << nodeLabel(i, m_nodes[i].m_pos) << "\"];\n";
			}
		}
		//リンク書き出し
		{
			for (const auto y : step(m_adjacencyMatrix.size()))
			{
				const auto& line = m_adjacencyMatrix[y];
				for (const auto x : step(line.size()))
				{
					//y -> x
					if (line[x] == True())
					{
						ofs << "node" << y << " -> " << "node" << x << ";\n";
					}
				}
			}
		}
		ofs << "}\n";
	}

private:

	std::vector<std::vector<char>> transposed()
	{
		std::vector<std::vector<char>> result = std::vector<std::vector<char>>(m_adjacencyMatrix.size(), std::vector<char>(m_adjacencyMatrix.size()));
		for (int y = 0; y < m_adjacencyMatrix.size(); ++y)
		{
			auto& line = m_adjacencyMatrix[y];
			for (int x = 0; x < m_adjacencyMatrix.size(); ++x)
			{
				result[x][y] = line[x];
			}
		}

		return result;
	}

	void debugPrint()const
	{
		TEST_LOG(L"========================================================");

		std::vector<Vec2> nodes;

		for (const auto& node : m_nodes)
		{
			nodes.push_back(ToVec2(node.m_pos));
		}

		TEST_LOG(L"Nodes: ", nodes);
		TEST_LOG(L"--------------------");

		TEST_LOG(L"Adjacency Matrix: ");
		for (const auto& line : m_adjacencyMatrix)
		{
			std::vector<int> xs(line.size());
			for (const auto i : step(line.size()))
			{
				xs[i] = line[i];
			}

			TEST_LOG(xs);
		}

		TEST_LOG(L"========================================================");
	}

	bool isClockWise(const std::vector<size_t>& closedPath)const
	{
		double sum = 0;

		for (int i = 0; i < closedPath.size(); ++i)
		{
			const auto& p1 = m_nodes[closedPath[i]].m_pos;
			const auto& p2 = m_nodes[closedPath[(i + 1) % closedPath.size()]].m_pos;

			sum += (p2.X - p1.X)*(p2.Y + p1.Y);
		}

		return sum < 0.0;
	}

	std::vector<size_t> findShortestLoop(size_t index)
	{
		return dijkstraSearch(index, index);
	}

	std::vector<size_t> dijkstraSearch(size_t startNode, size_t endNode)
	{
		SPTree rootNode = std::make_shared<ShortestPathTree>();
		std::shared_ptr<ShortestNodeMap> shortestNodeMap = std::make_shared<ShortestNodeMap>();

		const auto costFunction = [&](size_t fromIndex, size_t toIndex) {return ToVec2(m_nodes[fromIndex].m_pos).distanceFromSq(ToVec2(m_nodes[toIndex].m_pos)); };

		rootNode->initRoot(startNode, shortestNodeMap);

		rootNode->calcCandidates(m_adjacencyMatrix[rootNode->index()], costFunction);

		while (true)
		{
			auto nextCandidate = rootNode->globalMinimumCostPath(0.0, rootNode);

			//追加するノードが存在しない -> 探索完了
			if (!nextCandidate.first)
			{
				TEST_LOG(L"Break: search completed");
				break;
			}

			auto newNode = nextCandidate.first->addChild(nextCandidate.second.first, nextCandidate.first);

			//目的ノードに到達
			if (shortestNodeMap->find(endNode) != shortestNodeMap->end())
			{
				TEST_LOG(L"Break: target path found");
				break;
			}

			newNode->calcCandidates(m_adjacencyMatrix[newNode->index()], costFunction);
		}

		if (shortestNodeMap->find(endNode) != shortestNodeMap->end())
		{
			auto it = shortestNodeMap->find(endNode);

			const auto toRoot = it->second->toRoot();
			TEST_LOG(L"Shortest path(reversed): ", toRoot);

			return toRoot;
		}

		TEST_LOG(L"Shortest path not found");
		return{};
	}

	std::vector<GraphNode> m_nodes;
	std::vector<std::vector<char>> m_adjacencyMatrix;

	std::vector<size_t> loopPoints1;
	std::vector<size_t> loopPoints2;
	std::vector<size_t> endPoints;

	std::vector<Vec2> pp;

	Font m_debugFont = Font(12);
};

inline ClippingRelation CalcRetation(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB)
{
	const Polygon poly = ToPoly(polygonA);
	const Polygon hole = ToPoly(polygonB);

	if (!poly.intersects(hole))
	{
		LOG(L"Relation is ClippingRelation::Distant");
		return ClippingRelation::Distant;
	}

	if (poly.contains(hole))
	{
		LOG(L"Relation is ClippingRelation::Contain");
		return ClippingRelation::Contain;
	}

	LOG(L"polygonB.size: ", polygonB.size());
	if (poly.contains(ToVec2(MidPoint(polygonB[0], polygonB[polygonB.size() / 2]))))
	{
		LOG(L"Relation is ClippingRelation::AdjacentInner");
		return ClippingRelation::AdjacentInner;
	}
	LOG(L"Relation is ClippingRelation::AdjacentOuter");
	return ClippingRelation::AdjacentOuter;
}

/*
newSegmentIndex: セグメントの追加が起きる場合があるので、新たなセグメントに振るインデックスに使う
*/
inline std::pair<ClipperLib::Paths, Optional<AdditionalInfo>> PolygonSubtract(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB, size_t& newSegmentIndex)
{
	const Polygon poly = ToPoly(polygonA);
	const Polygon hole = ToPoly(polygonB);

	if (poly.contains(hole))
	{
		LOG(L"PolygonSubtract: 穴の追加");

		double rayLength = Window::Height();

		size_t startIndex = 0;
		double minimumHeight = polygonB[0].Y;
		for (size_t i = 0; i < polygonB.size(); ++i)
		{
			if (polygonB[i].Y < minimumHeight)
			{
				minimumHeight = polygonB[i].Y;
				startIndex = i;
			}
		}

		using IntersectionType = std::pair<ClipperLib::IntPoint, size_t>;
		std::vector<IntersectionType> intersectionList;
		for (; intersectionList.empty(); rayLength *= 2.0)
		{
			const Line divLine(ToVec2(polygonB[startIndex]), ToVec2(polygonB[startIndex]) + Vec2(0, -1)*rayLength);
			for (size_t i = 0; i < polygonA.size(); ++i)
			{
				const Line polyLine = ToLine(polygonA[i], polygonA[(i + 1) % polygonA.size()]);
				if (auto intersectionOpt = divLine.intersectsAt(polyLine))
				{
					if (polygonA[i].Z == polygonA[(i + 1) % polygonA.size()].Z)
					{
						intersectionList.emplace_back(ToIntPoint(intersectionOpt.value(), polygonA[i].Z), i);
						//auto pt = ToIntPoint(intersectionOpt.value(), 0);
						//ZFillFunc(polygonA[i], polygonA[(i + 1) % polygonA.size()], polygonB[startIndex], polygonB[startIndex], pt);
					}
					else
					{
						LOG_ERROR(L"iとi+1の間で曲線が切り替わる");
						return{};
					}
				}
			}
		}

		const Vec2 basePos = ToVec2(polygonB[startIndex]);

		std::sort(intersectionList.begin(), intersectionList.end(),
			[&](const IntersectionType& a, const IntersectionType& b) {
			return basePos.distanceFromSq(ToVec2(a.first)) < basePos.distanceFromSq(ToVec2(b.first));
		}
		);

		ClipperLib::Path result;

		std::pair<ClipperLib::IntPoint, size_t> edgeA = intersectionList.front();
		ClipperLib::IntPoint edgeB = polygonB[startIndex];

		/*
		CurveSegmentsには存在しない線分を追加するので、それも返す必要がある
		*/
		AdditionalInfo segments;
		{
			edgeA.first.Z = MergeIndex(edgeA.first.Z, newSegmentIndex);
			edgeB.Z = MergeIndex(edgeB.Z, newSegmentIndex);

			segments.additionalSegments.emplace_back();
			auto& newSegment = segments.additionalSegments.back();
			newSegment.setLine(ClipVertex(edgeA.first).m_pos, ClipVertex(edgeB).m_pos);

			++newSegmentIndex;
		}

		result.push_back(edgeA.first);
		for (size_t i = 1; i < polygonA.size(); ++i)
		{
			result.push_back(polygonA[(edgeA.second + i) % polygonA.size()]);
		}
		result.push_back(edgeA.first);

		/*result.push_back(polygonB[startIndex]);
		for (size_t i = 1; i < polygonB.size(); ++i)
		{
		result.push_back(polygonB[(startIndex + i) % polygonB.size()]);
		}
		result.push_back(polygonB[startIndex]);*/
		result.push_back(edgeB);
		for (size_t i = 1; i < polygonB.size(); ++i)
		{
			result.push_back(polygonB[(startIndex + i) % polygonB.size()]);
		}
		result.push_back(edgeB);

		return std::pair<ClipperLib::Paths, Optional<AdditionalInfo>>{ {result}, segments };
	}
	else
	{
		LOG(L"PolygonSubtract: グラフ探索");
		VerticesGraph graph;
		return std::pair<ClipperLib::Paths, Optional<AdditionalInfo>>{ graph.setPolygons(polygonA, polygonB), none };
	}
}

inline ClipperLib::Paths PolygonSubtract2(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB)
{
	VerticesGraph graph;
	return graph.setPolygons(polygonA, polygonB);
}

inline bool IsAppliableGraph(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB)
{
	const Polygon poly = ToPoly(polygonA);
	const Polygon hole = ToPoly(polygonB);

	return !poly.contains(hole);
}
