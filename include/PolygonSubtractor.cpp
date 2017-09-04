#include <Siv3D.hpp>

#include "PolygonSubtractor.hpp"

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
	LOG_ERROR(__LINE__);

}

class ShortestPathTree;
using SPTree = std::shared_ptr<ShortestPathTree>;

//�@ShortestPathTree�̒��ŁA�ŏ��p�X��\���m�[�h�̃}�b�v
//�@ShortestPathTree�ɓ����C���f�b�N�X����x����Ȃ��悤�ɂ��邽�߂Ɏg��
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
		//�C���e���Z���X�̐Ԑ����
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
				//��߂�֎~
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

		//�����ւ̃����N�͏�ɖ����Ƃ���i�_�C�N�X�g���@�Ń��[�v��T�����߁j
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
		const Polygon subjectPoly = ToPoly(poly1);
		const Polygon clipPoly = ToPoly(poly2);

		std::vector<size_t> removeIndices;

		//�m�[�h�̏�����
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

		//�אڍs��̏�����
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
				//�N���b�v�|���S���̎���ł͒T�������ɐ�����݂��Ȃ��i�ǂ��炪���v��肩�킩��Ȃ����߁j
				//���̑���A�T���̉ߒ��ł͌�߂���֎~����
				addBidirectionalLink(offset + i, offset + (i + 1) % poly2.size());
			}
		}

		//debugPrint();

		/*
		���_�}���ɂ�����������@
		���L�_������ꍇ�ɂ��܂������Ȃ�
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

						//��_�͗L���Ȓ��_�Ƃ���
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
						//���S�ɓ���Ȑ��ł���΁A���̏�͍s�����ł��Ȃ����̂Ƃ���

						removeLink(l1Begin, l1End);
						removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
						removeLink(poly1.size() + l2End, poly1.size() + l2Begin);
					}
					//1�̋��L�_�����ꍇ�́A�����N��ǉ�����
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
					//line1�ɉ����ă\�[�g
					std::sort(intersectionList.begin(), intersectionList.end(),
						[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
					{
						return line1.begin.distanceFromSq(ToVec2(a.first)) < line1.begin.distanceFromSq(ToVec2(b.first));
					});

					//��_�͎Օ�����Ă��Ȃ����̂Ƃ݂Ȃ�
					//���̏ꍇ�A�אڂ�����_�̊Ԃ��Օ�����Ă���ɂ�������炸�A�p�X���Ȃ���\��������
					//���������āA��_�̊Ԃɂ�����_��}������i���̓_���Օ�����邩�ǂ����Ōq���邩�ǂ��������܂�j

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

					//�����N�̍ē\��t��
					addLink(l1Begin, intersectionList.front().second);
					for (int i = 0; i + 1 < intersectionList.size(); ++i)
					{
						addLink(intersectionList[i].second, intersectionList[i + 1].second);
					}
					addLink(intersectionList.back().second, l1End);

					//loop1�̏��H���
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
					//line2�ɉ����ă\�[�g
					std::sort(intersectionListLine2.begin(), intersectionListLine2.end(),
						[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
					{
						return line2.begin.distanceFromSq(ToVec2(a.first)) < line2.begin.distanceFromSq(ToVec2(b.first));
					});

					//�����N�̍ē\��t��
					addBidirectionalLink(poly1.size() + l2Begin, intersectionListLine2.front().second);
					for (int i = 0; i + 1 < intersectionListLine2.size(); ++i)
					{
						addBidirectionalLink(intersectionListLine2[i].second, intersectionListLine2[i + 1].second);
					}
					addBidirectionalLink(intersectionListLine2.back().second, poly1.size() + l2End);
				}
			}

			//�s�v�ȘA�����폜
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
		�G�b�W�R�s�[�{�A���폜�ɂ�����������@
		���������܂��������N���̂����݂��Ȃ��Ȃ�̂ŁA���_�̑}���͕K�v�Ȃ�
		*/
		{
			std::vector<std::vector<std::pair<ClipperLib::IntPoint, size_t>>> intersectionListLines2(poly2.size());

			struct TransitionPoint
			{
				//���L�_�̏ꍇ��poly1��poly2�̃C���f�b�N�X�����ꂼ�ꎝ���Ă���
				TransitionPoint(size_t index2, size_t index1):
					poly2Index(index2),
					poly1Index_1(index1)
				{}

				//��_�̏ꍇ��node�̃C���f�b�N�X��poly2�̂��̂Ƃ��Ď����Ă��āA��������ޑO���poly1�̃C���f�b�N�X�������Ă���
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
			std::vector<TransitionPoint> transitionPointsInPoly2;
			
			class TransitionLines
			{
			public:

				//TransitionPoint�͌�_�܂��͋��L�_
				//�܂�A�eTransitionPoint�̊Ԃ�poly1�̓����E�O���E���L����̂ǂꂩ�ł���
				//���L���ォ�ǂ����͒ǉ����鎞�Ɋ��ɕ������Ă���A�������O������TransitionLine�̒��_��poly1�̓������ǂ����Ŕ���ł���

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
					//m_transitionPoints �̊e�_�� loopPoints2 �ɉ����`�i�܂� poly2 �ɉ����Ď��v���j�Ƀ\�[�g
					std::sort(m_transitionPoints.begin(), m_transitionPoints.end(), 
						[&](const TransitionPointLine& a, const TransitionPointLine& b) 
					{
						return indexOfLoopPoints2(a.first.poly2Index, loopPoints2).value() < indexOfLoopPoints2(b.first.poly2Index, loopPoints2).value();
					});


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

						const auto transitionLineType = [&](const ClipperLib::IntPoint& pos)
						{
							return poly1.contains(ToVec2(pos)) ? Inner : Outer;
						};

						//beginLoopIndex2 �� endLoopIndex2 �ň͂܂����� poly1 �̓������O�����𔻒肷��
						if (numOfPoints == 2)
						{
							//�P���_���Ȃ��ꍇ�F�܂�P�Ȃ�����ł���ꍇ�́A�P���ɒ��_������ē������O�������肷��΂悢
							const auto samplePoint = MidPoint(nodes[loopPoints2[beginLoopIndex2]].m_pos, nodes[loopPoints2[endLoopIndex2]].m_pos);
							m_transitionPoints[i].second = transitionLineType(samplePoint);
						}
						else
						{
							//�P�ȏ�_������ꍇ�Fbegin/end �̊Ԃł͓���/�O���͕ω����Ȃ��̂ŁA�ǂꂪ��̓_�������Ă��Ĕ��肷��΂悢
							const auto samplePoint = nodes[loopPoints2[(beginLoopIndex2 + 1) % loopPoints2.size()]].m_pos;
							m_transitionPoints[i].second = transitionLineType(samplePoint);
						}
					}
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

				//LinearSearch
				//loopPoints2 �͌�_���܂݂���i�����Č�_�̃C���f�b�N�X�͌��̕��ɂ���j���ߏ����ɕ��ׂ邱�Ƃ��ł��Ȃ�
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

				size_t findBeginOfLoopPoints2(int lineIndex, const std::vector<size_t>& loopPoints2)const
				{
					const size_t beginIndex = beginOfLine(lineIndex).poly2Index;
					const size_t beginLoopIndex2 = indexOfLoopPoints2(beginIndex, loopPoints2).value();

					return beginLoopIndex2;
				}

				//�K�� findBeginOfLoopPoints2 < findEndOfLoopPoints2 �ƂȂ錋�ʂ�Ԃ�
				//���������ăC���f�b�N�X�Ƃ��ė��p����ɂ� loopPoints2.size() �ŏ�]�����
				size_t findEndOfLoopPoints2(int lineIndex, const std::vector<size_t>& loopPoints2)const
				{
					const size_t endIndex = endOfLine(lineIndex).poly2Index;
					const size_t endLoopIndex2 = indexOfLoopPoints2(endIndex, loopPoints2).value() + loopPoints2.size();

					return endLoopIndex2;
				}

			private:

				enum TransitionLineType { Inner, Outer, Equal, Unknown };

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

						//��_�͖����Ȓ��_�Ƃ���i���E�ɃR�s�[����邽�߂�����ʂ�K�v�͂Ȃ��̂Łj
						const size_t crossPoint = addNode({ intersectionPos, False() });
						TEST_LOG(L"detect cross point at: ", crossPointOpt.value());

						transitionLines.addCrossPoint(crossPoint, l1Begin, l1End);

						transitionPointsInPoly2.emplace_back(crossPoint);

						intersectionList.emplace_back(intersectionPos, crossPoint);
						intersectionListLine2.emplace_back(intersectionPos, crossPoint);

						removeLink(l1Begin, l1End);

						removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
						removeLink(poly1.size() + l2End, poly1.size() + l2Begin);
					}
					else if (IsSamePos(poly1[l1Begin], poly2[l2Begin]) && IsSamePos(poly1[l1End], poly2[l2End])
						|| IsSamePos(poly1[l1Begin], poly2[l2End]) && IsSamePos(poly1[l1End], poly2[l2Begin]))
					{
						//���S�ɓ���Ȑ��ł���΁A���̏�͍s�����ł��Ȃ����̂Ƃ���

						// l1.p0(l2.p0) -> l1.p1(l2.p1)

						removeLink(l1Begin, l1End);
						removeLink(poly1.size() + l2Begin, poly1.size() + l2End);
						removeLink(poly1.size() + l2End, poly1.size() + l2Begin);

						if (IsSamePos(poly1[l1Begin], poly2[l2Begin]))
						{
							transitionPointsInPoly2.emplace_back(l2Begin, l1Begin);
							transitionPointsInPoly2.emplace_back(l2End, l1End);

							transitionLines.addSharedLine(l2Begin, l1Begin, l2End, l1End);
						}
						else
						{
							transitionPointsInPoly2.emplace_back(l2Begin, l1End);
							transitionPointsInPoly2.emplace_back(l2End, l1Begin);

							transitionLines.addSharedLine(l2Begin, l1End, l2End, l1Begin);
						}
					}
					//1�̋��L�_�����ꍇ�́A�����N��ǉ�����
					// l1.p0 -> l1.p1(l2.p0)
					//            |
					//            v
					//          l2.p1
					else if (IsSamePos(poly1[l1Begin], poly2[l2Begin]))
					{
						transitionLines.addSharedPoint(l2Begin, l1Begin);
						addBidirectionalLink(l1Begin, poly1.size() + l2Begin);
						transitionPointsInPoly2.emplace_back(l2Begin, l1Begin);
					}
					else if (IsSamePos(poly1[l1Begin], poly2[l2End]))
					{
						transitionLines.addSharedPoint(l2End, l1Begin);
						addBidirectionalLink(l1Begin, poly1.size() + l2End);
						transitionPointsInPoly2.emplace_back(l2End, l1Begin);
					}
					else if (IsSamePos(poly1[l1End], poly2[l2Begin]))
					{
						transitionLines.addSharedPoint(l2Begin, l1End);
						addBidirectionalLink(l1End, poly1.size() + l2Begin);
						transitionPointsInPoly2.emplace_back(l2Begin, l1End);
					}
					else if (IsSamePos(poly1[l1End], poly2[l2End]))
					{
						transitionLines.addSharedPoint(l2End, l1End);
						addBidirectionalLink(l1End, poly1.size() + l2End);
						transitionPointsInPoly2.emplace_back(l2End, l1End);
					}
				}

				if (!intersectionList.empty())
				{
					//line1�ɉ����ă\�[�g
					std::sort(intersectionList.begin(), intersectionList.end(),
						[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
					{
						return line1.begin.distanceFromSq(ToVec2(a.first)) < line1.begin.distanceFromSq(ToVec2(b.first));
					});

					//�����N�̍ē\��t��
					addLink(l1Begin, intersectionList.front().second);
					for (int i = 0; i + 1 < intersectionList.size(); ++i)
					{
						addLink(intersectionList[i].second, intersectionList[i + 1].second);
					}
					addLink(intersectionList.back().second, l1End);

					//loop1�̏��H���
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
					//line2�ɉ����ă\�[�g
					std::sort(intersectionListLine2.begin(), intersectionListLine2.end(),
						[&](const std::pair<ClipperLib::IntPoint, size_t>& a, const std::pair<ClipperLib::IntPoint, size_t>& b)
					{
						return line2.begin.distanceFromSq(ToVec2(a.first)) < line2.begin.distanceFromSq(ToVec2(b.first));
					});

					//�����N�̍ē\��t��
					addBidirectionalLink(poly1.size() + l2Begin, intersectionListLine2.front().second);
					for (int i = 0; i + 1 < intersectionListLine2.size(); ++i)
					{
						addBidirectionalLink(intersectionListLine2[i].second, intersectionListLine2[i + 1].second);
					}
					addBidirectionalLink(intersectionListLine2.back().second, poly1.size() + l2End);

					//loop2�̏��H���
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
			�S�Ă̌�_�E���L�_�Ԃ̐܂���ɂ��āAline1�ɂ��̐܂���̃R�s�[�����A
			��͊O���̃C���f�b�N�X�ƃ����N�𒣂�A������͓����̃C���f�b�N�X�ƃ����N�𒣂�悤�ɂ���B
			�����āA�O���Ɠ����̊Ԃɂ��Ẵ����N��؂藣���s�����ł��Ȃ��悤�ɂ���B
			*/
			{
				transitionLines.solve(m_nodes, loopPoints2, subjectPoly);

				for (size_t line = 0; line < transitionLines.size(); ++line)
				{
					const size_t beginIndex = transitionLines.findBeginOfLoopPoints2(line, loopPoints2);
					const size_t endIndex = transitionLines.findEndOfLoopPoints2(line, loopPoints2);

					for (size_t it = beginIndex; it <= endIndex; ++it)
					{
						const size_t index = it % loopPoints2.size();
						copyNodeWithoutLink(loopPoints2[index]);
						��������
					}
				}
			}

			//�s�v�ȘA�����폜
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

		m_adjacencyMatrix = transposed();

#ifdef OUTPUT_LOG
		debugPrint();
#endif

		//�A�������L���v�f��������
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
					LOG_ERROR(L"�H�̌v�Z�Ɏ��s");
					continue;
				}

				if (!isClockWise(routeIndices))
				{
					std::reverse(routeIndices.begin(), routeIndices.end());
				}

				bool isNewLoop = true;
				//���̕H���ߋ��Ɍv�Z�������̂Ɠ��������ׂ�
				for (const auto& previousComputed : resultIndicesNumAndFrontIndex)
				{
					//�C���f�b�N�X�̌����Ⴆ�ΈقȂ�H�ł���͂�
					if (previousComputed.first != routeIndices.size())
					{
						continue;
					}

					for (auto index : routeIndices)
					{
						//1�ł��������L�_������Γ����H���Ƃ݂Ȃ�
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

		return resultPolygons;
	}

	void debugDraw()
	{
		const double radius = 10.0;

		if (Input::KeyShift.pressed)
		{
			for (const auto y : step(m_adjacencyMatrix.size()))
			{
				const auto& line = m_adjacencyMatrix[y];
				for (const auto x : step(line.size()))
				{
					//y -> x
					if (line[x] == True())
					{
						const Vec2 dir = ToLine(m_nodes[y].m_pos, m_nodes[x].m_pos).vector().normalized();
						Line(ToVec2(m_nodes[y].m_pos) + dir*radius, ToVec2(m_nodes[x].m_pos) - dir*radius).drawArrow(1.0, { 5.0,5.0 }, Palette::Skyblue);
					}
				}
			}

			for (const auto& node : m_nodes)
			{
				Circle(ToVec2(node.m_pos), radius).drawFrame(1.0, 0.0, node.m_isEnabledVertex == True() ? Palette::Orange : Palette::Gray);
			}
		}
		else
		{
			for (auto i : step(loopPoints1.size()))
			{
				const size_t currentIndex = loopPoints1[i];
				const size_t nextIndex = loopPoints1[(i + 1) % loopPoints1.size()];

				ToLine(m_nodes[currentIndex].m_pos, m_nodes[nextIndex].m_pos).drawArrow(1.0, { 7.5,7.5 }, Palette::Yellow);
			}
		}

		for (auto p : endPoints)
		{
			Circle(ToVec2(m_nodes[p].m_pos), radius).draw(Palette::Purple);
		}
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

			//�ǉ�����m�[�h�����݂��Ȃ� -> �T������
			if (!nextCandidate.first)
			{
				TEST_LOG(L"Break: search completed");
				break;
			}

			auto newNode = nextCandidate.first->addChild(nextCandidate.second.first, nextCandidate.first);

			//�ړI�m�[�h�ɓ��B
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
};

ClipperLib::Paths PolygonSubtract(const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB)
{
	const Polygon poly = ToPoly(polygonA);
	const Polygon hole = ToPoly(polygonB);

	if (poly.contains(hole))
	{
		LOG(L"PolygonSubtract: ���̒ǉ�");

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
					}
					else
					{
						LOG_ERROR(L"i��i+1�̊ԂŋȐ����؂�ւ��");
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

		std::pair<ClipperLib::IntPoint, size_t> nearest = intersectionList.front();

		result.push_back(nearest.first);
		for (size_t i = 1; i < polygonA.size(); ++i)
		{
			result.push_back(polygonA[(nearest.second + i) % polygonA.size()]);
		}
		result.push_back(nearest.first);

		result.push_back(polygonB[startIndex]);
		for (size_t i = 1; i < polygonB.size(); ++i)
		{
			result.push_back(polygonB[(startIndex + i) % polygonB.size()]);
		}
		result.push_back(polygonB[startIndex]);

		return{ result };
	}
	else
	{
		LOG(L"PolygonSubtract: �O���t�T��");
		VerticesGraph graph;
		return graph.setPolygons(polygonA, polygonB);
	}

	
}
