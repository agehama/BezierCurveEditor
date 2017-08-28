#pragma once
#include <queue>
#include <Siv3D.hpp> // August 2016 v2
#include "include/AnchorPoint.hpp"
#include "include/BoundingRect.hpp"
#include "include/BezierCurve.hpp"
#include "include/CurveSegment.hpp"

#include "include/clipper/clipper.hpp"

const double scaleInt = 100000.0;
//const double scaleInt = 1.0;

std::vector<ClipperLib::IntPoint> intersectionPoints;
std::vector<Line> linesForDebug;

double getVerticalDistance(const Vec2& origin, const Vec2& a, const Vec2& b)
{
	const Vec2 project = (a - origin) * (a - origin).dot(b - origin) / (a - origin).lengthSq();
	return ((origin - b) + project).lengthSq();
}

//void ZFillFunc(ClipperLib::IntPoint& e1bot, ClipperLib::IntPoint& e1top, ClipperLib::IntPoint& e2bot, ClipperLib::IntPoint& e2top, ClipperLib::IntPoint& pt)
//{
//	const Vec2 p(pt.X, pt.Y);
//	const Vec2& p1 = Line(e1bot.X, e1bot.Y, e1top.X, e1top.Y).closest(p);
//	const Vec2& p2 = Line(e2bot.X, e2bot.Y, e2top.X, e2top.Y).closest(p);
//
//	if (p1.distanceFromSq(p) < p2.distanceFromSq(p))
//	{
//		pt.Z = e1bot.Z;
//	}
//	else
//	{
//		pt.Z = e2bot.Z;
//	}
//}
//void ZFillFunc(ClipperLib::IntPoint& e1bot, ClipperLib::IntPoint& e1top, ClipperLib::IntPoint& e2bot, ClipperLib::IntPoint& e2top, ClipperLib::IntPoint& pt)
//{
//	const Vec2 pB(pt.X, pt.Y);
//	
//	const Vec2 pO1(e1bot.X, e1bot.Y);
//	const Vec2 pA1(e1top.X, e1top.Y);
//
//	const Vec2 pO2(e2bot.X, e2bot.Y);
//	const Vec2 pA2(e2top.X, e2top.Y);
//
//	if (getVerticalDistance(pO1, pA1, pB) < getVerticalDistance(pO2, pA2, pB))
//	{
//		pt.Z = e1bot.Z;
//	}
//	else
//	{
//		pt.Z = e2bot.Z;
//	}
//}
void ZFillFunc(ClipperLib::IntPoint& e1bot, ClipperLib::IntPoint& e1top, ClipperLib::IntPoint& e2bot, ClipperLib::IntPoint& e2top, ClipperLib::IntPoint& pt)
{
	intersectionPoints.push_back(pt);
	pt.Z = 0;
	pt.Z += e1bot.Z;
	pt.Z += (e2bot.Z << 32);
}

struct ClipVertex
{
	ClipVertex() = default;
	ClipVertex(int x, int y, ClipperLib::cInt z) :
		m_pos(1.0*x / scaleInt, 1.0*y / scaleInt),
		m_Z(z)
	{}

	ClipVertex(const ClipperLib::IntPoint& p) :
		m_pos(1.0*p.X / scaleInt, 1.0*p.Y / scaleInt),
		m_Z(p.Z)
	{}

	bool samePos(const ClipVertex& other)const
	{
		return m_pos == other.m_pos;
	}

	Vec2 m_pos;
	ClipperLib::cInt m_Z;
};

using Vertices = std::vector<ClipVertex>;

std::vector<Vertices> PathToPolygons(const ClipperLib::Path& path)
{
	std::vector<ClipVertex> ps;
	std::vector<Vertices> polygons;

	for (int current = 0; current < path.size(); ++current)
	{
		const ClipVertex currentPos(path[current].X, path[current].Y, path[current].Z);
		ps.push_back(currentPos);

		for (int previous = static_cast<int>(ps.size()) - 2; 0 <= previous; --previous)
		{
			if (ps[previous].samePos(currentPos))
			{
				polygons.emplace_back(std::vector<ClipVertex>(ps.cbegin() + previous + 1, ps.cend()));
				ps.erase(ps.begin() + previous + 1, ps.end());
			}
		}
	}

	polygons.emplace_back(std::vector<ClipVertex>(ps.begin(), ps.end()));

	return polygons;
}

/*
パスに射影した線分について、そのパス上での長さを返す

折れ線への線分の射影
　線分の始点と終点についてそれぞれ、
1. 折れ線の全ての線へ下ろした垂線の足の位置を計算する
2. その位置が折れ線の中にあるような垂線について、最小の長さを持つ垂線の足の位置が射影先となる
*/
double projectionLengthSq(const std::vector<Vec2>& openPath, const Line& line)
{
	const Vec2 pA = line.begin;
	const Vec2 pB = line.end;

	double minLengthA = DBL_MAX;
	double minLengthB = DBL_MAX;

	//正確に計算するならインデックスを取ってその間の長さを加算しなければならないが、
	//パスがそこまで複雑な形状になることは考えにくいため、始点と終点のみ取ってその長さを返せばいいと思う
	Vec2 minLengthPosA, minLengthPosB;
	//size_t minLengthIndexA, minLengthIndexB;

	for (size_t i = 0; i + 1 < openPath.size(); ++i)
	{
		const Line pathLine(openPath[i], openPath[i + 1]);

		const auto getProject = [&pathLine](const Vec2& pos)->Optional<std::pair<double, Vec2>>
		{
			const Vec2 vA = pos - pathLine.begin;
			const Vec2 vP = pathLine.vector();

			//lineの始点から垂線の足への相対ベクトル
			const Vec2 vH = vP * vA.dot(vP) / vP.dot(vP);

			if (vH.dot(vP) < 0 ||//垂線の足がpathLine.beginより手前にある
				vP.dot(vP) < vH.dot(vH)//垂線の足がpathLine.endより奥にある
				)
			{
				//return none;
			}
			
			return std::pair<double, Vec2>((vA - vH).lengthSq(), pathLine.begin + vH);
		};

		if (auto projectAOpt = getProject(pA))
		{
			const auto projectA = projectAOpt.value();
			if (projectA.first < minLengthA)
			{
				minLengthA = projectA.first;
				minLengthPosA = projectA.second;
				//minLengthIndexA = i;
			}
		}

		if (auto projectBOpt = getProject(pB))
		{
			const auto projectB = projectBOpt.value();
			if (projectB.first < minLengthB)
			{
				minLengthB = projectB.first;
				minLengthPosB = projectB.second;
				//minLengthIndexB = i;
			}
		}
	}

	return minLengthPosA.distanceFromSq(minLengthPosB);
}

//bool IsClockWise(const Vertices& polygonInt)
//{
//	double sum = 0;
//
//	for (int i = 0; i < polygonInt.size(); ++i)
//	{
//		const auto& p1 = polygonInt[i];
//		const auto& p2 = polygonInt[(i + 1) % polygonInt.size()];
//
//		sum += (p2.x - p1.x)*(p2.y + p1.y);
//	}
//
//	return sum < 0.0;
//}

struct SegmentInfo
{
	size_t pathIndex;
	size_t segmentIndex;
	double startT;
	double endT;
};

struct CurveSegmentInfo
{
	BezierCurve m_curve;
	double startT;
	double endT;
};

class BezierCurvePath
{
public:

	BezierCurvePath() = default;

	BezierCurvePath(const CSVReader& reader, size_t row)
	{
		for (size_t i = 0; i + 2 < reader.columns(row); i += 3)
		{
			m_anchorPoints.emplace_back(reader.get<Vec2>(row, i), reader.get<Vec2>(row, i + 1), reader.get<Vec2>(row, i + 2));
		}
		m_isClosed = true;

		constractCurveSegments();
		calcIntersections();
	}

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
			if (m_anchorPoints[i].updateControlPoint(AnchorPointRadius() * 2))
			{
				break;
			}
		}

		constractCurveSegments();
		calcIntersections();
	}

	void draw(bool isActive)const
	{
		if (isActive)
		{
			for (const auto& p : m_anchorPoints)
			{
				p.draw(AnchorPointRadius());
			}
		}

		std::vector<Vec2> pp;
		for (const auto& segment : m_curveSegments)
		{
			const auto& curve = segment.curve();

			const int divNum = 30;
			for (int p = 0; p < divNum; ++p)
			{
				const double t = 1.0*p / divNum;
				pp.push_back(curve(t));
			}
		}

		if (!pp.empty())
		{
			if (!m_isClosed)
			{
				pp.push_back(m_anchorPoints.back().anchorPoint());
			}

			//LineString(pp).draw(Palette::Yellow, m_isClosed);

			LineString(pp).draw(Color(isActive ? Palette::Yellow : Palette::White, 64), m_isClosed);

			//Polygon(pp).draw(Color(Palette::White, 64));

			if (isActive)
			{
				for (const auto& curveSegment : m_curveSegments)
				{
					curveSegment.draw();
				}
			}
		}
	}

	void debugDrawPath(const ClipperLib::Path& path)const
	{
		const auto loops = PathToPolygons(path);

		for (const auto& loop : loops)
		{
			if (Input::KeyControl.pressed)
			{
				for (size_t i = 0; i < loop.size(); ++i)
				{
					Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos).drawArrow(1, { 5.0,5.0 }, HSV(30.0*loop[i].m_Z, 1, 1).toColor());
				}
			}
			else
			{
				for (size_t i = 0; i < loop.size(); ++i)
				{
					Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos).draw({ HSV(30.0*loop[i].m_Z, 1, 1).toColor(), HSV(30.0*loop[(i + 1) % loop.size()].m_Z, 1, 1).toColor() });
					//Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos).draw(HSV(15.0*startIndex, 1, 1));
				}
			}
			
		}
	}

	/*
	多角形に変換
	*/
	ClipperLib::Path getPath(size_t startIndex, int divNum)const
	{
		ClipperLib::Path result;

		for (int segment = 0; segment < m_curveSegments.size(); ++segment)
		{
			for (int d = 0; d < divNum; ++d)
			{
				//const double progress = Clamp(1.0*d / (divNum - 1), 0.01, 0.99);
				//const double progress = 1.0*d / (divNum - 1);
				const double progress = 1.0*d / divNum;
				const auto p = m_curveSegments[segment].curve().get(progress).asPoint();
				//result << ClipperLib::IntPoint(p.x, p.y, startIndex + segment);
				result << ClipperLib::IntPoint(p.x * scaleInt, p.y * scaleInt, startIndex + segment);
			}
		}

		return result;
	}

	//二分探索的に計算しようとする方法
	//直線距離は単調増加でないので、ちゃんと曲線に沿った距離の近似をする必要がある
	//始点は入れる
	//終点は入れない
	ClipperLib::Path getPathHighPrecision(size_t startIndex, double interval, double permissibleError = 1.0)const
	{
		ClipperLib::Path result;
		
		for (int segment = 0; segment < m_curveSegments.size(); ++segment)
		{
			const auto& curve = m_curveSegments[segment].curve();
			const Vec2 startPos = curve(0.0);
			result << ClipperLib::IntPoint(startPos.x * scaleInt, startPos.y * scaleInt, startIndex + segment);
			
			const auto binarySearchStep = [&](double prevT, double leftBound, double rightBound)->double
			{
				for (int count = 0; count < 100; ++count)
				{
					double newT = (leftBound + rightBound)*0.5;

					const double distance = curve.length(prevT, newT);
					if ((interval - permissibleError) <= distance && distance <= (interval + permissibleError))//OK!
					{
						const Vec2 newPos = curve(newT);
						result << ClipperLib::IntPoint(newPos.x * scaleInt, newPos.y * scaleInt, startIndex + segment);
						return newT;
					}
					else if (distance < interval - permissibleError)//足りない
					{
						leftBound = newT;
					}
					else//進みすぎ
					{
						rightBound = newT;
					}
				}

				LOG_ERROR(L"収束に失敗");

				return (leftBound + rightBound)*0.5;
			};

			double currentT = 0.0;
			for (; interval + permissibleError < curve.length(currentT, 1);)
			{
				currentT = binarySearchStep(currentT, currentT, 1.0);
			}
		}

		return result;
	}

	std::vector<Vec2> getSegmentPathHighPrecision(size_t segmentIndex, double interval, double permissibleError = 1.0)const
	{
		std::vector<Vec2> result;

		const auto& curve = m_curveSegments[segmentIndex].curve();
		
		result.push_back(curve(0.0));

		const auto binarySearchStep = [&](double prevT, double leftBound, double rightBound)->double
		{
			for (int count = 0; count < 100; ++count)
			{
				double newT = (leftBound + rightBound)*0.5;

				const double distance = curve.length(prevT, newT);
				if ((interval - permissibleError) <= distance && distance <= (interval + permissibleError))//OK!
				{
					const Vec2 newPos = curve(newT);
					result.push_back(newPos);
					return newT;
				}
				else if (distance < interval - permissibleError)//足りない
				{
					leftBound = newT;
				}
				else//進みすぎ
				{
					rightBound = newT;
				}
			}

			LOG_ERROR(L"収束に失敗");

			return (leftBound + rightBound)*0.5;
		};

		double currentT = 0.0;
		for (; interval + permissibleError < curve.length(currentT, 1);)
		{
			currentT = binarySearchStep(currentT, currentT, 1.0);
		}
		
		result.push_back(curve(1.0));

		return result;
	}

	/*
	多角形に変換(パスとの衝突点を考慮して分割する)
	*/
	/*ClipperLib::Path getPathIntersection(size_t startIndex,const BezierCurvePath& other)const
	{
		ClipperLib::Path result;

		return result;
	}*/

	bool isClosed()const
	{
		return m_isClosed;
	}

	size_t numOfSegments()const
	{
		return m_curveSegments.size();
	}

	const CurveSegment& segment(size_t index)const
	{
		return m_curveSegments[index];
	}

	void output(CSVWriter& writer)const
	{
		for (const auto& p : m_anchorPoints)
		{
			writer.write(p[0], p[1], p[2]);
		}
		writer.nextLine();
	}

	void addHole()
	{

	}
		
private:

	void calcIntersections()
	{
		int intersectionCount = 0;

		for (size_t i = 0; i < m_curveSegments.size(); ++i)
		{
			for (size_t j = i + 1; j < m_curveSegments.size(); ++j)
			{
				const auto result = m_curveSegments[i].intersectionPoints(m_curveSegments[j]);
				
				intersectionCount += result.size();

				for (const auto& point : result)
				{
					Circle(point.second, 5).draw(Palette::Red);
				}
			}

			if (auto t = m_curveSegments[i].selfIntersection())
			{
				++intersectionCount;

				const Vec2 pos = m_curveSegments[i].curve().get(t.value().first);
				Circle(pos, AnchorPointRadius()).draw(Color(255, 255, 0));
			}
		}

		Window::SetTitle(intersectionCount);
	}

	/*
	アンカーポイント -> セグメントへの変換
	*/
	void constractCurveSegments()
	{
		m_curveSegments.clear();

		for (size_t i = 0; i + 1 < m_anchorPoints.size() || (m_isClosed && i < m_anchorPoints.size()); ++i)
		{
			const auto l0 = m_anchorPoints[i].outerHandle();
			const auto l1 = m_anchorPoints[(i + 1) % m_anchorPoints.size()].innerHandle();

			m_curveSegments.emplace_back(l0.begin, l0.end, l1.begin, l1.end);
		}
	}

	static double AnchorPointRadius()
	{
		return 5.0;
	}

	std::vector<AnchorPoint> m_anchorPoints;
	std::vector<CurveSegment> m_curveSegments;

	std::vector<std::vector<AnchorPoint>> m_holePoints;
	std::vector<std::vector<CurveSegment>> m_holeSegments;

	bool m_isClosed = false;
};

class LineStringTree
{
public:

	//using SegmentLoop = std::vector<CurveSegment>;
	using SegmentLoop = std::vector<std::pair<CurveSegment, Line>>;

	void draw()
	{
		if (Input::KeyUp.clicked)
		{
			++m_drawDepth;
			m_drawIndex = 0;
		}
		if (Input::KeyDown.clicked)
		{
			--m_drawDepth;
			m_drawIndex = 0;
		}
		m_drawDepth = Clamp(m_drawDepth, 0, 10);

		if (Input::KeyLeft.clicked)
		{
			--m_drawIndex;
		}
		if (Input::KeyRight.clicked)
		{
			++m_drawIndex;
		}

		drawImpl(m_drawDepth, 0, m_drawIndex);
	}

	void add(const LineString& contour)
	{
		m_contours.push_back(contour);
	}
	void nextLineColor()
	{
		m_lines.emplace_back();
	}

	void addLineColor(const Line& line, const Color& color)
	{
		m_lines.back().emplace_back(line, color);
	}

	void addColor(const Color& color)
	{
		m_colors.push_back(color);
	}

	std::shared_ptr<LineStringTree> addChild()
	{
		m_childs.push_back(std::make_shared<LineStringTree>());
		return m_childs.back();
	}

	void setHole(bool isHole)
	{
		m_isHole = isHole;
	}

	void addSegment(const CurveSegment& curveSegment, const Line& line)
	{
		auto& curveSegments = m_loops.back();	
		//curveSegments.push_back(curveSegment);
		curveSegments.emplace_back(curveSegment, line);
	}

	void nextLoop()
	{
		m_loops.emplace_back();
	}

private:

	void drawImpl(int maxDepth, int currentDepth,int drawIndex)
	{
		if (maxDepth < currentDepth)
		{
			return;
		}

		if (maxDepth == currentDepth)
		{
			//直線描画（デバッグ用）
			if (Input::KeyShift.pressed)
			{
				const int currentDrawIndex = (drawIndex % m_contours.size());
				
				if (Input::KeyControl.pressed)
				{
					for (auto i : step(m_contours.size()))
					{
						if (i != currentDrawIndex)continue;

						Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_contours.size(), L", AB: ", Line(*m_contours[i].begin(), *m_contours[i].rbegin()), L", Mouse: ", Mouse::Pos());

						for (int line = 0; line < m_contours[i].num_lines; ++line)
						{
							m_contours[i].line(line).drawArrow(1.0, { 5.0,5.0 }, m_colors[i]);
							if (line == 0)
							{
								Circle(m_contours[i].point(0),5).draw(m_colors[i]);
							}
						}
					}
				}
				else
				{
					for (auto i : step(m_contours.size()))
					{
						if (i != currentDrawIndex)continue;

						Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_contours.size(), L", AB: ", Line(*m_contours[i].begin(), *m_contours[i].rbegin()), L", Mouse: ", Mouse::Pos());

						m_contours[i].draw(m_colors[i]);
					}
				}
			}
			/*if (Input::KeyShift.pressed)
			{
				const int currentDrawIndex = (drawIndex % m_lines.size());

				if (Input::KeyControl.pressed)
				{
					for (auto i : step(m_lines.size()))
					{
						if (i != currentDrawIndex)continue;
						
						Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_lines.size(), L", AB: ", Line(m_lines[i].begin()->first.begin, m_lines[i].rbegin()->first.end), L", Mouse: ", Mouse::Pos());

						for (int line = 0; line < m_lines[i].size(); ++line)
						{
							m_lines[i][line].first.drawArrow(1.0, { 5.0,5.0 }, m_lines[i][line].second);
							if (line == 0)
							{
								Circle(m_lines[i][line].first.begin, 5).draw(m_lines[i][line].second);
							}
						}
					}
				}
				else
				{
					for (auto i : step(m_lines.size()))
					{
						if (i != currentDrawIndex)continue;

						Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_lines.size(), L", AB: ", Line(m_lines[i].begin()->first.begin, m_lines[i].rbegin()->first.end), L", Mouse: ", Mouse::Pos());

						for (int line = 0; line < m_lines[i].size(); ++line)
						{
							m_lines[i][line].first.draw(1.0, m_lines[i][line].second);
						}
					}
				}
			}*/
			//カーブ描画
			else
			{
				const int currentDrawIndex = (drawIndex % m_loops.size());
				Window::SetTitle(L"Depth: ", currentDepth, L", Vert: ", currentDrawIndex, L" of ", m_loops.size());

				for (auto i : step(m_loops.size()))
				{
					if (i != currentDrawIndex)continue;

					//const auto& curveSegment = m_curveSegments[i];

					for (const auto& curveSegmentPair : m_loops[i])
					{
						const auto& curveSegment = curveSegmentPair.first;

						if (Input::KeyControl.pressed)
						{
							curveSegment.curve().drawArrow(8, HSV(15 * i, 1, 1), curveSegment.range().first, curveSegment.range().second);
						}
						else
						{
							curveSegment.curve().draw(30, HSV(15 * i, 1, 1), curveSegment.range().first, curveSegment.range().second);
							if (Input::KeyC.pressed)
							{
								Circle(curveSegmentPair.second.begin, 5).drawFrame(1.0, 0.0, HSV(15 * i, 1, 1));
								Circle(curveSegmentPair.second.end, 5).drawFrame(1.0, 0.0, HSV(15 * i, 1, 1));

							}
							
						}
					}
				}
			}
		}

		for (auto p : m_childs)
		{
			p->drawImpl(maxDepth, currentDepth + 1, drawIndex);
		}
	}

	std::vector<SegmentLoop> m_loops;
	//std::vector<std::pair<SegmentLoop,Line>> m_loops;
	int m_drawIndex = 0;
	int m_drawDepth = 0;
	bool m_isHole = false;

	std::vector<LineString> m_contours;
	std::vector<std::vector<std::pair<Line, Color>>> m_lines;
	std::vector<Color> m_colors;
	std::vector<std::shared_ptr<LineStringTree>> m_childs;
};

class BezierPathClipper
{
public:

	BezierPathClipper()
	{
		m_paths.emplace_back();
		m_editIndex = 0;
	}

	void update()
	{
		if (m_editIndex)
		{
			m_paths[m_editIndex.value()].update();

			if (m_paths[m_editIndex.value()].isClosed() && !Input::MouseR.pressed)
			{
				m_paths.emplace_back();
				m_editIndex = m_paths.size() - 1;
			}
		}

		if (Input::KeyEnter.clicked && 3 == m_paths.size())
		{
			size_t indexOffset = 1;

			//const int divNum = 10000;
			const int divNum = 25;
			
			//auto path1 = m_paths[0].getPath(indexOffset, divNum);
			auto path1 = m_paths[0].getPathHighPrecision(indexOffset, 5.0);
			indexOffset += m_paths[0].numOfSegments();

			//auto path2 = m_paths[1].getPath(indexOffset, divNum);
			auto path2 = m_paths[1].getPathHighPrecision(indexOffset, 5.0);
			indexOffset += m_paths[1].numOfSegments();

			/*
			pathを使ってブーリアン
			*/
			ClipperLib::Clipper clipper;
			clipper.ZFillFunction(ZFillFunc);

			clipper.AddPath(path1, ClipperLib::PolyType::ptSubject, true);
			clipper.AddPath(path2, ClipperLib::PolyType::ptClip, true);

			ClipperLib::PolyTree resultPolyTree;
			/*ClipperLib::Paths resultPaths;
			clipper.Execute(ClipperLib::ClipType::ctUnion, resultPaths);*/
			ClipperLib::Paths resultPaths;
			//clipper.Execute(ClipperLib::ClipType::ctUnion, resultPolyTree);
			//clipper.Execute(ClipperLib::ClipType::ctDifference, resultPolyTree);
			//clipper.Execute(ClipperLib::ClipType::ctIntersection, resultPolyTree);
			clipper.Execute(ClipperLib::ClipType::ctXor, resultPolyTree);
			
			/*
			結果から新たな曲線を構築
			*/

			//drawCurves = makeResultPath(resultPaths);
			//m_segmentInfos = makeResultPath(resultPaths);
			
			m_pTree = makeResult(resultPolyTree);
			
			/*m_paths.emplace_back();
			unpackZIndex;*/
			
		}

		if (Input::KeyControl.pressed && Input::KeyS.clicked)
		{
			auto path = Dialog::GetSave({ ExtensionFilterPair(ExtensionFilter::CSV) });
			if (path)
			{
				CSVWriter writer(path.value());
				for (const auto& path : m_paths)
				{
					if (path.isClosed())
					{
						path.output(writer);
					}
				}
			}/*

			std::vector<std::vector<Vec2>> pts;
			if (path)
			{
				CSVReader reader(path.value());
				for (int row = 0; row < reader.rows; ++row)
				{
					std::vector<Vec2> temp;
					for (int x = 0; x < reader.columns(row); ++x)
					{
						temp.push_back(reader.get<Vec2>(row, x));
					}
					pts.push_back(temp);
				}
			}

			while (System::Update())
			{
				for (auto i : step(pts.size()))
				{
					const auto& pp = pts[i];
					for (const auto& ppp : pp)
					{
						Circle(ppp, 10).draw(HSV(60.0*i, 1, 1));
					}
				}
			}*/

		}

		if (Dragdrop::HasItems())
		{
			const auto filepaths = Dragdrop::GetFilePaths();
			if (!filepaths.empty())
			{
				CSVReader reader(filepaths.front());

				m_paths.clear();
				for (size_t line = 0; line < reader.rows; ++line)
				{
					m_paths.emplace_back(reader, line);
				}

				m_paths.emplace_back();
				m_editIndex = m_paths.size() - 1;
			}
		}
	}

	void draw()
	{
		for(auto i : step(m_paths.size()))
		{
			m_paths[i].draw((m_editIndex ? m_editIndex.value() == i : false) || Input::KeyO.pressed);
		}

		//for (const auto& curve : drawCurves)
		/*for(auto i : step(drawCurves.size()))
		{
			LineString current = drawCurves[i].first;
			current.push_back(drawCurves[(i + 1) % drawCurves.size()].first.point(0));
			current.draw(drawCurves[i].second);
		}*/
		//for (const auto& segmentInfo : m_segmentInfos)
		/*if (m_segmentInfos.empty())
		{
			Window::SetTitle(L"Empty");
		}*/

		if (Input::MouseL.pressed)
		{
			size_t indexOffset = 0;

			//const int divNum = 10000;
			//const int divNum = 25;

			for (auto i : step(m_paths.size()))
			{
				//m_paths[i].debugDrawPath(indexOffset, divNum);
				m_paths[i].debugDrawPath(m_paths[i].getPathHighPrecision(indexOffset, 5.0));
				indexOffset += m_paths[i].numOfSegments();
			};
		}
		else if (m_pTree)
		{
			m_pTree->draw();
		}

		if (Input::KeyD.pressed)
		{
			if (Input::KeyControl.pressed)
			{
				for (const auto& line : linesForDebug)
				{
					line.drawArrow(1, { 5.0,5.0 }, Palette::Cyan);
				}
			}
			else
			{
				for (const auto& line : linesForDebug)
				{
					line.draw(1, Palette::Cyan);
				}
			}
			/*for (const auto& p : intersectionPoints)
			{
				Circle(ClipVertex(p.X, p.Y, p.Z).m_pos, 5).drawFrame(1, 0, HSV(15.0*static_cast<int>(p.Z & INT32_MAX), 1, 1));
			}*/
		}
		
		
		/*if (Input::KeyShift.pressed)
		{
			for (const auto& curve : debugCurves)
			{
				curve.first.draw(curve.second);
			}
		}
		else
		{
			for (auto i : step(m_segmentInfos.size()))
			{
				const auto& segmentInfo = m_segmentInfos[i];
				if (i == 0)
				{
					Window::SetTitle(Vec4(segmentInfo.pathIndex, segmentInfo.segmentIndex, segmentInfo.startT, segmentInfo.endT));
				}

				if (Input::KeyControl.pressed)
				{
					m_paths[segmentInfo.pathIndex].segment(segmentInfo.segmentIndex).curve().drawArrow(8, HSV(15 * i, 1, 1), segmentInfo.startT, segmentInfo.endT);
				}
				else
				{
					m_paths[segmentInfo.pathIndex].segment(segmentInfo.segmentIndex).curve().draw(30, HSV(15 * i, 1, 1), segmentInfo.startT, segmentInfo.endT);
				}
			}
		}
		
		if (Input::KeySpace.pressed)
		{
			for (const auto& pos : m_segmentPoss)
			{
				Circle(pos, 4).drawFrame(0, 1, Palette::Cyan);
			}
		}*/
	}


private:

	/*std::pair<size_t, size_t> unpackZIndex(size_t ZIndex)const
	{
		size_t indexOffset = 0;

		for (size_t path = 0; path < m_paths.size(); ++path)
		{
			size_t nextIndexOffset = indexOffset + m_paths[path].numOfSegments();
			if (ZIndex < nextIndexOffset)
			{
				return{ path,ZIndex - indexOffset };
			}
			indexOffset = nextIndexOffset;
		}

		return{ m_paths.size(),0 };
	}*/


	std::vector<Vec2> getSegmentPath(int zIndex)const
	{
		const double interval = 5.0;
		if (const auto indicesOpt = unpackZIndex(zIndex))
		{
			return m_paths[indicesOpt.value().first].getSegmentPathHighPrecision(indicesOpt.value().second, interval);
		}
		
		return{};
	}

	Optional<BezierCurve> getSegmentCurve(int zIndex)const
	{
		if (const auto indicesOpt = unpackZIndex(zIndex))
		{
			return m_paths[indicesOpt.value().first].segment(indicesOpt.value().second).curve();
		}

		return none;	
	}

	int combineZIndex(const ClipVertex& p1, ClipVertex& p2)const
	{
		const int lower1 = static_cast<int>(p1.m_Z& INT32_MAX);
		const int upper1 = static_cast<int>(p1.m_Z >> 32);
		const int lower2 = static_cast<int>(p2.m_Z& INT32_MAX);
		const int upper2 = static_cast<int>(p2.m_Z >> 32);

		/*if (static_cast<int>(p1.m_pos.x + 0.5) == 412 && static_cast<int>(p1.m_pos.y + 0.5) == 175 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 412 && static_cast<int>(p2.m_pos.y + 0.5) == 175 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 407 && static_cast<int>(p1.m_pos.y + 0.5) == 172 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 407 && static_cast<int>(p2.m_pos.y + 0.5) == 172
			)*/
		/*if (static_cast<int>(p1.m_pos.x + 0.5) == 289 && static_cast<int>(p1.m_pos.y + 0.5) == 472 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 290 && static_cast<int>(p2.m_pos.y + 0.5) == 471 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 307 && static_cast<int>(p1.m_pos.y + 0.5) == 428 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 309 && static_cast<int>(p2.m_pos.y + 0.5) == 422
			)*/
		/*if (static_cast<int>(p1.m_pos.x + 0.5) == 296 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 296 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 606 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 306 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 503 && static_cast<int>(p1.m_pos.y + 0.5) == 70 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 503 && static_cast<int>(p2.m_pos.y + 0.5) == 70
			)
		{
			LOG(L"-------------------------");
			LOG(L"Debug: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
			LOG(L"Debug: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
		}*/

		if (static_cast<int>(p1.m_pos.x + 0.5) == 327 && static_cast<int>(p1.m_pos.y + 0.5) == 293 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 327 && static_cast<int>(p2.m_pos.y + 0.5) == 293 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 333 && static_cast<int>(p1.m_pos.y + 0.5) == 277 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 333 && static_cast<int>(p2.m_pos.y + 0.5) == 277 
			)
		{
			LOG(L"-------------------------");
			LOG(L"Debug: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
			LOG(L"Debug: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
		}

		//連続している（普通のケース）
		if (lower1 == lower2)
		{
			return lower1;
		}
		//このケースはあり得るか？
		else if (upper1 != 0 && upper1 == upper2)
		{
			//LOG(L"NotExpectedData: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
			return upper1;
		}
		//片方のupperのみ存在する場合：片方の端点は交点なので二つインデックスが存在する。
		//このような場合は交点ではない方のインデックスを取ってくればよい
		else if (upper1 == 0 && upper2 != 0)
		{
			return lower1;
		}
		else if (upper2 == 0 && upper1 != 0)
		{
			return lower2;
		}
		//連続はしてないが、どちらの端点も交点ではない
		//次の曲線に繋がるケース
		//恐らく一つ目の曲線を採用すればよい
		//LOG(L"NotExpectedData: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
		//LOG(L"NotExpectedData: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
		/*
		[app]20 th line: (412,175), z1:26
		[app]21 th line: (406.61509,172.05365), z1:25
		[app]22 th line: (389.71378,164.17129), z1:107374182438
		*/

		/*
		どっちかわからないときは、
		・距離が小さければほぼ点なので無視してよい
		・そうでないときは、二つのパス（最初に作った折れ線）に線分をそれぞれ射影して、長さの大きい方を取る
		
		折れ線への線分の射影
		線分の始点と終点についてそれぞれ
		1. 折れ線の全ての線へ下ろした垂線の足の位置を計算する
		2. その位置が折れ線の中にあるような垂線について、最小の長さを持つ垂線の足の位置が射影先となる		
		*/


		/*const Line line(p1.m_pos, p2.m_pos);
		const double lengthSq1 = projectionLengthSq(getSegmentPath(lower1), line);
		const double lengthSq2 = projectionLengthSq(getSegmentPath(lower2), line);

		if (static_cast<int>(p1.m_pos.x + 0.5) == 296 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 296 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 606 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 306 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 503 && static_cast<int>(p1.m_pos.y + 0.5) == 70 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 503 && static_cast<int>(p2.m_pos.y + 0.5) == 70
			)
		{
			LOG(L"Debug: lengthSq1=", lengthSq1);
			LOG(L"Debug: lengthSq2=", lengthSq2);
		}

		return lengthSq1 < lengthSq2 ? lower2 : lower1;*/

		/*{
			LOG(L"-------------------------");
			LOG(L"Debug: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
			LOG(L"Debug: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
		}*/
		
		double lengthSq1 = 0, lengthSq2 = 0;

		Optional<double> curve1T1, curve1T2, curve2T1, curve2T2;
		Optional<BezierCurve> curve1Opt, curve2Opt;

		if(curve1Opt = getSegmentCurve(lower1))
		{
			const auto& curve1 = curve1Opt.value();
			curve1T1 = curve1.closestPointOpt(p1.m_pos, 0.1);
			curve1T2 = curve1.closestPointOpt(p2.m_pos, 0.1);

			if (curve1T1 && curve1T2)
			{
				lengthSq1 = Line(curve1(curve1T1.value()), curve1(curve1T2.value())).lengthSq();
			}
		}

		if(curve2Opt = getSegmentCurve(lower2))
		{
			const auto& curve2 = curve2Opt.value();
			curve2T1 = curve2.closestPointOpt(p1.m_pos, 0.1);
			curve2T2 = curve2.closestPointOpt(p2.m_pos, 0.1);

			if (curve2T1 && curve2T2)
			{
				lengthSq2 = Line(curve2(curve2T1.value()), curve2(curve2T2.value())).lengthSq();
			}
		}

		/*if (static_cast<int>(p1.m_pos.x + 0.5) == 296 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 296 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 606 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 306 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
			static_cast<int>(p1.m_pos.x + 0.5) == 503 && static_cast<int>(p1.m_pos.y + 0.5) == 70 ||
			static_cast<int>(p2.m_pos.x + 0.5) == 503 && static_cast<int>(p2.m_pos.y + 0.5) == 70
			)*/if (static_cast<int>(p1.m_pos.x + 0.5) == 327 && static_cast<int>(p1.m_pos.y + 0.5) == 293 ||
				static_cast<int>(p2.m_pos.x + 0.5) == 327 && static_cast<int>(p2.m_pos.y + 0.5) == 293 ||
				static_cast<int>(p1.m_pos.x + 0.5) == 333 && static_cast<int>(p1.m_pos.y + 0.5) == 277 ||
				static_cast<int>(p2.m_pos.x + 0.5) == 333 && static_cast<int>(p2.m_pos.y + 0.5) == 277
				)
		{
			LOG(L"Debug: Curve1:t1=", curve1T1);
			LOG(L"Debug: Curve1:t2=", curve1T2);
			LOG(L"Debug: Curve2:t1=", curve2T1);
			LOG(L"Debug: Curve2:t2=", curve2T2);

			LOG(L"Debug: lengthSq1=", lengthSq1);
			LOG(L"Debug: lengthSq2=", lengthSq2);
		}

		//p1とp2の間に連続している曲線の境界点が存在する場合、新たに分割点を挿入する必要がある
		if (lengthSq1 == 0 && lengthSq2 == 0)
		{
			if (curve1T1 && curve2T2)
			{
				/*
				//この二点が同じ点を指すはず
				curve1T2 = curve1Opt.value().closestPoint(p2.m_pos);
				curve2T1 = curve2Opt.value().closestPoint(p1.m_pos);
				*/
				p2.m_pos = curve2Opt.value().get(curve2Opt.value().closestPoint(p1.m_pos));
				p2.m_Z = lower2;
				return lower1;
			}
			if (curve1T2 && curve2T1)
			{
				//あり得る？
				LOG_ERROR(L"curve1T2 && curve2T1");
			}
		}
		
		return lengthSq1 < lengthSq2 ? lower2 : lower1;
	}

	/*std::pair<size_t, size_t> unpackZIndex(ClipperLib::cInt Z)const
	{
		const int indexLower = Z & INT32_MAX;

		int indexOffset = 1;

		for (int path = 0; path < m_paths.size(); ++path)
		{
			int nextIndexOffset = indexOffset + static_cast<int>(m_paths[path].numOfSegments());
			if (indexLower < nextIndexOffset)
			{
				return{ static_cast<size_t>(path),static_cast<size_t>(indexLower - indexOffset) };
			}
			indexOffset = nextIndexOffset;
		}

		return{ 0,0 };
	}*/
	
	//std::pair<size_t, size_t> unpackZIndex(int indexZ)const
	Optional<std::pair<size_t, size_t>> unpackZIndex(int indexZ)const
	{
		int indexOffset = 1;

		for (int path = 0; path < m_paths.size(); ++path)
		{
			int nextIndexOffset = indexOffset + static_cast<int>(m_paths[path].numOfSegments());
			if (indexZ < nextIndexOffset)
			{
				return std::pair<size_t, size_t>{ static_cast<size_t>(path),static_cast<size_t>(indexZ - indexOffset) };
			}
			indexOffset = nextIndexOffset;
		}

		//return{ 0,0 };
		return none;
	}

	std::vector<SegmentInfo> makeResultPath(ClipperLib::Paths& paths)
	{
		struct CurveInfo
		{
			size_t pathIndex;
			size_t segmentIndex;
			double t;
		};

		std::vector<Vec2> tempPoints;
		int tempZ = 0;
		for (auto& path : paths)
		{
			for (auto& point : path)
			{
				const Vec2 points = Vec2(point.X, point.Y) / scaleInt;
				
				if (!tempPoints.empty() && tempZ != point.Z)
				{
					tempPoints.push_back(points);
					debugCurves.push_back(std::pair<LineString, Color>(tempPoints, HSV(15 * tempZ, 1, 1)));
					tempPoints.clear();
				}

				tempPoints.push_back(points);
				tempZ = point.Z;
			}
		}

		if (!tempPoints.empty())
		{
			debugCurves.push_back(std::pair<LineString, Color>(tempPoints, HSV(15 * tempZ, 1, 1)));
			tempPoints.clear();
		}

		std::vector<std::pair<Point, int>> curves;

		int currentZ;
		for (auto& path : paths)
		{
			for (auto& point : path)
			{
				currentZ = point.Z;
				if (curves.empty() || curves.back().second != point.Z)
				{
					curves.push_back(std::pair<Point, int>(Point(point.X, point.Y), point.Z));
				}
			}
		}

		std::vector<SegmentInfo> segmentInfos;
		for (auto i : step(curves.size()))
		{
			const Vec2 startPos = curves[i].first / scaleInt;
			const Vec2 endPos = curves[(i + 1) % curves.size()].first / scaleInt;

			m_segmentPoss.push_back(startPos);

			if (const auto indicesOpt = unpackZIndex(curves[i].second))
			{
				const auto indices = indicesOpt.value();
				const double startT = m_paths[indices.first].segment(indices.second).curve().closestPoint(startPos);
				const double endT = m_paths[indices.first].segment(indices.second).curve().closestPoint(endPos);

				segmentInfos.push_back(SegmentInfo{ indices.first, indices.second, startT, endT });
			}
		}

		return segmentInfos;
	}

	std::shared_ptr<LineStringTree> makeResult(ClipperLib::PolyTree& polyTree)
	{
		auto makeLineStringTree = [&](auto rec, std::shared_ptr<LineStringTree> pTree, ClipperLib::PolyNode* node)->void
		{
			//デバッグ用LineStringの構築
			/*{
				std::vector<Vec2> tempPoints;
				int tempZ = 0;
				for (auto& point : node->Contour)
				{
					const Vec2 points = Vec2(point.X, point.Y) / scaleInt;

					if (!tempPoints.empty() && tempZ != point.Z)
					{
						tempPoints.push_back(points);
						pTree->add(LineString(tempPoints));
						pTree->addColor(HSV(15 * tempZ, 1, 1));

						tempPoints.clear();
					}

					tempPoints.push_back(points);
					tempZ = point.Z;
				}

				if (!tempPoints.empty())
				{
					pTree->add(LineString(tempPoints));
					pTree->addColor(HSV(15 * tempZ, 1, 1));
					tempPoints.clear();
				}
			}*/
			{
				const auto loops = PathToPolygons(node->Contour);

				const auto toInt = [](ClipperLib::cInt z)
				{
					return static_cast<int>(z&INT32_MAX) + static_cast<int>(z >> 32);
				};

				int count = 0;
				for (auto& loop : loops)
				{
					ClipperLib::cInt tempZ = 0;
					std::vector<Vec2> tempPoints;
					for (auto& point : loop)
					{
						if (!tempPoints.empty() && tempZ != point.m_Z)
						{
							tempPoints.push_back(point.m_pos);
							pTree->add(LineString(tempPoints));
							pTree->addColor(HSV(15 * toInt(tempZ), 1, 1));
							++count;
							tempPoints.clear();
						}

						tempPoints.push_back(point.m_pos);
						tempZ = point.m_Z;
					}

					if (!tempPoints.empty())
					{
						pTree->add(LineString(tempPoints));
						pTree->addColor(HSV(15 * toInt(tempZ), 1, 1));
						++count;
						tempPoints.clear();
					}
				}
				/*int id = 0;
				for (auto& loop : loops)
				{
					std::vector<Vec2> tempPoints;
					for (auto& point : loop)
					{
						tempPoints.push_back(point.m_pos);
					}

					tempPoints.push_back(tempPoints.front());

					pTree->add(LineString(tempPoints));
					pTree->addColor(HSV(15 * id, 1, 1));
					++id;
				}*/
				
				/*for (auto& loop : loops)
				{
					pTree->nextLineColor();

					for (size_t i = 0; i < loop.size(); ++i)
					{
						pTree->addLineColor(Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos), HSV(30.0*loop[i].m_Z, 1, 1));
					}
				}*/
			}

			//曲線情報
			{
				pTree->setHole(node->IsHole());

				const auto loops = PathToPolygons(node->Contour);

				/*
				std::vector<std::pair<Point, int>> curves;

				int currentZ;
				for (auto& point : node->Contour)
				{
					currentZ = point.Z;
					if (curves.empty() || curves.back().second != point.Z)
					{
						curves.push_back(std::pair<Point, int>(Point(point.X, point.Y), point.Z));
					}
				}

				std::vector<SegmentInfo> segmentInfos;
				for (auto i : step(curves.size()))
				{
					const Vec2 startPos = curves[i].first / scaleInt;
					const Vec2 endPos = curves[(i + 1) % curves.size()].first / scaleInt;

					m_segmentPoss.push_back(startPos);

					const auto indices = unpackZIndex(curves[i].second);

					const auto curve = m_paths[indices.first].segment(indices.second).curve();

					const double startT = curve.closestPoint(startPos);
					const double endT = curve.closestPoint(endPos);

					pTree->addSegment(CurveSegment(curve, startT, endT));
				}
				*/

				using Curves = std::vector<ClipVertex>;
				std::vector<Curves> loopCurves;

				for (const auto& loop : loops)
				{
					Curves curves;
					for (auto& point : loop)
					{
						if (curves.empty() || curves.back().m_Z != point.m_Z)
						{
							curves.push_back(point);
						}
					}
					loopCurves.push_back(curves);
				}

				std::vector<SegmentInfo> segmentInfos;
				for (auto& loopCurve : loopCurves)
				{
					pTree->nextLoop();

					for (auto i : step(loopCurve.size()))
					{
						/*const Vec2 startPos = loopCurve[i].m_pos;
						const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;*/
						
						const int zIndex = combineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()]);
						const Vec2 startPos = loopCurve[i].m_pos;
						const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

						//const int zIndex = combineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()]);
						//const int zIndex = combineZIndex(loopCurve, i, (i + 1) % loopCurve.size());

						//m_lineForDebug.push_back(Line(startPos, endPos));
						//linesForDebug.push_back(Line(startPos, endPos));

						m_segmentPoss.push_back(startPos);

						if (const auto indicesOpt = unpackZIndex(zIndex))
						{
							const auto& indices = indicesOpt.value();

							const auto curve = m_paths[indices.first].segment(indices.second).curve();

							const double startT = curve.closestPoint(startPos);
							const double endT = curve.closestPoint(endPos);

							/*if (zIndex == 39 || zIndex == 40)
							{
								linesForDebug.push_back(Line(curve(startT), curve(endT)));
							}*/
							
							pTree->addSegment(CurveSegment(curve, startT, endT), Line(startPos, endPos));
						}
					}
				}
				
			}

			if (node->ChildCount() == 0)
			{
				return;
			}

			for (int i = 0; i < node->ChildCount(); ++i)
			{
				auto childNode = pTree->addChild();
				rec(rec, childNode, node->Childs[i]);
			}
		};

		//ClipperLib::PolyNode* root = polyTree.GetFirst();
		ClipperLib::PolyNode* root = &polyTree;
		std::shared_ptr<LineStringTree> pTree = std::make_shared<LineStringTree>();
		makeLineStringTree(makeLineStringTree, pTree, root);

		return pTree;
	}

	std::vector<BezierCurvePath> m_paths;
	Optional<size_t> m_editIndex;

	std::vector<std::pair<LineString, Color>> debugCurves;
	std::vector<SegmentInfo> m_segmentInfos;
	std::vector<Vec2> m_segmentPoss;

	//std::vector<Line> m_lineForDebug;

	std::shared_ptr<LineStringTree> m_pTree;
};