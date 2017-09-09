#pragma once
#include <queue>
#include <Siv3D.hpp> // August 2016 v2
#include "include/AnchorPoint.hpp"
#include "include/BoundingRect.hpp"
#include "include/BezierCurve.hpp"
#include "include/CurveSegment.hpp"
#include "include/PolygonSubtractor.hpp"

#include "include/clipper/clipper.hpp"

std::vector<ClipperLib::IntPoint> intersectionPoints;
std::vector<Line> linesForDebug;

double getVerticalDistance(const Vec2& origin, const Vec2& a, const Vec2& b)
{
	const Vec2 project = (a - origin) * (a - origin).dot(b - origin) / (a - origin).lengthSq();
	return ((origin - b) + project).lengthSq();
}

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

inline void TestSave(const ClipperLib::Path& pathA, const ClipperLib::Path& pathB,const String& filename)
{
	std::vector<Vec2> psA, psB;

	for (const auto& point : pathA)
	{
		psA.push_back(ClipVertex(point).m_pos);
	}
	for (const auto& point : pathB)
	{
		psB.push_back(ClipVertex(point).m_pos);
	}

	Image image(Window::Size(), Palette::White);

	LineString(psA).write(image, 2.0, Palette::Blue);
	LineString(psB).write(image, 2.0, Palette::Red);

	image.savePNG(filename);
}

inline void TestSave2(const std::vector<ClipperLib::Path>& paths, const String& filename)
{
	Image image(Window::Size(), Palette::White);

	int count = 0;
	for (const auto& pathA : paths)
	{
		std::vector<Vec2> psA;

		for (const auto& point : pathA)
		{
			psA.push_back(ClipVertex(point).m_pos);
		}

		for (size_t i = 0; i < psA.size(); ++i)
		{
			Line(psA[i], psA[(i + 1) % psA.size()]).writeArrow(image, 1.0, { 5.0,5.0 }, HSV(30 * count, 1, 1));
		}
		++count;
	}

	image.savePNG(filename);
}

using Vertices = std::vector<ClipVertex>;

std::vector<Vertices> PathToPolygons(const ClipperLib::Path& path, bool division = true)
{
	std::vector<ClipVertex> ps;
	std::vector<Vertices> polygons;

	for (int current = 0; current < path.size(); ++current)
	{
		const ClipVertex currentPos(path[current].X, path[current].Y, path[current].Z);
		ps.push_back(currentPos);

		for (int previous = static_cast<int>(ps.size()) - 2; 0 <= previous; --previous)
		{
			if (division && ps[previous].samePos(currentPos))
			{
				polygons.emplace_back(std::vector<ClipVertex>(ps.cbegin() + previous + 1, ps.cend()));
				ps.erase(ps.begin() + previous + 1, ps.end());
			}
		}
	}

	polygons.emplace_back(std::vector<ClipVertex>(ps.begin(), ps.end()));

	return polygons;
}

bool IsClockWise(const Vertices& closedPath)
{
	double sum = 0;

	for (int i = 0; i < closedPath.size(); ++i)
	{
		const auto& p1 = closedPath[i];
		const auto& p2 = closedPath[(i + 1) % closedPath.size()];

		sum += (p2.m_pos.x - p1.m_pos.x)*(p2.m_pos.y + p1.m_pos.y);
	}

	return sum < 0.0;
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

			LineString(pp).draw(Color(isActive ? Palette::Yellow : Palette::Black, 64), m_isClosed);
			
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
		/*
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
		*/

		ClipperLib::Path result;

		for (int segment = 0; segment < m_curveSegments.size(); ++segment)
		{
			//getSpecificPathHighPrecision(result, startIndex, segment, interval, permissibleError);
			m_curveSegments[segment].getSpecificPathHighPrecision(result, startIndex + segment, interval, permissibleError);
		}

		return result;
	}

	//void getSpecificPathHighPrecision(ClipperLib::Path& output, size_t startIndex, size_t segmentIndex, double interval, double permissibleError = 1.0)const
	//{
	//	const auto& curve = m_curveSegments[segmentIndex].curve();
	//	const Vec2 startPos = curve(0.0);
	//	output << ClipperLib::IntPoint(startPos.x * scaleInt, startPos.y * scaleInt, startIndex + segmentIndex);

	//	const auto binarySearchStep = [&](double prevT, double leftBound, double rightBound)->double
	//	{
	//		for (int count = 0; count < 100; ++count)
	//		{
	//			double newT = (leftBound + rightBound)*0.5;

	//			const double distance = curve.length(prevT, newT);
	//			if ((interval - permissibleError) <= distance && distance <= (interval + permissibleError))//OK!
	//			{
	//				const Vec2 newPos = curve(newT);
	//				output << ClipperLib::IntPoint(newPos.x * scaleInt, newPos.y * scaleInt, startIndex + segmentIndex);
	//				return newT;
	//			}
	//			else if (distance < interval - permissibleError)//足りない
	//			{
	//				leftBound = newT;
	//			}
	//			else//進みすぎ
	//			{
	//				rightBound = newT;
	//			}
	//		}

	//		LOG_ERROR(L"収束に失敗");

	//		return (leftBound + rightBound)*0.5;
	//	};

	//	double currentT = 0.0;
	//	for (; interval + permissibleError < curve.length(currentT, 1);)
	//	{
	//		currentT = binarySearchStep(currentT, currentT, 1.0);
	//	}
	//}

	std::vector<Vec2> getSegmentPathHighPrecision(size_t segmentIndex, double interval, double permissibleError = 1.0)const
	{
		std::vector<Vec2> result;

		const auto& curveSegment = m_curveSegments[segmentIndex];
		curveSegment.getSpecificPathHighPrecision(result, interval, permissibleError);

		return result;

		/*
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
		*/
	}
	
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

		//LOG(L"intersectionCount: ", intersectionCount);
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

	bool m_isClosed = false;
};

class LineStringTree
{
public:

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

		Window::SetTitle(L"Depth: ", m_drawDepth, L", Poly: ", m_drawIndex, L"/", m_loops.size());

		drawImpl(m_drawDepth, 0, m_drawIndex);
	}

	void drawAll()
	{
		drawAllImpl();
	}
	
	void add(const LineString& contour)
	{
		m_contours.push_back(contour);
	}

	void nextLineColor(bool isHole)
	{
		(isHole ? m_lineHoles : m_lines).emplace_back();
		(isHole ? m_lineHoleColors : m_lineColors).emplace_back();
	}

	/*void addLineColor(bool isHole, const Line& line, const Color& color)
	{
		(isHole ? m_lineHoles.back() : m_lines.back()).emplace_back(line, color);
	}*/
	void addLineColor(bool isHole, const Vec2& linePos, const Color& color)
	{
		//(isHole ? m_lineHoles.back() : m_lines.back()).emplace_back(line, color);
		(isHole ? m_lineHoles.back() : m_lines.back()).emplace_back(linePos);
		(isHole ? m_lineHoleColors.back() : m_lineColors.back()).emplace_back(color);
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

	void addSegment(bool isHole, const CurveSegment& curveSegment, const Line& line)
	{
		auto& curveSegments = isHole ? m_loopHoles.back() : m_loops.back();
		curveSegments.emplace_back(curveSegment, line);
	}

	void nextLoop(bool isHole)
	{
		(isHole ? m_loopHoles : m_loops).emplace_back();
	}

	bool hasAnyHole()const
	{
		return !m_loopHoles.empty();

		//m_childs;
	}

	size_t numOfLoops()const
	{
		return m_loops.size();
	}

	std::vector<std::pair<CurveSegment, Line>>& getLoopCurve(size_t index)
	{
		return m_loops[index];
	}

	const std::vector<std::pair<CurveSegment, Line>>& getLoopCurve(size_t index)const
	{
		return m_loops[index];
	}

	std::vector<Vec2>& getLoopLines(size_t index)
	{
		return m_lines[index];
	}

	const std::vector<Vec2>& getLoopLines(size_t index)const
	{
		return m_lines[index];
	}

	size_t numOfHoles()const
	{
		return m_loopHoles.size();
	}

	std::vector<std::pair<CurveSegment, Line>>& getHoleCurve(size_t index)
	{
		return m_loopHoles[index];
	}

	const std::vector<std::pair<CurveSegment, Line>>& getHoleCurve(size_t index)const
	{
		return m_loopHoles[index];
	}

	std::vector<Vec2>& getHoleLines(size_t index)
	{
		return m_lineHoles[index];
	}

	const std::vector<Vec2>& getHoleLines(size_t index)const
	{
		return m_lineHoles[index];
	}

	void eraseLoop(size_t loopIndex)
	{
		m_lines.erase(m_lines.begin() + loopIndex);
		m_loops.erase(m_loops.begin() + loopIndex);
	}

	void eraseHole(size_t holeIndex)
	{
		m_lineHoles.erase(m_lineHoles.begin() + holeIndex);
		m_loopHoles.erase(m_loopHoles.begin() + holeIndex);
	}

	ClipperLib::Path getLoopPath(size_t loopIndex, size_t& zIndex, double interval, double permissibleError = 1.0)const
	{
		const auto& loopCurve = getLoopCurve(loopIndex);

		ClipperLib::Path result;

		for (int segment = 0; segment < loopCurve.size(); ++segment)
		{
			loopCurve[segment].first.getSpecificPathHighPrecision(result, zIndex + segment, interval, permissibleError);
		}

		zIndex += loopCurve.size();

		return result;
	}

	ClipperLib::Path getHolePath(size_t holeIndex, size_t& zIndex, double interval, double permissibleError = 1.0)const
	{
		const auto& holeCurve = getHoleCurve(holeIndex);

		ClipperLib::Path result;

		for (int segment = 0; segment < holeCurve.size(); ++segment)
		{
			holeCurve[segment].first.getSpecificPathHighPrecision(result, zIndex + segment, interval, permissibleError);
		}

		zIndex += holeCurve.size();

		return result;
	}

	void append(const std::weak_ptr<LineStringTree> pTreeWeak)
	{
		if (const auto pTree = pTreeWeak.lock())
		{
			LOG(L"Before Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());

			m_loops.insert(m_loops.end(), pTree->m_loops.begin(), pTree->m_loops.end());
			m_loopHoles.insert(m_loopHoles.end(), pTree->m_loopHoles.begin(), pTree->m_loopHoles.end());
			m_contours.insert(m_contours.end(), pTree->m_contours.begin(), pTree->m_contours.end());
			m_lines.insert(m_lines.end(), pTree->m_lines.begin(), pTree->m_lines.end());
			m_lineHoles.insert(m_lineHoles.end(), pTree->m_lineHoles.begin(), pTree->m_lineHoles.end());
			m_lineColors.insert(m_lineColors.end(), pTree->m_lineColors.begin(), pTree->m_lineColors.end());
			m_lineHoleColors.insert(m_lineHoleColors.end(), pTree->m_lineHoleColors.begin(), pTree->m_lineHoleColors.end());

			LOG(L"After Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());
		}
	}

	void logger(const String& tag)
	{
		LOG(tag, L" Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());
	}

	void dump(const String& filename, const Size& imageSize = { 640,480 })
	{
		Image image(imageSize, Palette::White);

		dumpImpl(image);

		//for (auto i : step(m_lines.size()))
		//{
		//	const auto& currentLoop = m_lines[i];
		//	const auto& currentLoopColor = m_lineColors[i];

		//	for (int lineIndex = 0; lineIndex < currentLoop.size(); ++lineIndex)
		//	{
		//		//currentLoop[lineIndex];
		//		//currentLoop[(lineIndex + 1) % currentLoop.size()];
		//		//m_lines[i][lineIndex].first.drawArrow(1.0, { 5.0,5.0 }, m_lines[i][lineIndex].second);
		//		Line(currentLoop[lineIndex], currentLoop[(lineIndex + 1) % currentLoop.size()]).drawArrow(1.0, { 5.0,5.0 }, Palette::Cyan);
		//		if (lineIndex == 0)
		//		{
		//			//Circle(m_lines[i][lineIndex].first.begin, 5).draw(m_lines[i][lineIndex].second);
		//			Circle(currentLoop[lineIndex], 3).draw(Palette::Cyan);
		//		}
		//	}
		//}

		image.savePNG(filename);
	}

private:

	void dumpImpl(Image& image)
	{
		for (int i = 0; i < numOfLoops(); ++i)
		{
			Polygon(getLoopLines(i)).write(image, Color(Palette::Cyan, 64));
		}

		for (int i = 0; i < numOfHoles(); ++i)
		{
			//const auto lines = getHoleLines(i);
			LineString(getHoleLines(i)).write(image, 1.0, Palette::Red, true);
		}

		for (auto p : m_childs)
		{
			p->dumpImpl(image);
		}

	}

	void drawImpl(int maxDepth, int currentDepth, int drawIndex)
	{
		if (maxDepth < currentDepth)
		{
			return;
		}

		if (maxDepth == currentDepth)
		{
			//直線描画（デバッグ用）
			/*if (Input::KeyShift.pressed)
			{
				if (Input::KeyControl.pressed)
				{
					const int currentDrawIndex = (drawIndex % m_contours.size());
					for (auto i : step(m_contours.size()))
					{
						if (i != currentDrawIndex)continue;

						//Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_contours.size(), L", AB: ", Line(*m_contours[i].begin(), *m_contours[i].rbegin()), L", Mouse: ", Mouse::Pos());

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

						//Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_contours.size(), L", AB: ", Line(*m_contours[i].begin(), *m_contours[i].rbegin()), L", Mouse: ", Mouse::Pos());

						m_contours[i].draw(m_colors[i]);
					}
				}
			}*/
			if (Input::KeyShift.pressed)
			{
				if (Input::KeyControl.pressed)
				{
					for (auto i : step(m_lines.size()))
					{
						const int currentDrawIndex = (drawIndex % m_lines.size());
						if (i != currentDrawIndex)continue;
						
						//Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_lines.size(), L", AB: ", Line(m_lines[i].begin()->first.begin, m_lines[i].rbegin()->first.end), L", Mouse: ", Mouse::Pos());
						Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_lines.size(), L", AB: ", Line(m_lines[i].front(), m_lines[i].back()), L", Mouse: ", Mouse::Pos());

						const auto& currentLoop = m_lines[i];
						const auto& currentLoopColor = m_lineColors[i];
						
						for (int lineIndex = 0; lineIndex < currentLoop.size(); ++lineIndex)
						{
							//currentLoop[lineIndex];
							//currentLoop[(lineIndex + 1) % currentLoop.size()];
							//m_lines[i][lineIndex].first.drawArrow(1.0, { 5.0,5.0 }, m_lines[i][lineIndex].second);
							Line(currentLoop[lineIndex], currentLoop[(lineIndex + 1) % currentLoop.size()]).drawArrow(1.0, { 5.0,5.0 }, currentLoopColor[lineIndex]);
							if (lineIndex == 0)
							{
								//Circle(m_lines[i][lineIndex].first.begin, 5).draw(m_lines[i][lineIndex].second);
								Circle(currentLoop[lineIndex], 5).draw(currentLoopColor[lineIndex]);
							}
						}
					}
				}
				else
				{
					for (auto i : step(m_lines.size()))
					{
						const int currentDrawIndex = (drawIndex % m_lines.size());
						if (i != currentDrawIndex)continue;

						//Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_lines.size(), L", AB: ", Line(m_lines[i].begin()->first.begin, m_lines[i].rbegin()->first.end), L", Mouse: ", Mouse::Pos());
						Window::SetTitle(L"Depth: ", currentDepth, L", Line: ", currentDrawIndex, L"/", m_lines.size(), L", AB: ", Line(m_lines[i].front(), m_lines[i].back()), L", Mouse: ", Mouse::Pos());

						const auto& currentLoop = m_lines[i];
						const auto& currentLoopColor = m_lineColors[i];

						for (int lineIndex = 0; lineIndex < currentLoop.size(); ++lineIndex)
						{
							//m_lines[i][line].first.draw(1.0, m_lines[i][line].second);
							Line(currentLoop[lineIndex], currentLoop[(lineIndex + 1) % currentLoop.size()]).draw(currentLoopColor[lineIndex]);
						}
					}
				}
			}
			//カーブ描画
			else
			{
				const auto getColor = [&](bool isHole) {return isHole ? Palette::Red : Palette::Cyan; };

				for (auto i : step(m_loops.size()))
				{
					const int currentDrawIndex = (drawIndex % m_loops.size());
					//Window::SetTitle(L"Depth: ", currentDepth, L", Vert: ", currentDrawIndex, L" of ", m_loops.size());

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
							//curveSegment.curve().draw(30, HSV(15 * i, 1, 1), curveSegment.range().first, curveSegment.range().second);
							//curveSegment.curve().draw(30, getColor(m_loopIsHole[i]), curveSegment.range().first, curveSegment.range().second);
							curveSegment.curve().draw(30, Palette::Cyan, curveSegment.range().first, curveSegment.range().second);
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

	void drawAllImpl()
	{
		//直線描画（デバッグ用）
		if (Input::KeyShift.pressed)
		{
			if (Input::KeyControl.pressed)
			{
				for (auto i : step(m_contours.size()))
				{
					for (int line = 0; line < m_contours[i].num_lines; ++line)
					{
						m_contours[i].line(line).drawArrow(1.0, { 5.0,5.0 }, m_colors[i]);
						if (line == 0)
						{
							Circle(m_contours[i].point(0), 5).draw(m_colors[i]);
						}
					}
				}
			}
			else
			{
				for (auto i : step(m_contours.size()))
				{
					m_contours[i].draw(m_colors[i]);
				}
			}
		}
		//カーブ描画
		else
		{
			const auto getColor = [&](bool isHole) {return isHole ? Palette::Red : Palette::Cyan; };

			for (auto i : step(m_loops.size()))
			{
				for (const auto& curveSegmentPair : m_loops[i])
				{
					const auto& curveSegment = curveSegmentPair.first;

					if (Input::KeyControl.pressed)
					{
						//curveSegment.curve().drawArrow(8, getColor(m_loopIsHole[i]), curveSegment.range().first, curveSegment.range().second);
						curveSegment.curve().drawArrow(8, Palette::Cyan, curveSegment.range().first, curveSegment.range().second);
					}
					else
					{
						//curveSegment.curve().draw(30, getColor(m_loopIsHole[i]), curveSegment.range().first, curveSegment.range().second);
						curveSegment.curve().draw(30, Palette::Cyan, curveSegment.range().first, curveSegment.range().second);
						if (Input::KeyC.pressed)
						{
							/*Circle(curveSegmentPair.second.begin, 5).drawFrame(1.0, 0.0, getColor(m_loopIsHole[i]));
							Circle(curveSegmentPair.second.end, 5).drawFrame(1.0, 0.0, getColor(m_loopIsHole[i]));*/
							Circle(curveSegmentPair.second.begin, 5).drawFrame(1.0, 0.0, Palette::Cyan);
							Circle(curveSegmentPair.second.end, 5).drawFrame(1.0, 0.0, Palette::Cyan);
						}
					}
				}
			}
		}

		for (auto p : m_childs)
		{
			p->drawAllImpl();
		}
	}

	std::vector<SegmentLoop> m_loops;
	std::vector<SegmentLoop> m_loopHoles;
	//std::vector<char> m_loopIsHole;
	
	//std::vector<std::pair<SegmentLoop,Line>> m_loops;
	int m_drawIndex = 0;
	int m_drawDepth = 0;
	//bool m_isHole = false;

	std::vector<LineString> m_contours;
	/*std::vector<std::vector<std::pair<Line, Color>>> m_lines;
	std::vector<std::vector<std::pair<Line, Color>>> m_lineHoles;*/
	std::vector<std::vector<Vec2>> m_lines;
	std::vector<std::vector<Vec2>> m_lineHoles;
	std::vector<std::vector<Color>> m_lineColors;
	std::vector<std::vector<Color>> m_lineHoleColors;

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
			const double polygonizeInterval = 5.0;

			{
				size_t indexOffset = 1;

				auto path1 = m_paths[0].getPathHighPrecision(indexOffset, polygonizeInterval);
				indexOffset += m_paths[0].numOfSegments();

				auto path2 = m_paths[1].getPathHighPrecision(indexOffset, polygonizeInterval);
				indexOffset += m_paths[1].numOfSegments();

				/*
				pathを使ってブーリアン
				*/
				ClipperLib::Clipper clipper;
				clipper.ZFillFunction(ZFillFunc);

				clipper.AddPath(path1, ClipperLib::PolyType::ptSubject, true);
				clipper.AddPath(path2, ClipperLib::PolyType::ptClip, true);

				//ClipperLib::PolyTree resultPolyTree;
				ClipperLib::Paths resultPaths;
				clipper.Execute(ClipperLib::ClipType::ctXor, resultPaths);
				//clipper.Execute(ClipperLib::ClipType::ctDifference, resultPaths);

				//drawCurves = makeResultPath(resultPaths);
				//m_segmentInfos = makeResultPath(resultPaths);

				//m_pTree = makeResult(resultPolyTree);
				m_pTree = MakeResultPath(resultPaths, m_paths);
				m_pTree->logger(L"After MakeResultPath");
			}

			class SegmentsHolder
			{
			public:
				
				std::vector<Vec2> getSegmentPathHighPrecision(size_t segmentIndex, double interval, double permissibleError = 1.0)const
				{
					std::vector<Vec2> result;
					m_curveSegmentPtrs[segmentIndex]->getSpecificPathHighPrecision(result, interval, permissibleError);
					return result;
				}

				const CurveSegment& segment(size_t index)const
				{
					return *m_curveSegmentPtrs[index];
				}

				size_t numOfSegments()const
				{
					return m_curveSegmentPtrs.size();
				}

				void add(const CurveSegment* ptr)
				{
					m_curveSegmentPtrs.push_back(ptr);
				}

			private:

				std::vector<const CurveSegment*> m_curveSegmentPtrs;

			};

			m_pTree->dump(L"original.png");

			int processCount = 0;

			for (size_t holeIndex = 0; m_pTree->hasAnyHole();)
			{
				auto holeVertices = m_pTree->getHoleLines(holeIndex);
				std::reverse(holeVertices.begin(), holeVertices.end());
				const Polygon polygonHole(holeVertices);

				size_t indexOffset = 1;
				auto pathHole = m_pTree->getHolePath(holeIndex, indexOffset, polygonizeInterval);

				SegmentsHolder holeSegmentsHolder;

				const auto holeSegments = m_pTree->getHoleCurve(holeIndex);
				for (const auto& holeSegment : holeSegments)
				{
					holeSegmentsHolder.add(&holeSegment.first);
				}

				int iterationNums = m_pTree->numOfLoops();
				for (int loopIndex = 0; loopIndex < iterationNums;)
				{
					const Polygon polygonLoop(m_pTree->getLoopLines(loopIndex));

					if (polygonHole.intersects(polygonLoop))
					{
						LOG(L"穴の除去, Hole: ", m_pTree->numOfHoles(), L", Loop: ", m_pTree->numOfLoops());

						SegmentsHolder loopSegmentsHolder;

						const auto loopSegments = m_pTree->getLoopCurve(loopIndex);
						for (const auto& loopSegment : loopSegments)
						{
							loopSegmentsHolder.add(&loopSegment.first);
						}

						//1回のクリップ毎に、インデックスは振りなおす
						size_t currentIndexOffset = indexOffset;
						auto pathLoop = m_pTree->getLoopPath(loopIndex, currentIndexOffset, polygonizeInterval);

						/*ClipperLib::Clipper clipper;
						clipper.ZFillFunction(ZFillFunc);

						clipper.AddPath(pathLoop, ClipperLib::PolyType::ptSubject, true);
						clipper.AddPath(pathHole, ClipperLib::PolyType::ptClip, true);

						ClipperLib::Paths resultPaths;
						clipper.Execute(ClipperLib::ClipType::ctXor, resultPaths);*/

						ClipperLib::Paths resultPaths = PolygonSubtract(pathLoop, pathHole);

						std::shared_ptr<LineStringTree> pTree2 = MakeResultPathWithoutDivision(resultPaths, std::vector<SegmentsHolder>({ holeSegmentsHolder, loopSegmentsHolder }));

						++processCount;
						//m_pTree->dump(Format(L"process_", processCount, L".png"));
						TestSave(pathLoop, pathHole, Format(L"process_", processCount, L".png"));
						TestSave2(resultPaths, Format(L"process_", processCount, L"_result.png"));

						if (pTree2->hasAnyHole())
						{
							LOG_ERROR(L"穴の除去に失敗");
						}
						else
						{
							if (pTree2->numOfLoops() == 1)
							{
								const Polygon newPolygonLoop(pTree2->getLoopLines(0));
								LOG_ERROR(L"Before erode area: ", polygonLoop.area(), L", After erode area: ", newPolygonLoop.area());
								m_pTree->append(pTree2);
							}
							else
							{
								LOG_ERROR(L"Loop was divided to ", pTree2->numOfLoops(), L" new loops");
								m_pTree->append(pTree2);
							}

							m_pTree->eraseLoop(loopIndex);
							--iterationNums;
						}
					}
					else
					{
						++loopIndex;
						continue;
					}
				}

				m_pTree->eraseHole(0);
			}
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
			//m_pTree->drawAll();
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

	template<class PathHolderType>
	static std::vector<Vec2> GetSegmentPath(int zIndex, const PathHolderType& originalPaths)
	{
		const double interval = 5.0;
		if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
		{
			return originalPaths[indicesOpt.value().first].getSegmentPathHighPrecision(indicesOpt.value().second, interval);
		}
		
		return{};
	}
	
	template<class PathHolderType>
	static Optional<BezierCurve> GetSegmentCurve(int zIndex, const PathHolderType& originalPaths)
	{
		if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
		{
			return originalPaths[indicesOpt.value().first].segment(indicesOpt.value().second).curve();
		}

		return none;	
	}

	template<class PathHolderType>
	static int CombineZIndex(const ClipVertex& p1, ClipVertex& p2, const PathHolderType& originalPaths)
	{
		const int lower1 = static_cast<int>(p1.m_Z & INT32_MAX);
		const int upper1 = static_cast<int>(p1.m_Z >> 32);
		const int lower2 = static_cast<int>(p2.m_Z & INT32_MAX);
		const int upper2 = static_cast<int>(p2.m_Z >> 32);

		//連続している（普通のケース）
		if (lower1 == lower2)
		{
			return lower1;
		}
		//あり得る？
		else if (upper1 != 0 && upper1 == upper2)
		{
			LOG_ERROR(L"L(", __LINE__, L"): upper1 != 0 && upper1 == upper2");
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
		//次の曲線に繋がるケース
		/*
		どっちかわからないときは、
		・距離が小さければほぼ点なので無視してよい
		・そうでないときは、二つのパス（最初に作った折れ線）に線分をそれぞれ射影して、長さの大きい方を取る

		折れ線への線分の射影
		線分の始点と終点についてそれぞれ
		1. 折れ線の全ての線へ下ろした垂線の足の位置を計算する
		2. その位置が折れ線の中にあるような垂線について、最小の長さを持つ垂線の足の位置が射影先となる
		*/
		
		double lengthSq1 = 0, lengthSq2 = 0;

		Optional<double> curve1T1, curve1T2, curve2T1, curve2T2;
		Optional<BezierCurve> curve1Opt, curve2Opt;

		if (curve1Opt = GetSegmentCurve(lower1, originalPaths))
		{
			const auto& curve1 = curve1Opt.value();
			curve1T1 = curve1.closestPointOpt(p1.m_pos, 0.1);
			curve1T2 = curve1.closestPointOpt(p2.m_pos, 0.1);

			if (curve1T1 && curve1T2)
			{
				lengthSq1 = Line(curve1(curve1T1.value()), curve1(curve1T2.value())).lengthSq();
			}
		}

		if (curve2Opt = GetSegmentCurve(lower2, originalPaths))
		{
			const auto& curve2 = curve2Opt.value();
			curve2T1 = curve2.closestPointOpt(p1.m_pos, 0.1);
			curve2T2 = curve2.closestPointOpt(p2.m_pos, 0.1);

			if (curve2T1 && curve2T2)
			{
				lengthSq2 = Line(curve2(curve2T1.value()), curve2(curve2T2.value())).lengthSq();
			}
		}
		
		//p1とp2の間に連続している曲線の境界点が存在する場合、分割点を移動する必要がある
		if (lengthSq1 == 0 && lengthSq2 == 0)
		{
			if (curve1T1 && curve2T2)
			{
				/*
				//この二点が同じ点を指すはず
				curve1T2 = curve1Opt.value().closestPoint(p2.m_pos);
				curve2T1 = curve2Opt.value().closestPoint(p1.m_pos);
				*/
				const Vec2 moveTo = curve2Opt.value().get(curve2Opt.value().closestPoint(p1.m_pos));
				LOG(L"Path point ", p2.m_pos, L" is moved to ", moveTo);
				p2.m_pos = moveTo;
				p2.m_Z = lower2;

				return lower1;
			}
			if (curve1T2 && curve2T1)
			{
				//あり得る？
				LOG_ERROR(L"L(", __LINE__, L"): curve1T2 && curve2T1");
			}
		}

		return lengthSq1 < lengthSq2 ? lower2 : lower1;
	}
	
	//std::pair<size_t, size_t> unpackZIndex(int indexZ)const
	template<class PathHolderType>
	static Optional<std::pair<size_t, size_t>> UnpackZIndex(int indexZ, const PathHolderType& originalPaths)
	{
		int indexOffset = 1;

		for (int path = 0; path < originalPaths.size(); ++path)
		{
			int nextIndexOffset = indexOffset + static_cast<int>(originalPaths[path].numOfSegments());
			if (indexZ < nextIndexOffset)
			{
				return std::pair<size_t, size_t>{ static_cast<size_t>(path),static_cast<size_t>(indexZ - indexOffset) };
			}
			indexOffset = nextIndexOffset;
		}

		return none;
	}

	template<class PathHolderType>
	static std::shared_ptr<LineStringTree> MakeResultPath(const ClipperLib::Paths& resultPaths, const PathHolderType& originalPaths)
	{
		std::shared_ptr<LineStringTree> pTree = std::make_shared<LineStringTree>();

		for (const auto& contour : resultPaths)
		{
			const auto loops = PathToPolygons(contour);

			std::vector<char> isHoles;

			//ポリゴン取得（デバッグ用・穴の包含判定用）
			{
				const auto toInt = [](ClipperLib::cInt z)
				{
					return static_cast<int>(z&INT32_MAX) + static_cast<int>(z >> 32);
				};

				/*int count = 0;
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
				}*/

				int count = 0;
				for (const auto& loop : loops)
				{
					isHoles.push_back(!IsClockWise(loop));

					pTree->nextLineColor(isHoles.back());

					for (size_t i = 0; i < loop.size(); ++i)
					{
						//pTree->addLineColor(isHoles.back(), Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos), HSV(15.0*count, 1, 1));
						pTree->addLineColor(isHoles.back(), loop[i].m_pos, HSV(15.0*count, 1, 1));
					}

					pTree->logger(Format(L"A(", count, L")"));

					++count;
				}
			}

			using Curves = std::vector<ClipVertex>;
			std::vector<Curves> loopCurves;

			for (const auto& loop : loops)
			{
				Curves curves;
				for (const auto& point : loop)
				{
					if (curves.empty() || curves.back().m_Z != point.m_Z)
					{
						curves.push_back(point);
					}
				}
				loopCurves.push_back(curves);
			}

			int loopCount = 0;
			std::vector<SegmentInfo> segmentInfos;
			for (auto& loopCurve : loopCurves)
			{
				pTree->nextLoop(isHoles[loopCount]);

				for (auto i : step(loopCurve.size()))
				{
					const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], originalPaths);

					const Vec2 startPos = loopCurve[i].m_pos;
					const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

					//m_segmentPoss.push_back(startPos);

					if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
					{
						const auto& indices = indicesOpt.value();

						const auto curve = originalPaths[indices.first].segment(indices.second).curve();

						const double startT = curve.closestPoint(startPos);
						const double endT = curve.closestPoint(endPos);

						pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));
					}
				}

				pTree->logger(Format(L"B(", loopCount, L")"));

				++loopCount;
			}
		}

		return pTree;
	}

	template<class PathHolderType>
	static std::shared_ptr<LineStringTree> MakeResultPathWithoutDivision(const ClipperLib::Paths& resultPaths, const PathHolderType& originalPaths)
	{
		std::shared_ptr<LineStringTree> pTree = std::make_shared<LineStringTree>();

		for (const auto& contour : resultPaths)
		{
			//MakeResultPathとの差分はここだけ（いまのところ）
			const auto loops = PathToPolygons(contour, false);

			std::vector<char> isHoles;

			//ポリゴン取得（デバッグ用・穴の包含判定用）
			{
				const auto toInt = [](ClipperLib::cInt z)
				{
					return static_cast<int>(z&INT32_MAX) + static_cast<int>(z >> 32);
				};

				int count = 0;
				for (const auto& loop : loops)
				{
					isHoles.push_back(!IsClockWise(loop));

					pTree->nextLineColor(isHoles.back());

					for (size_t i = 0; i < loop.size(); ++i)
					{
						//pTree->addLineColor(isHoles.back(), Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos), HSV(15.0*count, 1, 1));
						pTree->addLineColor(isHoles.back(), loop[i].m_pos, HSV(15.0*count, 1, 1));
					}

					pTree->logger(Format(L"A(", count, L")"));

					++count;
				}
			}

			using Curves = std::vector<ClipVertex>;
			std::vector<Curves> loopCurves;

			for (const auto& loop : loops)
			{
				Curves curves;
				for (const auto& point : loop)
				{
					if (curves.empty() || curves.back().m_Z != point.m_Z)
					{
						curves.push_back(point);
					}
				}
				loopCurves.push_back(curves);
			}

			int loopCount = 0;
			std::vector<SegmentInfo> segmentInfos;
			for (auto& loopCurve : loopCurves)
			{
				pTree->nextLoop(isHoles[loopCount]);

				for (auto i : step(loopCurve.size()))
				{
					const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], originalPaths);

					const Vec2 startPos = loopCurve[i].m_pos;
					const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

					//m_segmentPoss.push_back(startPos);

					if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
					{
						const auto& indices = indicesOpt.value();

						const auto curve = originalPaths[indices.first].segment(indices.second).curve();

						const double startT = curve.closestPoint(startPos);
						const double endT = curve.closestPoint(endPos);

						pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));
					}
				}

				pTree->logger(Format(L"B(", loopCount, L")"));

				++loopCount;
			}
		}

		return pTree;
	}

	//std::shared_ptr<LineStringTree> makeResultPath(ClipperLib::Paths& paths)
	//{
	//	std::shared_ptr<LineStringTree> pTree = std::make_shared<LineStringTree>();

	//	std::vector<char> isHoles;
	//	
	//	for (auto& contour : paths)
	//	{
	//		const auto loops = PathToPolygons(contour);

	//		//ポリゴン取得（デバッグ用・穴の包含判定用）
	//		{
	//			const auto toInt = [](ClipperLib::cInt z)
	//			{
	//				return static_cast<int>(z&INT32_MAX) + static_cast<int>(z >> 32);
	//			};

	//			/*int count = 0;
	//			for (auto& loop : loops)
	//			{
	//				ClipperLib::cInt tempZ = 0;
	//				std::vector<Vec2> tempPoints;
	//				for (auto& point : loop)
	//				{
	//					if (!tempPoints.empty() && tempZ != point.m_Z)
	//					{
	//						tempPoints.push_back(point.m_pos);
	//						pTree->add(LineString(tempPoints));
	//						pTree->addColor(HSV(15 * toInt(tempZ), 1, 1));
	//						++count;
	//						tempPoints.clear();
	//					}

	//					tempPoints.push_back(point.m_pos);
	//					tempZ = point.m_Z;
	//				}

	//				if (!tempPoints.empty())
	//				{
	//					pTree->add(LineString(tempPoints));
	//					pTree->addColor(HSV(15 * toInt(tempZ), 1, 1));
	//					++count;
	//					tempPoints.clear();
	//				}
	//			}*/
	//			
	//			int count = 0;
	//			for (auto& loop : loops)
	//			{
	//				isHoles.push_back(!IsClockWise(loop));

	//				pTree->nextLineColor(isHoles.back());

	//				for (size_t i = 0; i < loop.size(); ++i)
	//				{
	//					//pTree->addLineColor(isHoles.back(), Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos), HSV(15.0*count, 1, 1));
	//					pTree->addLineColor(isHoles.back(), loop[i].m_pos, HSV(15.0*count, 1, 1));
	//				}

	//				++count;
	//			}
	//		}

	//		using Curves = std::vector<ClipVertex>;
	//		std::vector<Curves> loopCurves;

	//		for (const auto& loop : loops)
	//		{
	//			Curves curves;
	//			for (auto& point : loop)
	//			{
	//				if (curves.empty() || curves.back().m_Z != point.m_Z)
	//				{
	//					curves.push_back(point);
	//				}
	//			}
	//			loopCurves.push_back(curves);
	//		}

	//		int loopCount = 0;
	//		std::vector<SegmentInfo> segmentInfos;
	//		for (auto& loopCurve : loopCurves)
	//		{
	//			pTree->nextLoop(isHoles[loopCount]);

	//			for (auto i : step(loopCurve.size()))
	//			{
	//				const int zIndex = combineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()]);

	//				const Vec2 startPos = loopCurve[i].m_pos;
	//				const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

	//				m_segmentPoss.push_back(startPos);

	//				if (const auto indicesOpt = unpackZIndex(zIndex))
	//				{
	//					const auto& indices = indicesOpt.value();

	//					const auto curve = m_paths[indices.first].segment(indices.second).curve();

	//					const double startT = curve.closestPoint(startPos);
	//					const double endT = curve.closestPoint(endPos);
	//					
	//					pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));
	//				}
	//			}

	//			++loopCount;
	//		}
	//	}

	//	return pTree;
	//}

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

			std::vector<char> isHoles;
			{
				const auto loops = PathToPolygons(node->Contour);

				const auto toInt = [](ClipperLib::cInt z)
				{
					return static_cast<int>(z&INT32_MAX) + static_cast<int>(z >> 32);
				};

				int count = 0;
				for (auto& loop : loops)
				{
					isHoles.push_back(!IsClockWise(loop));

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
						//pTree->addLineColor(Line(loop[i].m_pos, loop[(i + 1) % loop.size()].m_pos), HSV(30.0*loop[i].m_Z, 1, 1));
						pTree->addLineColor(loop[i].m_pos, HSV(30.0*loop[i].m_Z, 1, 1));
					}
				}*/
			}

			//曲線情報
			{
				//pTree->setHole(node->IsHole());
				

				const auto loops = PathToPolygons(node->Contour);

				int loopCount = 0;

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
					pTree->nextLoop(isHoles[loopCount]);

					for (auto i : step(loopCurve.size()))
					{
						/*const Vec2 startPos = loopCurve[i].m_pos;
						const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;*/
						
						const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], m_paths);
						const Vec2 startPos = loopCurve[i].m_pos;
						const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

						//const int zIndex = combineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()]);
						//const int zIndex = combineZIndex(loopCurve, i, (i + 1) % loopCurve.size());

						//m_lineForDebug.push_back(Line(startPos, endPos));
						//linesForDebug.push_back(Line(startPos, endPos));

						//m_segmentPoss.push_back(startPos);

						if (const auto indicesOpt = UnpackZIndex(zIndex, m_paths))
						{
							const auto& indices = indicesOpt.value();

							const auto curve = m_paths[indices.first].segment(indices.second).curve();

							const double startT = curve.closestPoint(startPos);
							const double endT = curve.closestPoint(endPos);

							/*if (zIndex == 39 || zIndex == 40)
							{
								linesForDebug.push_back(Line(curve(startT), curve(endT)));
							}*/
							
							pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));
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
	//std::vector<Vec2> m_segmentPoss;

	//std::vector<Line> m_lineForDebug;

	std::shared_ptr<LineStringTree> m_pTree;
};

	//int combineZIndex(const ClipVertex& p1, ClipVertex& p2)const
	//{
	//	const int lower1 = static_cast<int>(p1.m_Z& INT32_MAX);
	//	const int upper1 = static_cast<int>(p1.m_Z >> 32);
	//	const int lower2 = static_cast<int>(p2.m_Z& INT32_MAX);
	//	const int upper2 = static_cast<int>(p2.m_Z >> 32);

	//	/*if (static_cast<int>(p1.m_pos.x + 0.5) == 412 && static_cast<int>(p1.m_pos.y + 0.5) == 175 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 412 && static_cast<int>(p2.m_pos.y + 0.5) == 175 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 407 && static_cast<int>(p1.m_pos.y + 0.5) == 172 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 407 && static_cast<int>(p2.m_pos.y + 0.5) == 172
	//		)*/
	//	/*if (static_cast<int>(p1.m_pos.x + 0.5) == 289 && static_cast<int>(p1.m_pos.y + 0.5) == 472 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 290 && static_cast<int>(p2.m_pos.y + 0.5) == 471 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 307 && static_cast<int>(p1.m_pos.y + 0.5) == 428 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 309 && static_cast<int>(p2.m_pos.y + 0.5) == 422
	//		)*/
	//	/*if (static_cast<int>(p1.m_pos.x + 0.5) == 296 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 296 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 606 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 306 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 503 && static_cast<int>(p1.m_pos.y + 0.5) == 70 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 503 && static_cast<int>(p2.m_pos.y + 0.5) == 70
	//		)
	//	{
	//		LOG(L"-------------------------");
	//		LOG(L"Debug: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
	//		LOG(L"Debug: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
	//	}*/

	//	if (static_cast<int>(p1.m_pos.x + 0.5) == 327 && static_cast<int>(p1.m_pos.y + 0.5) == 293 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 327 && static_cast<int>(p2.m_pos.y + 0.5) == 293 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 333 && static_cast<int>(p1.m_pos.y + 0.5) == 277 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 333 && static_cast<int>(p2.m_pos.y + 0.5) == 277 
	//		)
	//	{
	//		LOG(L"-------------------------");
	//		LOG(L"Debug: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
	//		LOG(L"Debug: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
	//	}

	//	//連続している（普通のケース）
	//	if (lower1 == lower2)
	//	{
	//		return lower1;
	//	}
	//	//このケースはあり得るか？
	//	else if (upper1 != 0 && upper1 == upper2)
	//	{
	//		//LOG(L"NotExpectedData: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
	//		return upper1;
	//	}
	//	//片方のupperのみ存在する場合：片方の端点は交点なので二つインデックスが存在する。
	//	//このような場合は交点ではない方のインデックスを取ってくればよい
	//	else if (upper1 == 0 && upper2 != 0)
	//	{
	//		return lower1;
	//	}
	//	else if (upper2 == 0 && upper1 != 0)
	//	{
	//		return lower2;
	//	}
	//	//連続はしてないが、どちらの端点も交点ではない
	//	//次の曲線に繋がるケース
	//	//恐らく一つ目の曲線を採用すればよい
	//	//LOG(L"NotExpectedData: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
	//	//LOG(L"NotExpectedData: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
	//	/*
	//	[app]20 th line: (412,175), z1:26
	//	[app]21 th line: (406.61509,172.05365), z1:25
	//	[app]22 th line: (389.71378,164.17129), z1:107374182438
	//	*/

	//	/*
	//	どっちかわからないときは、
	//	・距離が小さければほぼ点なので無視してよい
	//	・そうでないときは、二つのパス（最初に作った折れ線）に線分をそれぞれ射影して、長さの大きい方を取る
	//	
	//	折れ線への線分の射影
	//	線分の始点と終点についてそれぞれ
	//	1. 折れ線の全ての線へ下ろした垂線の足の位置を計算する
	//	2. その位置が折れ線の中にあるような垂線について、最小の長さを持つ垂線の足の位置が射影先となる		
	//	*/

	//	/*const Line line(p1.m_pos, p2.m_pos);
	//	const double lengthSq1 = projectionLengthSq(getSegmentPath(lower1), line);
	//	const double lengthSq2 = projectionLengthSq(getSegmentPath(lower2), line);

	//	if (static_cast<int>(p1.m_pos.x + 0.5) == 296 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 296 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 606 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 306 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 503 && static_cast<int>(p1.m_pos.y + 0.5) == 70 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 503 && static_cast<int>(p2.m_pos.y + 0.5) == 70
	//		)
	//	{
	//		LOG(L"Debug: lengthSq1=", lengthSq1);
	//		LOG(L"Debug: lengthSq2=", lengthSq2);
	//	}

	//	return lengthSq1 < lengthSq2 ? lower2 : lower1;*/

	//	/*{
	//		LOG(L"-------------------------");
	//		LOG(L"Debug: P1=", p1.m_pos, L", lower=", lower1, L", upper=", upper1);
	//		LOG(L"Debug: P2=", p2.m_pos, L", lower=", lower2, L", upper=", upper2);
	//	}*/
	//	
	//	double lengthSq1 = 0, lengthSq2 = 0;

	//	Optional<double> curve1T1, curve1T2, curve2T1, curve2T2;
	//	Optional<BezierCurve> curve1Opt, curve2Opt;

	//	if(curve1Opt = getSegmentCurve(lower1))
	//	{
	//		const auto& curve1 = curve1Opt.value();
	//		curve1T1 = curve1.closestPointOpt(p1.m_pos, 0.1);
	//		curve1T2 = curve1.closestPointOpt(p2.m_pos, 0.1);

	//		if (curve1T1 && curve1T2)
	//		{
	//			lengthSq1 = Line(curve1(curve1T1.value()), curve1(curve1T2.value())).lengthSq();
	//		}
	//	}

	//	if(curve2Opt = getSegmentCurve(lower2))
	//	{
	//		const auto& curve2 = curve2Opt.value();
	//		curve2T1 = curve2.closestPointOpt(p1.m_pos, 0.1);
	//		curve2T2 = curve2.closestPointOpt(p2.m_pos, 0.1);

	//		if (curve2T1 && curve2T2)
	//		{
	//			lengthSq2 = Line(curve2(curve2T1.value()), curve2(curve2T2.value())).lengthSq();
	//		}
	//	}

	//	/*if (static_cast<int>(p1.m_pos.x + 0.5) == 296 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 296 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 606 && static_cast<int>(p1.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 306 && static_cast<int>(p2.m_pos.y + 0.5) == 34 ||
	//		static_cast<int>(p1.m_pos.x + 0.5) == 503 && static_cast<int>(p1.m_pos.y + 0.5) == 70 ||
	//		static_cast<int>(p2.m_pos.x + 0.5) == 503 && static_cast<int>(p2.m_pos.y + 0.5) == 70
	//		)*/if (static_cast<int>(p1.m_pos.x + 0.5) == 327 && static_cast<int>(p1.m_pos.y + 0.5) == 293 ||
	//			static_cast<int>(p2.m_pos.x + 0.5) == 327 && static_cast<int>(p2.m_pos.y + 0.5) == 293 ||
	//			static_cast<int>(p1.m_pos.x + 0.5) == 333 && static_cast<int>(p1.m_pos.y + 0.5) == 277 ||
	//			static_cast<int>(p2.m_pos.x + 0.5) == 333 && static_cast<int>(p2.m_pos.y + 0.5) == 277
	//			)
	//	{
	//		LOG(L"Debug: Curve1:t1=", curve1T1);
	//		LOG(L"Debug: Curve1:t2=", curve1T2);
	//		LOG(L"Debug: Curve2:t1=", curve2T1);
	//		LOG(L"Debug: Curve2:t2=", curve2T2);

	//		LOG(L"Debug: lengthSq1=", lengthSq1);
	//		LOG(L"Debug: lengthSq2=", lengthSq2);
	//	}

	//	//p1とp2の間に連続している曲線の境界点が存在する場合、分割点を移動する必要がある
	//	if (lengthSq1 == 0 && lengthSq2 == 0)
	//	{
	//		if (curve1T1 && curve2T2)
	//		{
	//			/*
	//			//この二点が同じ点を指すはず
	//			curve1T2 = curve1Opt.value().closestPoint(p2.m_pos);
	//			curve2T1 = curve2Opt.value().closestPoint(p1.m_pos);
	//			*/
	//			const Vec2 moveTo = curve2Opt.value().get(curve2Opt.value().closestPoint(p1.m_pos));
	//			LOG(L"Path point ", p2.m_pos, L" is moved to ", moveTo);
	//			p2.m_pos = moveTo;
	//			p2.m_Z = lower2;

	//			return lower1;
	//		}
	//		if (curve1T2 && curve2T1)
	//		{
	//			//あり得る？
	//			LOG_ERROR(L"curve1T2 && curve2T1");
	//		}
	//	}
	//	
	//	return lengthSq1 < lengthSq2 ? lower2 : lower1;
	//}

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