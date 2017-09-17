#pragma once
#include <queue>
#include <Siv3D.hpp> // August 2016 v2
#include "include/AnchorPoint.hpp"
#include "include/BoundingRect.hpp"
#include "include/BezierCurve.hpp"
#include "include/CurveSegment.hpp"
#include "include/PolygonSubtractor.hpp"

#include "include/clipper/clipper.hpp"

int64 profilingTime1;
int64 profilingTime2;
int64 profilingTime3;
int64 profilingTime4;

//#define DEBUG_OUTPUT_IMAGE

//std::vector<std::vector<Vec2>> debugPoss;
//std::vector<CurveSegment> debugSegments;

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

inline void TestSave(const std::vector<ClipperLib::Path>& paths, const String& filename)
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

inline void TestSave(const std::vector<Vec2>& path, Image& image, bool loop, Color color = Palette::Red)
{
	if (loop)
	{
		for (size_t i = 0; i < path.size(); ++i)
		{
			Line(path[i], path[(i + 1) % path.size()]).writeArrow(image, 1.0, { 5.0,5.0 }, color);
		}
	}
	else
	{
		for (size_t i = 0; i + 1 < path.size(); ++i)
		{
			Line(path[i], path[i + 1]).writeArrow(image, 1.0, { 5.0,5.0 }, color);
		}
	}
}

inline void TestSave(const std::vector<std::vector<Vec2>>& paths, Image& image)
{
	int count = 0;
	for (const auto& path : paths)
	{
		TestSave(path, image, false, HSV(30 * count, 1, 1));
		++count;
	}
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

bool IsClockWise(const std::vector<Vec2>& closedPath)
{
	double sum = 0;

	for (int i = 0; i < closedPath.size(); ++i)
	{
		const auto& p1 = closedPath[i];
		const auto& p2 = closedPath[(i + 1) % closedPath.size()];

		sum += (p2.x - p1.x)*(p2.y + p1.y);
	}

	return sum < 0.0;
}

void CheckIntersection(const String& filename, const ClipperLib::Path & polygonA, const ClipperLib::Path & polygonB)
{
	const auto relationToString = [](ClippingRelation relation)->String
	{
		switch (relation)
		{
		case AdjacentInner:
			return L"AdjacentInner";
		case AdjacentOuter:
			return L"AdjacentOuter";
		case Contain:
			return L"Contain";
		case Distant:
			return L"Distant";
		default:
			return L"else";
		}
	};

	Font font(20);

	std::vector<Vec2> psA, psB;

	for (const auto& point : polygonA)
	{
		psA.push_back(ClipVertex(point).m_pos);
	}
	for (const auto& point : polygonB)
	{
		psB.push_back(ClipVertex(point).m_pos);
	}

	Image image(Window::Size(), Palette::White);

	LineString(psA).write(image, 2.0, Palette::Blue);
	LineString(psB).write(image, 2.0, Palette::Red);

	font(relationToString(CalcRetation(polygonA, polygonB))).write(image);

	image.savePNG(filename);
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

	BezierCurvePath(const Vec2& initialPos)
	{
		m_anchorPoints.emplace_back(initialPos, initialPos, initialPos);
		m_isClosed = false;
		m_isGrabbing = true;

		constractCurveSegments();
		calcIntersections();
	}

	bool update()
	{
		bool anchorPointsEditing = false;

		if (!m_isGrabbing)
		{	
			for (auto i : step(m_anchorPoints.size()))
			{
				if (m_anchorPoints[i].updateControlPoint(AnchorPointRadius() * 2))
				{
					anchorPointsEditing = true;
					break;
				}
			}
		}

		if (Input::MouseL.clicked)
		{
			if (2 <= m_anchorPoints.size() && Circle(m_anchorPoints.front().anchorPoint(), AnchorPointRadius()).mouseOver)
			{
				m_isClosed = true;
			}

			if (!anchorPointsEditing && !m_isClosed)
			{
				m_anchorPoints.emplace_back(Mouse::PosF());

				m_isGrabbing = true;
			}
		}

		if (m_isGrabbing && Input::MouseL.pressed)
		{
			if (!m_isClosed)
			{
				m_anchorPoints.back().setSymmetricallyB(Mouse::PosF());
			}
		}

		if (Input::MouseL.released)
		{
			m_isGrabbing = false;
		}

		constractCurveSegments();
		//calcIntersections();

		return anchorPointsEditing || m_isGrabbing;
	}

	void draw(int alpha = 255)const
	{
		//if (isActive || Input::KeyD.pressed)
		{
			for (const auto& p : m_anchorPoints)
			{
				p.draw(AnchorPointRadius(), alpha);
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

			//LineString(pp).draw(Color(isActive ? Palette::Yellow : Palette::Lime, 64), m_isClosed);
			//LineString(pp).draw(Color(isActive ? Palette::Yellow : Palette::Lightgrey, 128), m_isClosed);
			
			LineString(pp).draw(Color(51, 51, 51, alpha), m_isClosed);
			
			/*if (isActive)
			{
				for (const auto& curveSegment : m_curveSegments)
				{
					curveSegment.draw();
				}
			}*/
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
		return 7.5;
	}

	std::vector<AnchorPoint> m_anchorPoints;
	std::vector<CurveSegment> m_curveSegments;

	bool m_isClosed = false;
	bool m_isGrabbing = false;
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

		Window::SetTitle(L"Depth: ", m_drawDepth, L", Poly: ", m_drawIndex, L"/", m_loops.size(), L", ", Mouse::Pos());

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

	void fixSegments()
	{
		/*
		ただ近づけるだけでは上下に割れてくっ付かない可能性があるためもっと分けるべき？
		*/
		//prevSegment -> curveSegment -> postSegment
		auto fixSegmentStep = [](CurveSegment& prevSegment, CurveSegment& curveSegment, CurveSegment& postSegment)
		{
			const auto currentRange = curveSegment.range();
			const auto prevRange = prevSegment.range();
			const auto postRange = postSegment.range();

			const Vec2 prevEnd = prevSegment.curve().get(prevRange.second);
			const Vec2 currentBegin = curveSegment.curve().get(currentRange.first);
			const Vec2 currentEnd = curveSegment.curve().get(currentRange.second);
			const Vec2 postBegin = postSegment.curve().get(postRange.first);

			const double distPrevSq = (prevEnd - currentBegin).lengthSq();
			const double distPostSq = (postBegin - currentEnd).lengthSq();

			if (1.0 < distPrevSq)
			{
				const Vec2 mid = (prevEnd + currentBegin)*0.5;

				const double candidatePrevEndT = prevSegment.curve().closestPoint(mid);
				const Vec2 candidateNewPrevEnd = prevSegment.curve().get(candidatePrevEndT);
				const double candidateNewDistPrevSq = (candidateNewPrevEnd - currentBegin).lengthSq();

				Vec2 newPrevEnd = prevEnd;
				double newDistPrevSq = distPrevSq;

				if(candidateNewDistPrevSq < distPrevSq)
				{
					prevSegment.setEnd(candidatePrevEndT);
					newPrevEnd = candidateNewPrevEnd;
					newDistPrevSq = candidateNewDistPrevSq;
				}

				const double candidateCurrentBeginT = curveSegment.curve().closestPoint(mid);
				const Vec2 candidateNewCurrentBegin = curveSegment.curve().get(candidateCurrentBeginT);

				if ((newPrevEnd - candidateNewCurrentBegin).lengthSq() < newDistPrevSq)
				{
					curveSegment.setBegin(candidateCurrentBeginT);
				}
			}

			if (1.0 < distPostSq)
			{
				const Vec2 mid = (postBegin + currentBegin)*0.5;

				const double candidatePostBeginT = postSegment.curve().closestPoint(mid);
				const Vec2 candidateNewPostBegin = postSegment.curve().get(candidatePostBeginT);
				const double candidateNewDistPostSq = (candidateNewPostBegin - currentEnd).lengthSq();

				Vec2 newPostBegin = postBegin;
				double newDistPostSq = distPostSq;

				if (candidateNewDistPostSq < distPostSq)
				{
					postSegment.setBegin(candidatePostBeginT);
					newPostBegin = candidateNewPostBegin;
					newDistPostSq = candidateNewDistPostSq;
				}

				const double candidateCurrentEndT = curveSegment.curve().closestPoint(mid);
				const Vec2 candidateNewCurrentEnd = curveSegment.curve().get(candidateCurrentEndT);

				if ((newPostBegin - candidateNewCurrentEnd).lengthSq() < newDistPostSq)
				{
					curveSegment.setEnd(candidateCurrentEndT);
				}
			}
		};
		
		for (auto& hole : m_loopHoles)
		{
			for (size_t i = 0; i < hole.size(); ++i)
			{
				fixSegmentStep(hole[i].first, hole[(i + 1) % hole.size()].first, hole[(i + 2) % hole.size()].first);
			}
		}

		for (auto& loop : m_loops)
		{
			for (size_t i = 0; i < loop.size(); ++i)
			{
				fixSegmentStep(loop[i].first, loop[(i + 1) % loop.size()].first, loop[(i + 2) % loop.size()].first);
			}
		}
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

	void insertLoop(size_t loopIndex, const std::weak_ptr<LineStringTree> pTreeWeak)
	{
		if (const auto pTree = pTreeWeak.lock())
		{
			LOG_DEBUG(L"Before Insertion Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());

			m_loops.insert(m_loops.begin() + loopIndex, pTree->m_loops.begin(), pTree->m_loops.end());
			m_lines.insert(m_lines.begin() + loopIndex, pTree->m_lines.begin(), pTree->m_lines.end());
			
			LOG_DEBUG(L"After Insertion Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());
		}
	}

	void eraseHole(size_t holeIndex)
	{
		m_lineHoles.erase(m_lineHoles.begin() + holeIndex);
		m_loopHoles.erase(m_loopHoles.begin() + holeIndex);
	}

	/*
	全体(getLoopPath + getHolePath): 16ms
	*/
	ClipperLib::Path getLoopPath(size_t loopIndex, size_t& zIndex, double interval, double permissibleError = 1.0)const
	{
		StopwatchMicrosec watch(true);

		const auto& loopCurve = getLoopCurve(loopIndex);

		ClipperLib::Path result;

		for (int segment = 0; segment < loopCurve.size(); ++segment)
		{
			const auto range = loopCurve[segment].first.range();
			LOG_DEBUG(L"output path: ", range.first, L" -> ", range.second);
			loopCurve[segment].first.getSpecificPathHighPrecision(result, zIndex + segment, interval, permissibleError);
		}

		zIndex += loopCurve.size();

		//profilingTime += watch.us();

		return result;
	}

	void debugLoopPath(const String& filenameBase, size_t loopIndex, size_t zIndex, double interval, double permissibleError = 1.0)const
	{
		{
			Image image(640, 480, Palette::White);

			const auto& loopCurve = getLoopCurve(loopIndex);

			std::vector<Vec2> result;

			for (int segment = 0; segment < loopCurve.size(); ++segment)
			{
				loopCurve[segment].first.getSpecificPathHighPrecision(result, interval, permissibleError);
			}

			LineString(result).write(image, 1.0, Palette::Cyan, true);

			image.savePNG(Format(filenameBase, L"_loop.png"));
		}

		{
			Image image(640, 480, Palette::White);

			const auto& loopLines = getLoopLines(loopIndex);
			
			LineString(loopLines).write(image, 1.0, Palette::Lime, true);

			image.savePNG(Format(filenameBase, L"_curve.png"));
		}

		{
			Image image(640, 480, Palette::White);

			size_t index = 1;
			const auto& loopPath = getLoopPath(loopIndex, index, interval, permissibleError);

			std::vector<Vec2> path;
			for (const auto& p : loopPath)
			{
				path.push_back(ClipVertex(p).m_pos);
			}

			LineString(path).write(image, 1.0, Palette::Orange, true);

			image.savePNG(Format(filenameBase, L"_path.png"));
		}
	}

	ClipperLib::Path getHolePath(size_t holeIndex, size_t& zIndex, double interval, double permissibleError = 1.0)const
	{
		StopwatchMicrosec watch(true);

		const auto& holeCurve = getHoleCurve(holeIndex);

		ClipperLib::Path result;

		for (int segment = 0; segment < holeCurve.size(); ++segment)
		{
			holeCurve[segment].first.getSpecificPathHighPrecision(result, zIndex + segment, interval, permissibleError);
		}

		zIndex += holeCurve.size();

		//profilingTime += watch.us();

		return result;
	}

	void append(const std::weak_ptr<LineStringTree> pTreeWeak)
	{
		if (const auto pTree = pTreeWeak.lock())
		{
			LOG_DEBUG(L"Before Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());

			m_loops.insert(m_loops.end(), pTree->m_loops.begin(), pTree->m_loops.end());
			m_loopHoles.insert(m_loopHoles.end(), pTree->m_loopHoles.begin(), pTree->m_loopHoles.end());
			m_contours.insert(m_contours.end(), pTree->m_contours.begin(), pTree->m_contours.end());
			m_lines.insert(m_lines.end(), pTree->m_lines.begin(), pTree->m_lines.end());
			m_lineHoles.insert(m_lineHoles.end(), pTree->m_lineHoles.begin(), pTree->m_lineHoles.end());
			m_lineColors.insert(m_lineColors.end(), pTree->m_lineColors.begin(), pTree->m_lineColors.end());
			m_lineHoleColors.insert(m_lineHoleColors.end(), pTree->m_lineHoleColors.begin(), pTree->m_lineHoleColors.end());

			LOG_DEBUG(L"After Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());
		}
	}

	void insert(const std::weak_ptr<LineStringTree> pTreeWeak, size_t pos)
	{
		if (const auto pTree = pTreeWeak.lock())
		{
			LOG_DEBUG(L"Before Insertion Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());

			m_loops.insert(m_loops.begin() + pos, pTree->m_loops.begin(), pTree->m_loops.end());
			m_loopHoles.insert(m_loopHoles.begin() + pos, pTree->m_loopHoles.begin(), pTree->m_loopHoles.end());
			m_contours.insert(m_contours.begin() + pos, pTree->m_contours.begin(), pTree->m_contours.end());
			m_lines.insert(m_lines.begin() + pos, pTree->m_lines.begin(), pTree->m_lines.end());
			m_lineHoles.insert(m_lineHoles.begin() + pos, pTree->m_lineHoles.begin(), pTree->m_lineHoles.end());
			m_lineColors.insert(m_lineColors.begin() + pos, pTree->m_lineColors.begin(), pTree->m_lineColors.end());
			m_lineHoleColors.insert(m_lineHoleColors.begin() + pos, pTree->m_lineHoleColors.begin(), pTree->m_lineHoleColors.end());

			LOG_DEBUG(L"After Insertion Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());
		}
	}

	void logger(const String& tag)
	{
		LOG_DEBUG(tag, L" Holes: ", m_lineHoles.size(), L", ", m_loopHoles.size(), L", Loops: ", m_lines.size(), L", ", m_loops.size());
	}

	void dump(const String& filename, const String& filename2, const Size& imageSize = { 640,480 })
	{
		Image image(imageSize, Palette::White);
		Image image2(imageSize, Palette::White);

		LOG_DEBUG(L"dump to \"", filename, L"\"");
		dumpImpl(image, image2);

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
		image2.savePNG(filename2);
	}

	void dumpPoly(const String& filename, const Size& imageSize = { 640,480 })
	{
		Image image(imageSize, Palette::White);

		dumpImplPoly(image);

		image.savePNG(filename);
	}

	void dumpHoles(const String& filename, const Size& imageSize = { 640,480 })
	{
		Image image(imageSize, Palette::White);

		LOG_DEBUG(L"dump to \"", filename, L"\"");
		dumpImplHoles(image);

		image.savePNG(filename);
	}

	void debugLog()const
	{
		//m_loops.front().front().;

	}

private:

	void dumpImpl(Image& image, Image& image2)
	{
		//for (int i = 0; i < numOfLoops(); ++i)
		//{
		//	Polygon(getLoopLines(i)).write(image, Color(Palette::Cyan, 64));
		//}

		//for (int i = 0; i < numOfHoles(); ++i)
		//{
		//	//const auto lines = getHoleLines(i);
		//	LineString(getHoleLines(i)).write(image, 1.0, Palette::Red, true);
		//}

		const auto getColor = [&](bool isHole) {return isHole ? Palette::Red : Palette::Cyan; };

		//LOG(L"L(", __LINE__, L"): ", m_loops.size());
		//for (auto i : step(m_loops.size()))
		//{
		//	LOG_DEBUG(L"L(", __LINE__, L"): ", m_loops[i].size());
		//	for (const auto& curveSegmentPair : m_loops[i])
		//	{
		//		const auto& curveSegment = curveSegmentPair.first;

		//		//LOG(L"L(", __LINE__, L"): (first, last) = ", Vec2(curveSegment.range().first, curveSegment.range().second));
		//		//LOG(L"L(", __LINE__, L"): (first, last) = ", curveSegment.curve().p0, L", ", curveSegment.curve().p1, L", ", curveSegment.curve().p2, L", ", curveSegment.curve().p3);

		//		curveSegment.curve().writeArrow(image, 4, Palette::Purple, curveSegment.range().first, curveSegment.range().second);
		//		//curveSegment.curve().write(image, 8, Palette::Cyan, curveSegment.range().first, curveSegment.range().second);
		//	}
		//}

		/*for (auto i : step(m_loops.size()))
		{
			LOG_DEBUG(L"L(", __LINE__, L"): ", m_loops[i].size());
			for (const auto& curveSegmentPair : m_loops[i])
			{
				const auto& curveSegment = curveSegmentPair.first;

				std::vector<Vec2> points;
				curveSegment.getSpecificPathHighPrecision(points, 10.0,0.2);
				for (int p = 0; p+1 < points.size(); ++p)
				{
					Line(points[p], points[p + 1]).writeArrow(image, 1.0, { 5,5 }, Palette::Purple);
				}
			}
		}*/

		for (int i = 0; i < numOfLoops(); ++i)
		{
			//const auto lines = getHoleLines(i);
			LineString(getLoopLines(i)).write(image, 1.0, RandomColor(), true);
		}

		for (int i = 0; i < numOfLoops(); ++i)
		{
			//const auto lines = getHoleLines(i);
			const auto& segments = getLoopCurve(i);
			const Color color = RandomColor();
			for (const auto& segment : segments)
			{
				const auto range = segment.first.range();
				segment.first.curve().write(image2, 30, color, range.first, range.second);
			}
		}

		/*for (auto p : m_childs)
		{
			p->dumpImpl(image);
		}*/

	}

	void dumpImplPoly(Image& image)
	{
		for (int i = 0; i < numOfLoops(); ++i)
		{
			Polygon(getLoopLines(i)).write(image, HSV(15.0*i, 1, 1).toColor(64));
		}
	}

	void dumpImplHoles(Image& image)
	{
		for (int i = 0; i < numOfHoles(); ++i)
		{
			//LineString(getHoleLines(i)).write(image, 1.0, RandomColor(), true);
			Polygon(getHoleLines(i)).write(image, RandomColor().setAlpha(64));
		}

		/*for (auto p : m_childs)
		{
			p->dumpImplHoles(image);
		}*/

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
		if (!Input::KeyShift.pressed)
		{
			/*if (Input::KeyControl.pressed)
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
			}*/
			for (const auto& loop : m_lines)
			{
				Polygon(loop).draw(Color(230, 230, 245));
				//Polygon(loop).draw(Color(237, 237, 245));
				//Polygon(loop).draw(Color(Palette::Cyan, 64));
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
			LOG(L"======================================================");

			profilingTime1 = 0;
			profilingTime2 = 0;
			profilingTime3 = 0;
			profilingTime4 = 0;

			StopwatchMicrosec watch(true);
			//TimeProfiler profiler;
			//profiler.begin(L"全体");

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

				StopwatchMicrosec watch2(true);

				ClipperLib::Clipper clipper;
				clipper.ZFillFunction(ZFillFunc);

				clipper.AddPath(path1, ClipperLib::PolyType::ptSubject, true);
				clipper.AddPath(path2, ClipperLib::PolyType::ptClip, true);

				//ClipperLib::PolyTree resultPolyTree;
				ClipperLib::Paths resultPaths;

				
				//profiler.begin(L"clipper.Execute");
				clipper.Execute(ClipperLib::ClipType::ctXor, resultPaths);
				//profiler.end();
				LOG(L"clipper.Execute: ", static_cast<double>(watch2.us()) / 1000.0, L"[ms]");
				//clipper.Execute(ClipperLib::ClipType::ctDifference, resultPaths);

				//profilingTime += watch2.us();

				//drawCurves = makeResultPath(resultPaths);
				//m_segmentInfos = makeResultPath(resultPaths);

				//m_pTree = makeResult(resultPolyTree);
				
				StopwatchMicrosec watch3(true);
				//profiler.begin(L"MakeResultPath");
				/*
				全体(MakeResultPath + PolygonSubtract2 + MakeResultPathWithoutDivision): 50ms / 180ms
				*/
				m_pTree = MakeResultPath(resultPaths, m_paths);
				//profiler.end();
				//profilingTime += watch3.us();
				LOG(L"MakeResultPath: ", static_cast<double>(watch3.us()) / 1000.0, L"[ms]");

				m_pTree->logger(L"After MakeResultPath");

#ifdef DEBUG_OUTPUT_IMAGE
				//m_pTree->dump(L"original_loops.png", L"original_curves.png");
				for (size_t ii = 0; ii < m_pTree->numOfLoops(); ++ii)
				{
					m_pTree->debugLoopPath(Format(L"original_path", ii, L"_test"), ii, 1, polygonizeInterval);
				}
#endif
				
				//return;
			}

			int processCount = 0;

			int checkCount = 0;

			const int numOfLoops = m_pTree->numOfLoops();

#ifdef DEBUG_OUTPUT_IMAGE
			/*
			for (int loopIndex = 0; loopIndex < numOfLoops; ++loopIndex)
			{
				size_t indexOffset = 1;
				auto pathLoop = m_pTree->getLoopPath(loopIndex, indexOffset, polygonizeInterval);

				TestSave2({ pathLoop }, Format(L"inputPath_", loopIndex, L".png"));
			}*/
#endif

			for (int loopIndex = 0; loopIndex < numOfLoops; ++loopIndex)
			{
#ifdef DEBUG_OUTPUT_IMAGE
				{
					for (int innerLoopIndex = 0; innerLoopIndex < numOfLoops; ++innerLoopIndex)
					{
						size_t innerIndexOffset = 1;
						auto innerPathLoop = m_pTree->getLoopPath(innerLoopIndex, innerIndexOffset, polygonizeInterval);
						auto innerPathLines = m_pTree->getLoopLines(innerLoopIndex);
						
						TestSave({ innerPathLoop }, Format(L"innerInputPath_", processCount, L"_", innerLoopIndex, L".png"));
						Image image(640, 480, Palette::White);
						TestSave(innerPathLines, image, true);
						image.savePNG(Format(L"innerInputPath_", processCount, L"_", innerLoopIndex, L"_lines.png"));

						if (processCount == 0 && innerLoopIndex == 2)
						{
							//m_pTree->debugLoopPath(Format(L"path_dump_", checkCount, L"_loop.png"), Format(L"path_dump_", checkCount, L"_curve.png"), loopIndex, 1, polygonizeInterval);
							m_pTree->debugLoopPath(Format(L"path_dump_", checkCount), loopIndex, 1, polygonizeInterval);
						}
					}
				}
#endif

				//profiler.begin(L"remove hole loop");
				
				Optional<size_t> leaveLaterIndex;

				for (size_t holeIndex = 0; m_pTree->hasAnyHole(); ++holeIndex)
				{
					///loopのパスはクリップされるたびに変わる可能性があるので毎回ツリーからロードする
					//const Polygon polygonLoop(m_pTree->getLoopLines(loopIndex));

					SegmentsHolder loopSegmentsHolder;

					StopwatchMicrosec watch4(true);

					const auto loopSegments = m_pTree->getLoopCurve(loopIndex);

					for (const auto& loopSegment : loopSegments)
					{
						loopSegmentsHolder.add(&loopSegment.first);
					}

					size_t indexOffset = 1;
					/*if (checkCount == 6)
					{
						m_pTree->debugLoopPath(Format(L"path_dump_", checkCount, L"_loop.png"), Format(L"path_dump_", checkCount, L"_curve.png"), loopIndex, indexOffset, polygonizeInterval);
					}*/
					auto pathLoop = m_pTree->getLoopPath(loopIndex, indexOffset, polygonizeInterval);
					///

					//profilingTime += watch4.us();

					if (m_pTree->numOfHoles() <= holeIndex)
					{
						//後回しにしたホールがあればもどってくる
						if (leaveLaterIndex)
						{
							holeIndex = 0;
						}
						//そうでなければこのループについては正常終了
						else
						{
							LOG_DEBUG(L"穴の除去完了");
							//profiler.end(); 
							LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
							
							break;
						}
					}

					if (leaveLaterIndex)
					{
						if (leaveLaterIndex.value() == holeIndex)
						{
							LOG_ERROR(L"L(", __LINE__, L"): 穴の除去に失敗");
							//profiler.end(); 
							LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
							break;
						}
					}

					auto holeVertices = m_pTree->getHoleLines(holeIndex);
					LOG_DEBUG(L"hole is ClockWise? : ", static_cast<int>(IsClockWise(holeVertices)));
					std::reverse(holeVertices.begin(), holeVertices.end());
					//const Polygon polygonHole(holeVertices);

					//1回のクリップ毎に、インデックスは振りなおす
					size_t currentIndexOffset = indexOffset;
					auto pathHole = m_pTree->getHolePath(holeIndex, currentIndexOffset, polygonizeInterval);

					LOG_DEBUG(L"<A");
					const auto relation = CalcRetation(pathLoop, pathHole);
					LOG_DEBUG(L"B>");

#ifdef DEBUG_OUTPUT_IMAGE
					CheckIntersection(Format(L"path_check_", checkCount++, L".png"), pathLoop, pathHole);
#endif

					//内側に接する場合のみ実際にクリッピングを行う
					if(relation == ClippingRelation::AdjacentInner)
					{
						LOG_DEBUG(L"穴の除去, Hole: ", m_pTree->numOfHoles(), L", Loop: ", m_pTree->numOfLoops());

						SegmentsHolder holeSegmentsHolder;

						LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);
						const auto holeSegments = m_pTree->getHoleCurve(holeIndex);
						for (const auto& holeSegment : holeSegments)
						{
							holeSegmentsHolder.add(&holeSegment.first);
						}
						LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);
						
						++processCount;

#ifdef DEBUG_OUTPUT_IMAGE
						TestSave(pathLoop, pathHole, Format(L"process_", processCount, L".png"));
#endif

						LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);
						StopwatchMicrosec watch5(true);
						//profiler.begin(L"PolygonSubtract2");
						/*
						全体(PolygonSubtract2): 35.885ms
						*/
						auto results = PolygonSubtract2(pathLoop, pathHole);
						//profilingTime3 += watch5.us();
						//profiler.end();
						LOG(L"PolygonSubtract2: ", static_cast<double>(watch5.us()) / 1000.0, L"[ms]");
						
						LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);

						//std::shared_ptr<LineStringTree> pTree2 = MakeResultPathWithoutDivision(results.first, std::vector<SegmentsHolder>({ holeSegmentsHolder, loopSegmentsHolder }));
						//std::shared_ptr<LineStringTree> pTree2 = MakeResultPathWithoutDivision(results.first, std::vector<SegmentsHolder>({ loopSegmentsHolder, holeSegmentsHolder }));

						StopwatchMicrosec watch6(true);
						//profiler.begin(L"MakeResultPathWithoutDivision");
						std::shared_ptr<LineStringTree> pTree2 = MakeResultPathWithoutDivision(results, std::vector<SegmentsHolder>({ loopSegmentsHolder, holeSegmentsHolder }));
						//profiler.end();
						//profilingTime += watch6.us();
						LOG(L"MakeResultPathWithoutDivision: ", static_cast<double>(watch6.us()) / 1000.0, L"[ms]");


						LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);

#ifdef DEBUG_OUTPUT_IMAGE
						pTree2->dump(Format(L"dump_process_", processCount, L"_loops.png"), Format(L"dump_process_", processCount, L"_curves.png"));
						LOG_DEBUG(L"dump complete");
						
						TestSave(results, Format(L"process_", processCount, L"_result.png"));
#endif

						/*for (size_t ii = 0; ii < pTree2->numOfLoops(); ++ii)
						{
							pTree2->debugLoopPath(Format(L"result_", processCount, L"_path", ii, L"_test"), ii, 1, polygonizeInterval);
						}*/
						
						if (pTree2->hasAnyHole())
						{
							LOG_ERROR(L"L(", __LINE__, L"): 穴の除去に失敗");
							//profiler.end(); 
							LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
							return;
						}
						else
						{
							m_pTree->eraseLoop(loopIndex);

							if (pTree2->numOfLoops() == 1)
							{
								//const Polygon newPolygonLoop(pTree2->getLoopLines(0));
								//LOG_DEBUG(L"Before erode area: ", polygonLoop.area(), L", After erode area: ", newPolygonLoop.area());
								//m_pTree->append(pTree2);
								//m_pTree->insert(pTree2, loopIndex);
								m_pTree->insertLoop(loopIndex, pTree2);
							}
							else
							{
								LOG_DEBUG(L"Loop was divided to ", pTree2->numOfLoops(), L" new loops");
								//m_pTree->append(pTree2);
								//m_pTree->insert(pTree2, loopIndex);
								m_pTree->insertLoop(loopIndex, pTree2);
							}

							//m_pTree->eraseLoop(loopIndex);
						}

						//クリッピングの適用により接するようになったかもしれない
						leaveLaterIndex = none;
						
						/*loopSegmentsHolder.read(debugSegments);
						holeSegmentsHolder.read(debugSegments);*/

						//デバッグ用
						//return;
					}
					//完全に包含する場合は後回しにする
					else if (relation == ClippingRelation::Contain)
					{
						if (!leaveLaterIndex)
						{
							leaveLaterIndex = holeIndex;
						}
					}
					else if (relation == ClippingRelation::Unknown)
					{
						LOG_ERROR(L"polygon relation was unknown");
						//profiler.end(); 
						LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
						return;
					}
					//無視できるケース（外側に接する or 完全に外側）
					//else
					//{

					//	//++loopIndex;
					//	//continue;
					//}

					LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
				}

				//m_pTree->eraseHole(0);

				//profiler.end();
			}

#ifdef DEBUG_OUTPUT_IMAGE
			m_pTree->dump(L"result_loops.png", L"result_curves.png");
#endif

			LOG(L"全体: ", static_cast<double>(watch.us()) / 1000.0, L"[ms]");
			
			LOG(L"profilingTime1: ", static_cast<double>(profilingTime1) / 1000.0, L"[ms]");
			LOG(L"profilingTime2: ", static_cast<double>(profilingTime2) / 1000.0, L"[ms]");
			LOG(L"profilingTime3: ", static_cast<double>(profilingTime3) / 1000.0, L"[ms]");
			LOG(L"profilingTime4: ", static_cast<double>(profilingTime4) / 1000.0, L"[ms]");
			LOG(L"全体（profilingTime）: ", static_cast<double>(profilingTime1 + profilingTime2 + profilingTime3 + profilingTime4) / 1000.0, L"[ms]");
			
			//profiler.end();
			
			m_pTree->dumpPoly(L"result_poly.png");
			m_pTree->dumpHoles(L"result_holes.png");


			/*
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
						LOG_DEBUG(L"穴の除去, Hole: ", m_pTree->numOfHoles(), L", Loop: ", m_pTree->numOfLoops());

						SegmentsHolder loopSegmentsHolder;

						const auto loopSegments = m_pTree->getLoopCurve(loopIndex);
						for (const auto& loopSegment : loopSegments)
						{
							loopSegmentsHolder.add(&loopSegment.first);
						}

						//1回のクリップ毎に、インデックスは振りなおす
						size_t currentIndexOffset = indexOffset;
						auto pathLoop = m_pTree->getLoopPath(loopIndex, currentIndexOffset, polygonizeInterval);

						auto results = PolygonSubtract(pathLoop, pathHole, currentIndexOffset);
						if(results.second)
						{
							const auto& additionalSegments = results.second.value().additionalSegments;
							LOG_DEBUG(L"additionalSegments: ", additionalSegments.size());

							for (const auto& segment : additionalSegments)
							{
								loopSegmentsHolder.add(&segment);
							}
						}

						std::shared_ptr<LineStringTree> pTree2 = MakeResultPathWithoutDivision(results.first, std::vector<SegmentsHolder>({ holeSegmentsHolder, loopSegmentsHolder }));

						++processCount;
						TestSave(pathLoop, pathHole, Format(L"process_", processCount, L".png"));
						TestSave2(results.first, Format(L"process_", processCount, L"_result.png"));

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
			*/
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
			//m_pTree->draw();
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

		Window::SetTitle(Mouse::Pos());

		//if (!debugSegments.empty())
		//{
		//	static int count = 0;

		//	if (Input::KeyLeft.clicked)
		//	{
		//		--count;
		//	}
		//	if (Input::KeyRight.clicked)
		//	{
		//		++count;
		//	}
		//	count = Clamp<int>(count, 0, debugSegments.size() - 1);

		//	const auto& curveSegment = debugSegments[count];
		//	//curveSegment.curve().drawArrow(8, HSV(15 * i, 1, 1), curveSegment.range().first, curveSegment.range().second);
		//	curveSegment.curve().drawArrow(8, Palette::Orange, curveSegment.range().first, curveSegment.range().second);

		//	Window::SetTitle(count + 1, L", ", Mouse::Pos());
		//}

		//for (const auto index : step(debugPoss.size()))
		//{
		//	const Color color = HSV(index*17.0);
		//	const auto& debugLoop = debugPoss[index];
		//	for (size_t i = 0; i < debugLoop.size(); ++i)
		//	{
		//		Line(debugLoop[i], debugLoop[(i + 1) % debugLoop.size()]).drawArrow(1.0, {5.0,5.0}, color);
		//		//Circle(debugLoop[i], 5).drawFrame(1.0, 0.0);
		//	}
		//}
		
		
		
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

	static std::shared_ptr<LineStringTree> Clip(const BezierCurvePath& subject, const BezierCurvePath& clip, ClipperLib::ClipType clipType)
	{
		StopwatchMicrosec watch(true);

		const double polygonizeInterval = 5.0;
		size_t indexOffset = 1;

		auto subjectPath = subject.getPathHighPrecision(indexOffset, polygonizeInterval);
		indexOffset += subject.numOfSegments();

		auto clipPath = clip.getPathHighPrecision(indexOffset, polygonizeInterval);
		indexOffset += clip.numOfSegments();


		ClipperLib::Clipper clipper;
		clipper.ZFillFunction(ZFillFunc);

		clipper.AddPath(subjectPath, ClipperLib::PolyType::ptSubject, true);
		clipper.AddPath(clipPath, ClipperLib::PolyType::ptClip, true);

		ClipperLib::Paths resultPaths;

		clipper.Execute(clipType, resultPaths);


		SegmentsHolder segmentsHolder1;
		for (auto i : step(subject.numOfSegments()))
		{
			segmentsHolder1.add(&subject.segment(i));
		}

		SegmentsHolder segmentsHolder2;
		for (auto i : step(clip.numOfSegments()))
		{
			segmentsHolder2.add(&clip.segment(i));
		}

		auto resultTree = MakeResultPath(resultPaths, std::vector<SegmentsHolder>({ segmentsHolder1, segmentsHolder2 }));


		int processCount = 0;

		int checkCount = 0;

		const int numOfLoops = resultTree->numOfLoops();

		for (int loopIndex = 0; loopIndex < numOfLoops; ++loopIndex)
		{
			Optional<size_t> leaveLaterIndex;

			for (size_t holeIndex = 0; resultTree->hasAnyHole(); ++holeIndex)
			{
				SegmentsHolder loopSegmentsHolder;

				StopwatchMicrosec watch4(true);

				const auto loopSegments = resultTree->getLoopCurve(loopIndex);

				for (const auto& loopSegment : loopSegments)
				{
					loopSegmentsHolder.add(&loopSegment.first);
				}

				size_t indexOffset = 1;
				auto pathLoop = resultTree->getLoopPath(loopIndex, indexOffset, polygonizeInterval);

				if (resultTree->numOfHoles() <= holeIndex)
				{
					//後回しにしたホールがあればもどってくる
					if (leaveLaterIndex)
					{
						holeIndex = 0;
					}
					//そうでなければこのループについては正常終了
					else
					{
						LOG_DEBUG(L"穴の除去完了");
						LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
						break;
					}
				}

				if (leaveLaterIndex)
				{
					if (leaveLaterIndex.value() == holeIndex)
					{
						LOG_ERROR(L"L(", __LINE__, L"): 穴の除去に失敗");
						LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
						break;
					}
				}

				auto holeVertices = resultTree->getHoleLines(holeIndex);
				LOG_DEBUG(L"hole is ClockWise? : ", static_cast<int>(IsClockWise(holeVertices)));
				std::reverse(holeVertices.begin(), holeVertices.end());

				//1回のクリップ毎に、インデックスは振りなおす
				size_t currentIndexOffset = indexOffset;
				auto pathHole = resultTree->getHolePath(holeIndex, currentIndexOffset, polygonizeInterval);

				const auto relation = CalcRetation(pathLoop, pathHole);

				//内側に接する場合のみ実際にクリッピングを行う
				if (relation == ClippingRelation::AdjacentInner)
				{
					LOG_DEBUG(L"穴の除去, Hole: ", resultTree->numOfHoles(), L", Loop: ", resultTree->numOfLoops());

					SegmentsHolder holeSegmentsHolder;

					LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);
					const auto holeSegments = resultTree->getHoleCurve(holeIndex);
					for (const auto& holeSegment : holeSegments)
					{
						holeSegmentsHolder.add(&holeSegment.first);
					}
					LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);

					++processCount;
					
					LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);
					StopwatchMicrosec watch5(true);

					auto results = PolygonSubtract2(pathLoop, pathHole);

					LOG(L"PolygonSubtract2: ", static_cast<double>(watch5.us()) / 1000.0, L"[ms]");

					LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);

					StopwatchMicrosec watch6(true);
					std::shared_ptr<LineStringTree> pTree2 = MakeResultPathWithoutDivision(results, std::vector<SegmentsHolder>({ loopSegmentsHolder, holeSegmentsHolder }));

					LOG(L"MakeResultPathWithoutDivision: ", static_cast<double>(watch6.us()) / 1000.0, L"[ms]");

					LOG_DEBUG(__FUNCTIONW__, L": ", __LINE__);

					if (pTree2->hasAnyHole())
					{
						LOG_ERROR(L"L(", __LINE__, L"): 穴の除去に失敗");
						LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
						return nullptr;
					}
					else
					{
						resultTree->eraseLoop(loopIndex);

						if (pTree2->numOfLoops() != 1)
						{
							LOG_DEBUG(L"Loop was divided to ", pTree2->numOfLoops(), L" new loops");
						}

						resultTree->insertLoop(loopIndex, pTree2);
					}

					//クリッピングの適用により接するようになったかもしれないので
					leaveLaterIndex = none;
				}
				//完全に包含する場合は後回しにする
				else if (relation == ClippingRelation::Contain)
				{
					if (!leaveLaterIndex)
					{
						leaveLaterIndex = holeIndex;
					}
				}
				else if (relation == ClippingRelation::Unknown)
				{
					LOG_ERROR(L"polygon relation was unknown");
					LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
					return nullptr;
				}
				//無視できるケース（外側に接する or 完全に外側）

				LOG(L"remove hole loop: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");
			}
		}

		return resultTree;
	}

private:

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

		void read(std::vector<CurveSegment>& output)const
		{
			for (auto p : m_curveSegmentPtrs)
			{
				output.push_back(*p);
			}
		}

	private:

		std::vector<const CurveSegment*> m_curveSegmentPtrs;
	};

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
	
	/*template<class PathHolderType>
	static Optional<BezierCurve> GetSegmentCurve(int zIndex, const PathHolderType& originalPaths)
	{
		if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
		{
			return originalPaths[indicesOpt.value().first].segment(indicesOpt.value().second).curve();
		}

		return none;
	}*/
	template<class PathHolderType>
	static Optional<CurveSegment> GetSegmentCurve(int zIndex, const PathHolderType& originalPaths)
	{
		if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
		{
			return originalPaths[indicesOpt.value().first].segment(indicesOpt.value().second);
		}

		return none;
	}

	/*
	全体(profilingTime): 10ms
	*/
	template<class PathHolderType>
	static int CombineZIndex(const ClipVertex& p1, ClipVertex& p2, const PathHolderType& originalPaths,bool debugFlag=false)
	{
		//StopwatchMicrosec watch(true);

		const int lower1 = static_cast<int>(p1.m_Z & INT32_MAX);
		const int upper1 = static_cast<int>(p1.m_Z >> 32);
		const int lower2 = static_cast<int>(p2.m_Z & INT32_MAX);
		const int upper2 = static_cast<int>(p2.m_Z >> 32);

		LOG_DEBUG(L"CombineZIndex(", Vec4(p1.m_pos, upper1, lower1), L", ", Vec4(p2.m_pos, upper2, lower2), L")");

		//連続している（普通のケース）
		if (lower1 == lower2)
		{
			//profilingTime += watch.us();
			return lower1;
		}
		//あり得る？
		else if (upper1 != 0 && upper1 == upper2)
		{
			//profilingTime += watch.us();
			LOG_ERROR(L"L(", __LINE__, L"): upper1 != 0 && upper1 == upper2");
			return upper1;
		}
		//片方のupperのみ存在する場合：片方の端点は交点なので二つインデックスが存在する。
		//このような場合は交点ではない方のインデックスを取ってくればよい
		else if (upper1 == 0 && upper2 != 0)
		{
			//profilingTime += watch.us();
			return lower1;
		}
		else if (upper2 == 0 && upper1 != 0)
		{
			//profilingTime += watch.us();
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
		Optional<CurveSegment> curve1Opt, curve2Opt;

		const double error = 3.0;

		if (curve1Opt = GetSegmentCurve(lower1, originalPaths))
		{
			const auto& curve1 = curve1Opt.value();
			curve1T1 = curve1.closestPointOpt(p1.m_pos, error);
			curve1T2 = curve1.closestPointOpt(p2.m_pos, error);

			//if (lower1 == 6 && lower2 == 7)
			if(debugFlag)
			{
				LOG_DEBUG(L"================================");
				LOG_DEBUG(L"Curve1: ", curve1.curve(0.0), L" -> ", curve1.curve(1.0));
				LOG_DEBUG(L"case p1: ", p1.m_pos, L" then ", curve1T1);
				LOG_DEBUG(L"case p2: ", p2.m_pos, L" then ", curve1T2);
			}

			if (curve1T1 && curve1T2)
			{
				lengthSq1 = Line(curve1.curve(curve1T1.value()), curve1.curve(curve1T2.value())).lengthSq();
			}
		}

		if (curve2Opt = GetSegmentCurve(lower2, originalPaths))
		{
			const auto& curve2 = curve2Opt.value();
			curve2T1 = curve2.closestPointOpt(p1.m_pos, error);
			curve2T2 = curve2.closestPointOpt(p2.m_pos, error);

			//if (lower1 == 6 && lower2 == 7)
			if (debugFlag)
			{
				LOG_DEBUG(L"Curve2: ", curve2.curve(0.0), L" -> ", curve2.curve(1.0));
				LOG_DEBUG(L"case p1: ", p1.m_pos, L" then ", curve2T1);
				LOG_DEBUG(L"case p2: ", p2.m_pos, L" then ", curve2T2);
				LOG_DEBUG(L"--------------------------------");
			}

			if (curve2T1 && curve2T2)
			{
				lengthSq2 = Line(curve2.curve(curve2T1.value()), curve2.curve(curve2T2.value())).lengthSq();
			}
		}
		
		//p1とp2の間に連続している曲線の境界点が存在する場合、分割点を移動する必要がある
		if (lengthSq1 == 0 && lengthSq2 == 0)
		{
			if (curve1T1 && curve2T2)
			{
				/*if (debugFlag)
				{
					debugSegments.push_back(CurveSegment(curve1Opt.value(), 0, 1));
					debugSegments.push_back(CurveSegment(curve2Opt.value(), 0, 1));
				}*/
				/*
				//この二点が同じ点を指すはず
				curve1T2 = curve1Opt.value().closestPoint(p2.m_pos);
				curve2T1 = curve2Opt.value().closestPoint(p1.m_pos);
				*/
				//const Vec2 moveTo = curve2Opt.value().get(curve2Opt.value().closestPoint(p1.m_pos));
				const Vec2 moveTo = curve2Opt.value().curve().get(curve2Opt.value().curve().closestPoint(p1.m_pos));
				LOG_DEBUG(L"Path point ", p2.m_pos, L" is moved to ", moveTo);
				p2.m_pos = moveTo;
				p2.m_Z = lower2;

				//profilingTime += watch.us();
				return lower1;
			}
			if (curve1T2 && curve2T1)
			{
				//あり得る？
				LOG_ERROR(L"L(", __LINE__, L"): curve1T2 && curve2T1");
			}
		}

		//profilingTime += watch.us();
		LOG_DEBUG(L"Path length lengthSq1 < lengthSq2 = ", lengthSq1 < lengthSq2);
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

		StopwatchMicrosec wholeWatch(true);

		for (const auto& contour : resultPaths)
		{
			StopwatchMicrosec watch5(true);
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
			
			LOG(L"____path to polygons: ", static_cast<double>(watch5.us()) / 1000.0, L"[ms]");

			StopwatchMicrosec watch4(true);

			using Curves = std::vector<ClipVertex>;
			std::vector<Curves> loopCurves;

			for (const auto& loop : loops)
			{
				Curves curves;
				std::vector<Vec2> debugLoop;
				for (const auto& point : loop)
				{
					if (curves.empty() || curves.back().m_Z != point.m_Z)
					{
						curves.push_back(point);
						debugLoop.push_back(point.m_pos);
					}
				}
				//debugPoss.push_back(debugLoop);
				loopCurves.push_back(curves);
			}
			
			int loopCount = 0;

			std::vector<std::vector<std::pair<int, Vec2>>> segmentStartss;
			for (auto& loopCurve : loopCurves)
			{
				const auto color = RandomColor().setAlpha(64);

				segmentStartss.emplace_back();
				auto& segmentStarts = segmentStartss.back();

				for (auto i : step(loopCurve.size()))
				{
					const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], originalPaths);

					const Vec2 startPos = loopCurve[i].m_pos;
					const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

					segmentStarts.emplace_back(zIndex, startPos);
				}
			}
			
			std::vector<std::vector<std::pair<int, Vec2>>> uniqueSegmentStartss;
			for (auto& segmentStarts : segmentStartss)
			{
				uniqueSegmentStartss.emplace_back();
				auto& uniqueSegmentStarts = uniqueSegmentStartss.back();

				if (segmentStarts.empty())
				{
					continue;
				}
				
				uniqueSegmentStarts.push_back(segmentStarts.front());
				
				for (const auto& segmentStart : segmentStarts)
				{
					auto& currentUniqueSegmentStart = uniqueSegmentStarts.back();

					//同じセグメントであれば、終端を更新する
					//異なるセグメントならば追加する
					if (currentUniqueSegmentStart.first != segmentStart.first)
					{
						uniqueSegmentStarts.push_back(segmentStart);
					}
				}
			}

			LOG(L"____loop loopCurves: ", static_cast<double>(watch4.us()) / 1000.0, L"[ms]");

			int imageSaveCount = 0;

			StopwatchMicrosec watch3(true);
			for (const auto& segmentStarts : uniqueSegmentStartss)
			{
				pTree->nextLoop(isHoles[loopCount]);

				//Image image(640, 480, Palette::White);

				std::vector<Vec2> debugLoop;

				//LOG_DEBUG(L"Curve: ");
				for (auto i : step(segmentStarts.size()))
				{
					const auto& segmentStart = segmentStarts[i];

					const int zIndex = segmentStarts[i].first;

					const Vec2 startPos = segmentStarts[i].second;
					const Vec2 endPos = segmentStarts[(i + 1) % segmentStarts.size()].second;

					debugLoop.push_back(startPos);

					//LOG_DEBUG(L"____Pos:", startPos, L", index: ", zIndex);

					if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
					{
						const auto& indices = indicesOpt.value();

						const auto curve = originalPaths[indices.first].segment(indices.second).curve();

						const double startT = curve.closestPoint(startPos);
						const double endT = curve.closestPoint(endPos);

						//LOG_DEBUG(L"____Result1:", startPos, L" -> ", curve(startT));
						//LOG_DEBUG(L"____Result2:", endPos, L" -> ", curve(endT));
						
						pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));

						//curve.write(image, 30, RandomColor().setAlpha(128), startT, endT);
					}
				}

				//debugPoss.push_back(debugLoop);

#ifdef DEBUG_OUTPUT_IMAGE
				//image.savePNG(Format(L"debug_", imageSaveCount++, L".png"));
#endif

				++loopCount;
			}
			
			LOG(L"____loop pTree->addSegment: ", static_cast<double>(watch3.us()) / 1000.0, L"[ms]");

			StopwatchMicrosec watch2(true);
			pTree->fixSegments();
			LOG(L"____pTree->fixSegments: ", static_cast<double>(watch2.us()) / 1000.0, L"[ms]");

			/*
			for (auto& loopCurve : loopCurves)
			{
				Image image(640, 480, Palette::White);

				pTree->nextLoop(isHoles[loopCount]);

				std::vector<Vec2> debugLoop;

				const auto color = RandomColor().setAlpha(64);

				LOG_DEBUG(L"Curve: ");
				for (auto i : step(loopCurve.size()))
				{
					const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], originalPaths);

					const Vec2 startPos = loopCurve[i].m_pos;
					const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

					LOG_DEBUG(L"____Pos:", startPos, L", index: ", zIndex);

					debugLoop.push_back(startPos);

					//m_segmentPoss.push_back(startPos);

					if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
					{
						const auto& indices = indicesOpt.value();

						const auto curve = originalPaths[indices.first].segment(indices.second).curve();

						const double startT = curve.closestPoint(startPos);
						const double endT = curve.closestPoint(endPos);

						pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));

						curve.write(image, 30, RandomColor().setAlpha(128), startT, endT);
					}
				}

				image.savePNG(Format(L"debug_", imageSaveCount++, L".png"));

				debugPoss.push_back(debugLoop);

				pTree->logger(Format(L"B(", loopCount, L")"));

				++loopCount;
			}
			*/

			/*image.savePNG(Format(L"debug_", imageSaveCount++, L".png"));*/
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

			const auto upper = [](ClipperLib::cInt z)
			{
				return static_cast<int>(z >> 32);
			};
			const auto lower = [](ClipperLib::cInt z)
			{
				return static_cast<int>(z&INT32_MAX);
			};

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
				std::vector<Vec2> debugLoop;
				for (const auto& point : loop)
				{
					LOG_DEBUG(L"point: ", Vec4(point.m_pos, upper(point.m_Z), lower(point.m_Z)));

					if (curves.empty() || curves.back().m_Z != point.m_Z)
					{
						curves.push_back(point);
						debugLoop.push_back(point.m_pos);
					}
				}
				//debugPoss.push_back(debugLoop);
				loopCurves.push_back(curves);
			}

			int loopCount = 0;

			LOG_DEBUG(L"After Combine: ");
			std::vector<std::vector<std::pair<int, Vec2>>> segmentStartss;
			for (auto& loopCurve : loopCurves)
			{
				const auto color = RandomColor().setAlpha(64);

				std::vector<Vec2> debugLoop;

				segmentStartss.emplace_back();
				auto& segmentStarts = segmentStartss.back();

				for (auto i : step(loopCurve.size()))
				{
					const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], originalPaths, true);

					const Vec2 startPos = loopCurve[i].m_pos;
					const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

					debugLoop.push_back(startPos);
					segmentStarts.emplace_back(zIndex, startPos);

					LOG_DEBUG(L"____point: ", Vec3(startPos, zIndex));
				}

				//debugPoss.push_back(debugLoop);
			}

			std::vector<std::vector<std::pair<int, Vec2>>> uniqueSegmentStartss;
			for (auto& segmentStarts : segmentStartss)
			{
				uniqueSegmentStartss.emplace_back();
				auto& uniqueSegmentStarts = uniqueSegmentStartss.back();

				if (segmentStarts.empty())
				{
					continue;
				}

				uniqueSegmentStarts.push_back(segmentStarts.front());

				for (const auto& segmentStart : segmentStarts)
				{
					auto& currentUniqueSegmentStart = uniqueSegmentStarts.back();

					//同じセグメントであれば無視する（終端を更新する）
					//異なるセグメントならば追加する
					if (currentUniqueSegmentStart.first != segmentStart.first)
					{
						uniqueSegmentStarts.push_back(segmentStart);
					}
				}
			}

			static int imageSaveCount = 0;

			for (const auto& segmentStarts : uniqueSegmentStartss)
			{
				pTree->nextLoop(isHoles[loopCount]);

				//Image image(640, 480, Palette::White);

				std::vector<Vec2> debugLoop;

				LOG_DEBUG(L"Curve: ");
				for (auto i : step(segmentStarts.size()))
				{
					const auto& segmentStart = segmentStarts[i];

					const int zIndex = segmentStarts[i].first;

					const Vec2 startPos = segmentStarts[i].second;
					const Vec2 endPos = segmentStarts[(i + 1) % segmentStarts.size()].second;

					debugLoop.push_back(startPos);

					LOG_DEBUG(L"____Pos:", startPos, L", index: ", zIndex);

					if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
					{
						const auto& indices = indicesOpt.value();

						const auto curve = originalPaths[indices.first].segment(indices.second).curve();

						const double startT = curve.closestPoint(startPos);
						const double endT = curve.closestPoint(endPos);

						LOG_DEBUG(L"____Result1:", startPos, L" -> ", curve(startT));
						LOG_DEBUG(L"____Result2:", endPos, L" -> ", curve(endT));

						pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));

						//curve.write(image, 30, RandomColor().setAlpha(128), startT, endT);
					}
				}

				//debugPoss.push_back(debugLoop);

#ifdef DEBUG_OUTPUT_IMAGE
				image.savePNG(Format(L"debug_", imageSaveCount++, L".png"));
#endif

				++loopCount;
			}

			pTree->fixSegments();

			/*
			for (auto& loopCurve : loopCurves)
			{
			Image image(640, 480, Palette::White);

			pTree->nextLoop(isHoles[loopCount]);

			std::vector<Vec2> debugLoop;

			const auto color = RandomColor().setAlpha(64);

			LOG_DEBUG(L"Curve: ");
			for (auto i : step(loopCurve.size()))
			{
			const int zIndex = CombineZIndex(loopCurve[i], loopCurve[(i + 1) % loopCurve.size()], originalPaths);

			const Vec2 startPos = loopCurve[i].m_pos;
			const Vec2 endPos = loopCurve[(i + 1) % loopCurve.size()].m_pos;

			LOG_DEBUG(L"____Pos:", startPos, L", index: ", zIndex);

			debugLoop.push_back(startPos);

			//m_segmentPoss.push_back(startPos);

			if (const auto indicesOpt = UnpackZIndex(zIndex, originalPaths))
			{
			const auto& indices = indicesOpt.value();

			const auto curve = originalPaths[indices.first].segment(indices.second).curve();

			const double startT = curve.closestPoint(startPos);
			const double endT = curve.closestPoint(endPos);

			pTree->addSegment(isHoles[loopCount], CurveSegment(curve, startT, endT), Line(startPos, endPos));

			curve.write(image, 30, RandomColor().setAlpha(128), startT, endT);
			}
			}

			image.savePNG(Format(L"debug_", imageSaveCount++, L".png"));

			debugPoss.push_back(debugLoop);

			pTree->logger(Format(L"B(", loopCount, L")"));

			++loopCount;
			}
			*/

			/*image.savePNG(Format(L"debug_", imageSaveCount++, L".png"));*/
		}

		return pTree;
	}

	std::vector<BezierCurvePath> m_paths;
	Optional<size_t> m_editIndex;

	std::vector<std::pair<LineString, Color>> debugCurves;
	std::vector<SegmentInfo> m_segmentInfos;
	//std::vector<Vec2> m_segmentPoss;

	//std::vector<Line> m_lineForDebug;

	//std::vector<Vec2> m_debugPoss;

	std::shared_ptr<LineStringTree> m_pTree;
};
