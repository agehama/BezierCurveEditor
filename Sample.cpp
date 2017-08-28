#include <Siv3D.hpp>
#include "BezierCurvePath.hpp"

void Main()
{
	BezierPathClipper clipper;

	while (System::Update())
	{		
		clipper.update();
		clipper.draw();
	}
}
