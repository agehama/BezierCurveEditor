#include <Siv3D.hpp>
#include "BezierCurvePath.hpp"

void Main()
{
	Graphics::SetBackground(Palette::White);

	BezierPathClipper clipper;

	while (System::Update())
	{		
		clipper.update();
		clipper.draw();
	}
}
