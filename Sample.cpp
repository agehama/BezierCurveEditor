#include <Siv3D.hpp>
#include "BezierCurvePath.hpp"

void Main()
{
	BezierCurvePath path;

	while (System::Update())
	{		
		path.update();
		path.draw();
	}
}
