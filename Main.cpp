#include <Siv3D.hpp> // August 2016 v2
#include "BezierCurves.hpp"

void Main()
{
	BezierCurves curve;

	while (System::Update())
	{		
		curve.update();
		curve.draw();
	}
}
