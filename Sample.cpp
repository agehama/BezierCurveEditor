#include <Siv3D.hpp>
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
