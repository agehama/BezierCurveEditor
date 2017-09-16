#include <Siv3D.hpp>
#include <HamFramework.hpp>
#include "BezierCurvePath.hpp"

void Main()
{
	//Graphics::SetBackground(Palette::White);

	BezierPathClipper clipper;

	Camera2D camera2d;

	while (System::Update())
	{		
		camera2d.update();
		{
			auto t = camera2d.createTransformer();
			clipper.update();
			clipper.draw();
		}
		camera2d.draw();
	}
}
