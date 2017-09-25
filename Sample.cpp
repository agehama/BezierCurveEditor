#include <Siv3D.hpp>
#include <HamFramework.hpp>
#include "BezierCurvePath.hpp"

void Main()
{
	Graphics::SetBackground(Color(245, 245, 245));

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

/*
void Main()
{
	Graphics::SetBackground(Color(245, 245, 245));

	//BezierPathClipper clipper;
	Profiler::EnableWarning(false);

	std::vector<BezierCurvePath> paths;

	Optional<size_t> editIndex;

	Camera2D camera2d;

	GUI gui(GUIStyle::Default);
	gui.add(L"type", GUIRadioButton::Create({ L"Intersection",L"Union",L"Difference",L"Exclusive" }, 0));
	
	while (System::Update())
	{
		camera2d.update();
		
		{
			auto t = camera2d.createTransformer();

			if (!gui.getRect().mouseOver)
			{
				bool anyCurveEdited = false;
				for (auto& path : paths)
				{
					if (path.update())
					{
						anyCurveEdited = true;
						break;
					}
				}

				if (!anyCurveEdited && Input::MouseL.clicked && (paths.empty() || paths.back().isClosed()))
				{
					paths.emplace_back(Mouse::PosF());
				}
			}

			if (2 <= paths.size() && paths[1].isClosed())
			{
				const uint32 type = gui.radioButton(L"type").checkedItem.value();
				auto result = BezierPathClipper::Clip(paths[0], paths[1], static_cast<ClipperLib::ClipType>(type));

				if (result)
				{
					result->drawAll();
				}

				for (const auto& path : paths)
				{
					path.draw(result ? 96 : 255);
				}
			}
			else
			{
				for (const auto& path : paths)
				{
					path.draw();
				}
			}
		}

		camera2d.draw();
	}
}
*/