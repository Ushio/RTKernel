#pragma once

#include "raytracing_kernel.hpp"

inline std::shared_ptr<rt::Scene> scene_conelbox() {
	rt::CameraSetting cameraSetting;
	cameraSetting._imageHeight = 512;
	cameraSetting._imageWidth = 512;
	cameraSetting._eye.z = 1.0;

	std::vector<std::shared_ptr<rt::SceneElement>> sceneElements;
	// back
	{
		double size = 1.0;
		std::shared_ptr<rt::PolygonSceneElement> back(new rt::PolygonSceneElement(
			{
				rt::Vec3(-size, size, -1.0),
				rt::Vec3(size, size, -1.0),
				rt::Vec3(-size, -size, -1.0),
				rt::Vec3(size, -size, -1.0)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(1.0), rt::Vec3(0.75)),
			true
		));
		sceneElements.push_back(back);
	}
	// テスト あとでコーネルボックス作る
	{
		double size = 0.5;
		std::shared_ptr<rt::PolygonSceneElement> back(new rt::PolygonSceneElement(
			{
				rt::Vec3(-size, size, 1.0),
				rt::Vec3(size, size, 1.0),
				rt::Vec3(-size, -size, 1.0),
				rt::Vec3(size, -size, 1.0)
			},
			{
				1, 0, 2,
				1, 2, 3
			},
			rt::LambertianMaterial(rt::Vec3(1.0), rt::Vec3(0.75)),
			false
		));
		sceneElements.push_back(back);
	}

	return std::shared_ptr<rt::Scene>(new rt::Scene(cameraSetting, sceneElements));
}