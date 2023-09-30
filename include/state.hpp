#pragma once

using namespace xmath;
#include <g.gfx.h>
#include <algorithm>

using namespace g::gfx;

namespace ld54
{

struct State
{
	struct {
		g::gfx::texture heightmap;

		float height(vec<3> p)
		{
			auto x = std::max(0, std::min((int)heightmap.size[0], (int)p[0]));
			auto y = std::max(0, std::min((int)heightmap.size[1], (int)p[2]));

			return heightmap.sample(x, y)[0] * 0.25f;
		}
	} world;

	struct {
		g::game::fps_camera camera;
	} player;
};

} // namespace ld54