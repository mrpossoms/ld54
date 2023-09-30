#pragma once

using namespace xmath;
#include <g.gfx.h>

using namespace g::gfx;

namespace ld54
{

struct State
{
	struct {
		g::gfx::texture heightmap;
	} world;

	struct {
		g::game::camera_perspective camera;
	} player;
};

} // namespace ld54