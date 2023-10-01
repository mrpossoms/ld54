#pragma once
#include <g.h>

#include "state.hpp"

using namespace xmath;
using namespace g::gfx;

namespace ld54
{

struct Renderer
{
	Renderer(g::asset::store& assets) : assets(assets) {}

	void draw(State& state);

private:
	g::asset::store& assets;
	float terrain_height_min = 10000;
	float terrain_height_max = -10000;
    mesh<vertex::pos_norm_tan> terrain;
};

} // namespace ld54