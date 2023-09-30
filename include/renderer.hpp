#pragma once
#include <g.h>

#include "state.hpp"

using namespace xmath;

namespace ld54
{

struct Renderer
{
	Renderer(g::asset::store& assets) : assets(assets) {}

	void draw(State& state);

private:
	g::asset::store& assets;
    g::gfx::mesh<g::gfx::vertex::pos_uv_norm> terrain;
};

} // namespace ld54