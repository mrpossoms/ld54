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
    mesh<vertex::pos_norm_tan> terrain;
};

} // namespace ld54