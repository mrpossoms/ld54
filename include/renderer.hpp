#pragma once
#include <g.h>

#include "state.hpp"

using namespace xmath;

namespace ld54
{

struct Renderer
{
	void draw(State& state);

private:
    g::gfx::mesh<g::gfx::vertex::pos_uv_norm> terrain;
};

} // namespace ld54