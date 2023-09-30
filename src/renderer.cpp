#include "renderer.hpp"

using namespace ld54;
using namespace g::gfx;

void Renderer::draw(State& state)
{
	glClearColor(0.5, 0.5, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!terrain.is_initialized())
	{
		terrain = g::gfx::mesh<vertex::pos_uv_norm>::from_heightmap(state.world.heightmap);
	}
}