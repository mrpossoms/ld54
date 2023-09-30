#include "renderer.hpp"

using namespace ld54;
using namespace g::gfx;



void Renderer::draw(State& state)
{
	glClearColor(0.5, 0.5, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!terrain.is_initialized())
	{
		terrain = g::gfx::mesh_factory::from_heightmap(state.world.heightmap);
	}

	terrain.using_shader(assets.shader("terrain.vs+terrain.fs"))
	.set_camera(state.player.camera)
	["u_model"].mat4(mat<4,4>::I())
	.draw<GL_TRIANGLES>();

}