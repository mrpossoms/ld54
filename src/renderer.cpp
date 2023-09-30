#include "renderer.hpp"
#include <algorithm>

using namespace ld54;
using namespace g::gfx;

vec<3> normal_from_heightmap(const texture& tex, vec<2, int> pixel)
{
	vec<3> x_normal = {}, y_normal = {};
	int x_samples = 0, y_samples = 0;

	for (int x = std::max(0, (int)pixel[0] - 1); x < std::min((int)tex.size[0]-1, pixel[0] + 1); x++)
	{
		auto p_0 = vec<3>{(float)x, (float)pixel[1], (float)tex.sample(x, pixel[1])[0]};
		auto p_1 = vec<3>{(float)x+1, (float)pixel[1], (float)tex.sample(x+1, pixel[1])[0]};

		x_normal += vec<3>::cross({0, 0, 1}, p_1 - p_0).unit();
		x_samples += 1;
	}

	for (int y = std::max(0, (int)pixel[1] - 1); y < std::min((int)tex.size[1]-1, pixel[1] + 1); y++)
	{
		auto p_0 = vec<3>{(float)pixel[0], (float)y, (float)tex.sample(pixel[0], y)[0]};
		auto p_1 = vec<3>{(float)pixel[0], (float)y+1, (float)tex.sample(pixel[0], y+1)[0]};

		y_normal += vec<3>::cross({1, 0, 0}, p_1 - p_0).unit();
		y_samples += 1;
	}

	return (x_normal + y_normal) / (x_samples + y_samples);
}

void Renderer::draw(State& state)
{
	glClearColor(0.5, 0.5, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!terrain.is_initialized())
	{
		terrain = mesh_factory::from_heightmap<vertex::pos_norm_tan>(state.world.heightmap,
			[&](const texture& tex, int x, int y) -> vertex::pos_norm_tan {


				return {
					// position
					{
						x - tex.size[0] * 0.5f,
						static_cast<float>(tex.sample(x, y)[0] * 0.25f),
						y - tex.size[1] * 0.5f,
					},
					// normal
					normal_from_heightmap(state.world.heightmap, {x, y}),
					// tangent
					{ 1, 0, 0 },
				};
			}
		);
	}

	terrain.using_shader(assets.shader("planet.vs+terrain.fs") )
	.set_camera(state.player.camera)
	["u_model"].mat4(mat<4,4>::I())
	.draw<GL_TRIANGLES>();

}