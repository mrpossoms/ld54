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

int tex_min(const texture& tex)
{
	int min = tex.sample(0, 0)[0];
	for (int y = 0; y < tex.size[1]; y++)
	{
		for (int x = 0; x < tex.size[0]; x++)
		{
			min = std::min(min, (int)tex.sample(x, y)[0]);
		}
	}
	return min;
}

int tex_max(const texture& tex)
{
	int max = tex.sample(0, 0)[0];
	for (int y = 0; y < tex.size[1]; y++)
	{
		for (int x = 0; x < tex.size[0]; x++)
		{
			max = std::max(max, (int)tex.sample(x, y)[0]);
		}
	}
	return max;
}

void Renderer::draw(State& state)
{
	glClearColor(0.5, 0.5, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if (!terrain.is_initialized())
	{
		// float heightScaling = (14. / 256.);
		terrain_height_min = tex_min(state.world.heightmap); // * heightScaling;
		terrain_height_max = tex_max(state.world.heightmap); // * heightScaling;
		auto hm_zero = (terrain_height_max + terrain_height_min) * 0.5f;

		float debug_max = -10000;
		float debug_min = 10000;

		auto terrain_min = vec<3>{0, 0, 0};
		auto terrain_max = vec<3>{(float)state.world.heightmap.size[0], 0, (float)state.world.heightmap.size[1]};
		auto terrain_center = (terrain_min + terrain_max) * 0.5f;

		terrain = mesh_factory::from_heightmap<vertex::pos_norm_tan>(state.world.heightmap,
			[&](const texture& tex, int x, int y) -> vertex::pos_norm_tan {
				auto height = state.world.height({(float)x, 0, (float)y}, true);
				debug_max = std::max(debug_max, height);
				debug_min = std::min(debug_min, height);
				return {
					// position
					vec<3>{ (float)x, (float)height, (float)y} - terrain_center,
					// normal
					normal_from_heightmap(state.world.heightmap, {x, y}),
					// tangent
					{ 1, 0, 0 },
				};
			}
		);

		std::cout << "heightmap extreams: " << debug_min << ", " << debug_max << std::endl;
	}

	terrain.using_shader(assets.shader("planet.vs+terrain.fs") )
	.set_camera(state.player.camera)
	["u_model"].mat4(mat<4,4>::translation({0, 0, 0}))
	.draw<GL_TRIANGLES>();

	for (State::Car& car : state.world.cars)
	{
		// assets.geo("truck_0/body.obj").using_shader(assets.shader("object.vs+object.fs"))
		// .set_camera(state.player.camera)
		// ["u_model"].mat4(mat<4,4>::translation(car.position()))
		// .draw<GL_TRIANGLES>();

		auto forward = car.forward();
		auto steer_forward = car.steer_forward();

		auto R = car.orientation();

		assets.geo("truck_0/body.obj").using_shader(assets.shader("object.vs+object.fs"))
		.set_camera(state.player.camera)
		["u_model"].mat4(mat<4, 4>::translation(car.position() + car.up() * 0.25f) * R.transpose() * mat<4, 4>::rotation({0, 1, 0}, M_PI))
		.draw<GL_TRIANGLES>();	

		for (unsigned i = 0; i < 4; i++)
		{
			if (i < 2)
			{
				assets.geo("truck_0/tire.obj").using_shader(assets.shader("object.vs+object.fs"))
				.set_camera(state.player.camera)
				["u_model"].mat4(mat<4, 4>::translation(car.wheels[i]->pos + car.up() * 0.33f) * R.transpose() * mat<4, 4>::rotation(car.up(), -car.steer_angle) * mat<4, 4>::rotation({1, 0, 0}, -car.odometer))
				.draw<GL_TRIANGLES>();
			}
			else
			{
				assets.geo("truck_0/tire.obj").using_shader(assets.shader("object.vs+object.fs"))
				.set_camera(state.player.camera)
				["u_model"].mat4(mat<4, 4>::translation(car.wheels[i]->pos + car.up() * 0.33f) * R.transpose() * mat<4, 4>::rotation({1, 0, 0}, -car.odometer))
				.draw<GL_TRIANGLES>();				
			}

			glDisable(GL_DEPTH_TEST);
			if (i < 2)
			{
				g::gfx::debug::print(state.player.camera).color({1, 0, 1, 1}).ray(car.wheels[i]->pos, steer_forward);
			}
			else
			{
				g::gfx::debug::print(state.player.camera).color({1, 0, 1, 1}).ray(car.wheels[i]->pos, forward);
			}
			g::gfx::debug::print(state.player.camera).color({0, 0, 1, 1}).ray(car.position(), car.up());
			glEnable(GL_DEPTH_TEST);
		}

		glDisable(GL_DEPTH_TEST);
		for (unsigned i = 0; i < car.constraints.size(); i++)
		{
			auto& c = car.constraints[i];
			g::gfx::debug::print(state.player.camera).color({0, 1, 0, 1}).line(car.nodes[c.v[0].idx].pos, car.nodes[c.v[1].idx].pos);
		}

		for (unsigned i = 0; i < car.nodes.size(); i++)
		{
			g::gfx::debug::print(state.player.camera).color({1, 0, 0, 1}).point(car.nodes[i].pos);
		}
		glEnable(GL_DEPTH_TEST);
	}
}