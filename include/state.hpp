#pragma once

#include <g.gfx.h>
#include <algorithm>

#include "physics.hpp"

using namespace xmath;
using namespace g::gfx;

namespace ld54
{

struct State
{
	struct Car : public ld54::physics::pbd::Mesh
	{

		// struct Node
		// {
		// 	vec<3> pos;
		// 	vec<3> est_pos;
		// 	vec<3> vel;
		// 	float inv_mass;
		// };

		// std::vector<Node> nodes;
		// std::vector<std::vector<float>> distances;
		float steer_angle = 0;
		float odometer = 0;
		std::vector<physics::pbd::Node*> wheels;

		Car(const vec<3>& pos, float mass=1000);
		Car(const Car& o);
		void step(State& state, float dt);
		vec<3> position();
		vec<3> steer_forward();
		vec<3> forward();
		vec<3> right();
		vec<3> up();
		vec<3> velocity();
		mat<4, 4> orientation();
		void accelerate(float amount);
		void steer(float amount);
	};

	struct {
		g::gfx::texture heightmap;
		
		float height(vec<3> p, bool pixel=false)
		{
			if (!pixel)
			{
				p += vec<3>{heightmap.size[0] * 0.5f, 0, heightmap.size[1] * 0.5f};
			}

			auto x_0 = std::max(0, std::min((int)heightmap.size[0]-1, (int)p[0]));
			auto y_0 = std::max(0, std::min((int)heightmap.size[1]-1, (int)p[2]));

			auto x_1 = std::max(0, std::min((int)heightmap.size[0]-1, (int)p[0]+1));
			auto y_1 = std::max(0, std::min((int)heightmap.size[1]-1, (int)p[2]+1));

			auto h_00 = heightmap.sample(x_0, y_0)[0];
			auto h_01 = heightmap.sample(x_0, y_1)[0];
			auto h_10 = heightmap.sample(x_1, y_0)[0];
			auto h_11 = heightmap.sample(x_1, y_1)[0];

			auto dx = p[0] - (int)p[0];
			auto dy = p[2] - (int)p[2];

			auto h_0 = h_00 * (1 - dx) + h_10 * dx;
			auto h_1 = h_01 * (1 - dx) + h_11 * dx;
			auto h = h_0 * (1 - dy) + h_1 * dy;

			return h * 0.125f;
		}

		std::vector<Car> cars;
	} world;

	struct {
		g::game::fps_camera camera;
	} player;
};

} // namespace ld54