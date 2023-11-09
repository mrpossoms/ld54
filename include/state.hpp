#pragma once

#include <g.gfx.h>
#include <algorithm>
#include <optional>

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
		void nudge(const vec<3>& delta);
		vec<3> position(std::optional<vec<3>> pos=std::nullopt);
		void rotate(const quat<>& Q);
		vec<3> steer_forward();
		vec<3> forward();
		vec<3> right();
		vec<3> up();
		vec<3> velocity(std::optional<vec<3>> vel=std::nullopt);
		mat<4, 4> orientation();
		void accelerate(float amount);
		void steer(float amount);
		void flip();
	};

	struct World : public g::dyn::cd::collider
	{
		g::gfx::texture heightmap;
		
		float height(vec<3> p, bool pixel=false, vec<3>* normal_out = nullptr) const
		{
			if (!pixel)
			{
				p += vec<3>{heightmap.size[0] * 0.5f, 0, heightmap.size[1] * 0.5f};
			}

			auto x_0 = std::max(0, std::min((int)heightmap.size[0]-1, (int)p[0]));
			auto y_0 = std::max(0, std::min((int)heightmap.size[1]-1, (int)p[2]));

			auto x_1 = std::max(0, std::min((int)heightmap.size[0]-1, (int)p[0]+1));
			auto y_1 = std::max(0, std::min((int)heightmap.size[1]-1, (int)p[2]+1));

			auto h_00 = (float)heightmap.sample(x_0, y_0)[0];
			auto h_01 = (float)heightmap.sample(x_0, y_1)[0];
			auto h_10 = (float)heightmap.sample(x_1, y_0)[0];
			auto h_11 = (float)heightmap.sample(x_1, y_1)[0];

			if (normal_out)
			{
				*normal_out = vec<3>::cross(
					vec<3>{(float)x_0, h_01, (float)y_1} - vec<3>{(float)x_0, h_00, (float)y_0},
					vec<3>{(float)x_1, h_10, (float)y_0} - vec<3>{(float)x_0, h_00, (float)y_0}
				).unit();
			}

			auto dx = p[0] - (int)p[0];
			auto dy = p[2] - (int)p[2];

			auto h_0 = h_00 * (1 - dx) + h_10 * dx;
			auto h_1 = h_01 * (1 - dx) + h_11 * dx;
			auto h = h_0 * (1 - dy) + h_1 * dy;

			return h * 0.125f;
		}

		bool generates_rays() override { return false; }
		std::vector<ray>& rays() override
		{
			static std::vector<ray> rays;
			return rays; 
		}
		intersection ray_intersects(const ray& r) const override
		{
			auto& p0 = r.position;
			auto p1 = r.position + r.direction;
			auto h0 = height(p0);
			auto h1 = height(p1);
			
			if ((p0[1] - h0) < 0 || (p1[1] - h1) < 0)
			{
				auto dy = r.direction[1];
				auto t = (h0 - p0[1]) / dy;
				auto intersect = r.position + r.direction * t;
				vec<3> normal;
				height(intersect, false, &normal);
				return {
					t,
					r.position,
					r.direction,
					intersect,
					normal
				};
			}
			else
			{
				return {};
			}
		}


		std::vector<Car> cars;
	};

	World world;

	struct {
		g::game::fps_camera camera;
	} player;
};

} // namespace ld54