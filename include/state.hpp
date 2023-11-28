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
		static unsigned pair_id(unsigned i, unsigned j);
	};

	struct World : public g::dyn::cd::collider
	{
		g::gfx::texture heightmap;
		float max_height = 20;

		float height(vec<3> p, bool pixel=false, vec<3>* normal_out = nullptr) const;

		bool generates_rays() override { return false; }
		
		std::vector<ray>& rays() override
		{
			static std::vector<ray> rays;
			return rays; 
		}

		intersection ray_intersects(const ray& r) const override;


		std::vector<Car> cars;
	};

	World world;

	struct {
		g::game::fps_camera camera;
	} player;
};

} // namespace ld54