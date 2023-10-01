#pragma once

using namespace xmath;
#include <g.gfx.h>
#include <algorithm>

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

using namespace g::gfx;

namespace ld54
{

struct State
{
	struct Car
	{
		struct Wheel
		{
			vec<3> pos;
			vec<3> vel = {};
			vec<3> forward = {};
			float nominal_distances[4];
		};

		Wheel wheels[4];
		float steer_angle = 0;

		Car(const vec<3>& pos)
		{
			for (unsigned i = 0; i < 4; i++)
			{
				float t = M_PI/4 + (M_PI / 2) * i;
				wheels[i].pos = pos + vec<3>{std::cos(t), 0, std::sin(t)};
				wheels[i].forward = vec<3>{0, 0, 1};
			}

			for (unsigned i = 0; i < 4; i++)
			{
				for (unsigned j = 0; j < 4; j++)
				{
					if (i == j) continue;

					auto d = wheels[i].pos - wheels[j].pos;
					wheels[i].nominal_distances[j] = d.magnitude();
				}
			}
		}

		vec<3> position()
		{
			return (wheels[0].pos + wheels[1].pos + wheels[2].pos + wheels[3].pos) * 0.25f;
		}

		vec<3> steer_forward()
		{
			return quat<>::from_axis_angle(up(), steer_angle).rotate(forward());
		}

		vec<3> forward()
		{
			return (((wheels[0].pos + wheels[1].pos) * 0.5f) - ((wheels[2].pos + wheels[3].pos) * 0.5f)).unit();
		}

		vec<3> right()
		{
			return vec<3>::cross({0, 1, 0}, forward());
		}

		vec<3> up()
		{
			return vec<3>::cross(forward(), right());
		}

		vec<3> velocity()
		{
			return (wheels[0].vel + wheels[1].vel + wheels[2].vel + wheels[3].vel) * 0.25f;
		}

		void accelerate(float amount)
		{
			for (unsigned i = 2; i < 4; i++)
			{
				wheels[i].vel += forward() * amount;
			}
		}

		void steer(float amount)
		{
			steer_angle += amount;

			steer_angle = std::max<float>(-M_PI/4.f, std::min<float>(M_PI/4.f, steer_angle));

			auto f = forward();
			for (unsigned i = 0; i < 2; i++)
			{
				wheels[i].forward = vec<3>{std::cosf(steer_angle), 0, std::sinf(steer_angle)};
			}
		}

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

			return h * 0.25f;
		}

		std::vector<Car> cars;
	} world;

	struct {
		g::game::fps_camera camera;
	} player;
};

} // namespace ld54