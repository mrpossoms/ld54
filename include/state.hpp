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
	};

	struct {
		g::gfx::texture heightmap;
		
		float height(vec<3> p, bool pixel=false)
		{
			if (!pixel)
			{
				p += vec<3>{heightmap.size[0] * 0.5f, 0, heightmap.size[1] * 0.5f};
			}

			auto x = std::max(0, std::min((int)heightmap.size[0], (int)p[0]));
			auto y = std::max(0, std::min((int)heightmap.size[1], (int)p[2]));

			return heightmap.sample(x, y)[0] * 0.125f;
		}

		std::vector<Car> cars;
	} world;

	struct {
		g::game::fps_camera camera;
	} player;
};

} // namespace ld54