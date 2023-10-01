#pragma once
#include <g.h>

#include "btBulletDynamicsCommon.h"
#include "state.hpp"

namespace ld54
{
	struct Physics
	{
		vec<3> gravity = {0, -9.81f, 0};

		Physics()
		{
		}

		~Physics()
		{
		}

		void step(State& state, float dt)
		{
			for (auto& car : state.world.cars)
			{
				car.step(state, dt);
			}
		}
	};	
}


