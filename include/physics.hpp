#pragma once
#include <g.h>

#include "btBulletDynamicsCommon.h"
#include "state.hpp"

namespace ld54
{
	struct Physics
	{
		vec<3> gravity = {0, -0.81f, 0};

		Physics()
		{
		}

		~Physics()
		{
		}

		void step_car(State& state, State::Car& car, float dt)
		{
			auto sub_steps = 1;
			auto sub_dt = dt / sub_steps;

			for (unsigned i = 0; i < 4; i++)
			{
				car.wheels[i].vel += gravity * sub_dt;

				auto z = state.world.height(car.wheels[i].pos);

				if (z > car.wheels[i].pos[1])
				{
					car.wheels[i].pos[1] = z;
					car.wheels[i].vel[1] = 0;
				}
			}


			for (unsigned step = 0; step < sub_steps; step++)
			{
				for (unsigned i = 0; i < 4; i++)
				{
					for (unsigned j = 0; j < 4; j++)
					{
						if (i == j) continue;

						auto d = car.wheels[j].pos - car.wheels[i].pos;
						auto dist = d.magnitude();
						auto dir = d.unit();

						auto force = (dist - car.wheels[i].nominal_distances[j]); // * 1000.f;

						car.wheels[i].vel += dir * force * sub_dt;
					}

					car.wheels[i].pos += car.wheels[i].vel * sub_dt;
				}				
			}

		}

		void step(State& state, float dt)
		{
			for (auto& car : state.world.cars)
			{
				step_car(state, car, dt);
			}
		}
	};	
}


