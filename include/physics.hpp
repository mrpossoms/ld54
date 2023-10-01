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

		void step_car(State& state, State::Car& car, float dt)
		{
			auto sub_steps = 100;
			auto sub_dt = dt / sub_steps;

			auto steer_vec = car.steer_forward();
			auto forward = car.forward();
			for (unsigned i = 0; i < 4; i++)
			{
				auto& wheel = car.wheels[i];
				auto z = state.world.height(wheel.pos);

				auto dz = z - wheel.pos[1];
				if (dz > 0)
				{
					wheel.pos[1] = z;
					wheel.vel[1] = 0;
					// wheel.vel += vec<3>{0, dz, 0} * dt;
				}
				else
				{
					wheel.vel += gravity * dt;
				}

				if (i < 2)
				{ // front wheels
					float steer_power = steer_vec.dot(car.right()) * wheel.vel.dot(forward);
					wheel.vel += car.right() * steer_power * dt;
				}
				
				// skid friction
				auto skid = wheel.vel.dot(car.right());
				wheel.vel -= car.right() * skid * 4.0f * dt;

				// rolling resistance
				auto rolling = wheel.vel.dot(forward);
				wheel.vel -= forward * rolling * 0.5f * dt;
			}

			// apply constraint solving between wheels in sub steps
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

						auto force = (dist - car.wheels[i].nominal_distances[j]);

						car.wheels[i].vel += dir * force;
						// car.wheels[i].pos += dir * force * 0.5f * sub_dt;
					}
				}

				for (unsigned i = 0; i < 4; i++)
				{
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


