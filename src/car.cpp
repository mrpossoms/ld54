#include "state.hpp"

using namespace ld54;

State::Car::Car(const vec<3>& pos)
{
	auto node_count = sizeof(nodes) / sizeof(nodes[0]);
	for (unsigned i = 0; i < node_count; i++)
	{
		float t = M_PI/4 + (M_PI / 2) * i;
		nodes[i].pos = pos + vec<3>{std::cos(t), 0, std::sin(t)};
		// nodes[i].forward = vec<3>{0, 0, 1};

		Node* node = &nodes[i];
		wheels.push_back(node);
	}

	for (unsigned i = 0; i < node_count; i++)
	{
		for (unsigned j = 0; j < node_count; j++)
		{
			if (i == j) continue;

			auto d = nodes[i].pos - nodes[j].pos;
			distances[i][j] = d.magnitude();
		}
	}
}

State::Car::Car(const Car& o)
{
	memcpy(nodes, o.nodes, sizeof(nodes));
	memcpy(distances, o.distances, sizeof(distances));

	for (unsigned i = 0; i < 4; i++)
	{
		wheels.push_back(&nodes[i]);
	}
}

void State::Car::step(State& state, float dt)
{
	const auto gravity = vec<3>{0, -9.81f, 0};

	auto sub_steps = 100;
	auto sub_dt = dt / sub_steps;

	auto steer_vec = steer_forward();
	auto fwd = forward();
	for (unsigned i = 0; i < wheels.size(); i++)
	{
		auto& wheel = *wheels[i];
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
			float steer_power = steer_vec.dot(right()) * wheel.vel.dot(fwd);
			wheel.vel += right() * steer_power * dt;
		}
		
		// skid friction
		auto skid = wheel.vel.dot(right());
		wheel.vel -= right() * skid * 4.0f * dt;

		// rolling resistance
		auto rolling = wheel.vel.dot(fwd);
		wheel.vel -= fwd * rolling * 0.5f * dt;
	}

	// apply constraint solving between wheels in sub steps
	for (unsigned step = 0; step < sub_steps; step++)
	{
		for (unsigned i = 0; i < wheels.size(); i++)
		{
			for (unsigned j = 0; j < wheels.size(); j++)
			{
				if (i == j) continue;

				auto d = nodes[j].pos - nodes[i].pos;
				auto dist = d.magnitude();
				auto dir = d.unit();

				auto force = (dist - distances[i][j]);

				nodes[i].vel += dir * force * 0.9f;
				// nodes[i].pos += dir * force * 0.5f * sub_dt;
			}

		}

		for (unsigned i = 0; i < wheels.size(); i++)
		{
			nodes[i].pos += nodes[i].vel * sub_dt;
		}
	}

}

vec<3> State::Car::position()
{
	vec<3> net_pos;
	auto node_count = sizeof(nodes) / sizeof(nodes[0]);
	for (unsigned i = 0; i < node_count; i++)
	{
		net_pos += nodes[i].pos;
	}

	return net_pos / (float)node_count;
}

vec<3> State::Car::steer_forward()
{
	return quat<>::from_axis_angle(up(), steer_angle).rotate(forward());
}

vec<3> State::Car::forward()
{
	return (((wheels[0]->pos + wheels[1]->pos) * 0.5f) - ((wheels[2]->pos + wheels[3]->pos) * 0.5f)).unit();
}

vec<3> State::Car::right()
{
	return (wheels[0]->pos - wheels[1]->pos).unit();
}

vec<3> State::Car::up()
{
	return vec<3>::cross(forward(), right());
}

vec<3> State::Car::velocity()
{
	vec<3> net_vel;
	auto node_count = sizeof(nodes) / sizeof(nodes[0]);
	for (unsigned i = 0; i < node_count; i++)
	{
		net_vel += nodes[i].vel;
	}

	return net_vel / (float)node_count;	
}

void State::Car::accelerate(float amount)
{
	for (unsigned i = 2; i < 4; i++)
	{
		wheels[i]->vel += forward() * amount;
	}
}

void State::Car::steer(float amount)
{
	steer_angle += amount;

	steer_angle = std::max<float>(-M_PI/4.f, std::min<float>(M_PI/4.f, steer_angle));
}