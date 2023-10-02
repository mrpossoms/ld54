#include "state.hpp"

using namespace ld54;

State::Car::Car(const vec<3>& pos)
{
	vec<3> wheel_pattern[] = {
		{1.0f, 0, 1.5f},
		{-1.0f, 0, 1.5f},
		{1.0f, 0, -1.5f},
		{-1.0f, 0, -1.5f},
	};

	for (unsigned i = 0; i < 4; i++)
	{
		float t = M_PI/4 + (M_PI / 2) * i;
		Node n = {
			pos + wheel_pattern[i],
			{},
		};

		nodes.push_back(n);

		Node* node = &nodes.back();
		wheels.push_back(node);
	}

	nodes.push_back({pos + vec<3>{0.f, 1.5f, 1.5f}});
	nodes.push_back({pos + vec<3>{0.f, 1.5f, -1.5f}});

	for (unsigned i = 0; i < nodes.size(); i++)
	{
		distances.push_back(std::vector<float>());

		for (unsigned j = 0; j < nodes.size(); j++)
		{
			float dist = 0;
			if (i != j)
			{
				auto d = nodes[i].pos - nodes[j].pos;
				dist = d.magnitude();			
			}

			distances[i].push_back(dist);
		}
	}
}

State::Car::Car(const Car& o)
{
	nodes = o.nodes;
	distances = o.distances;
	
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

	for (unsigned i = 0; i < nodes.size(); i++)
	{
		auto& node = nodes[i];
		auto z = state.world.height(node.pos);
		auto dz = z - node.pos[1];
		if (dz > 0)
		{
			node.pos[1] = z;
			node.vel[1] = 0;
			// node.vel += vec<3>{0, dz, 0} * dt;
		}
		else
		{
			node.vel += gravity * dt;
		}
	}

	// apply constraint solving between wheels in sub steps
	for (unsigned step = 0; step < sub_steps; step++)
	{
		for (unsigned i = 0; i < nodes.size(); i++)
		{
			for (unsigned j = 0; j < nodes.size(); j++)
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

		for (unsigned i = 0; i < nodes.size(); i++)
		{
			nodes[i].pos += nodes[i].vel * sub_dt;
		}
	}

}

vec<3> State::Car::position()
{
	vec<3> net_pos;
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		net_pos += nodes[i].pos;
	}

	return net_pos / (float)nodes.size();
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
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		net_vel += nodes[i].vel;
	}

	return net_vel / (float)nodes.size();	
}

mat<4, 4> State::Car::orientation()
{
	auto r = right();
	auto u = up();
	auto f = forward();

	mat<4, 4> R = {
		{r[0], r[1], r[2], 0},
		{u[0], u[1], u[2], 0},
		{f[0], f[1], f[2], 0},
		{0,    0,    0,    1}
	};

	return R;
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