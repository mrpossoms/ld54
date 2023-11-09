#include "state.hpp"

using namespace ld54;
using namespace ld54::physics::pbd;

static unsigned pair_id(unsigned i, unsigned j)
{
	if (i < j)
	{
		return i * j + j;
	}
	else
	{
		return j * i + i;
	}
}

State::Car::Car(const vec<3>& pos, float mass)
{
	vec<3> wheel_pattern[] = {
		{1.1f, 0, 1.5f},
		{-1.1f, 0, 1.5f},
		{1.1f, 0, -1.5f},
		{-1.1f, 0, -1.5f},
	};

	for (unsigned i = 0; i < 4; i++)
	{
		float t = M_PI/4 + (M_PI / 2) * i;
		physics::pbd::Node n = {
			pos + wheel_pattern[i],
			{},
			1.f / 100.f
		};

		nodes.push_back(n);

		physics::pbd::Node* node = &nodes.back();
		wheels.push_back(node);
	}

	nodes.push_back({pos + vec<3>{-0.5f, 1.5f, 1.35f}});
	nodes.push_back({pos + vec<3>{-0.5f, 1.5f, -1.35f}});
	nodes.push_back({pos + vec<3>{0.5f, 1.5f, 1.35f}});
	nodes.push_back({pos + vec<3>{0.5f, 1.5f, -1.35f}});


	// nodes.push_back({pos + vec<3>{-0.5f, 1.0f, 1.35f}});
	// nodes.push_back({pos + vec<3>{-0.5f, 1.0f, -1.35f}});
	// nodes.push_back({pos + vec<3>{0.5f, 1.0f, 1.35f}});
	// nodes.push_back({pos + vec<3>{0.5f, 1.0f, -1.35f}});

	std::set<unsigned> pairs;

	for (unsigned i = 0; i < nodes.size(); i++)
	{
		for (unsigned j = 0; j < nodes.size(); j++)
		{
			float dist = 0;
			auto id = pair_id(i, j);
			if (i != j && pairs.find(id) == pairs.end())
			{
				auto d = nodes[i].pos - nodes[j].pos;
				dist = d.magnitude();

				float stiffness = 1.0f;

				if ((i < 4 && j >= 4) || (i >= 4 && j < 4))
				{
					stiffness = 0.2f;
				}
				
				constraints.push_back(Constraint{
					.type = Constraint::Type::Equality,
					.stiffness = stiffness,
					.params = { .equality = { dist } },
					.v {
						{ .idx = i, .parent = this },
						{ .idx = j, .parent = this },
					}
				});

				pairs.insert(id);
			}

			// distances[i].push_back(dist);
		}

		nodes[i].inv_mass = 1.0f / (mass / (float)nodes.size());
	}
}

State::Car::Car(const Car& o)
{
	nodes = o.nodes;
	constraints = o.constraints;
	
	for (unsigned i = 0; i < 4; i++)
	{
		wheels.push_back(&nodes[i]);
	}
}

void State::Car::step(State& state, float dt)
{
	const auto gravity = vec<3>{0, -9.81f, 0};

	auto sub_steps = 10;
	auto sub_dt = dt / sub_steps;

	auto steer_vec = steer_forward();
	auto fwd = forward();
	bool wheel_near_ground = false;
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

		if (wheel.pos[1] < state.world.height(wheel.pos))
		{
			wheel_near_ground = true;
		}
	}

	odometer += fwd.dot(velocity()) * 2.5 * dt;

	// Apply external forces
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		auto& node = nodes[i];
		// auto z = state.world.height(node.pos);
		// auto dz = z - node.pos[1];
		// if (dz > 0)
		// {
		// 	node.pos[1] = z;
		// 	node.vel[1] = 0;
		// 	// node.vel += vec<3>{0, dz, 0} * dt;
		// }
		// else
		{
			node.vel += gravity * dt;
		}
	}

/*
	// generate estimated positions for each vertex
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		nodes[i].est_pos = nodes[i].pos + nodes[i].vel * dt;
	}

	// apply constraint solving between wheels in sub steps
	for (unsigned step = 0; step < sub_steps; step++)
	{
		for (unsigned i = 0; i < nodes.size(); i++)
		{
			for (unsigned j = 0; j < nodes.size(); j++)
			{
				if (i == j) continue;

				auto d = nodes[i].est_pos - nodes[j].est_pos;
				auto dist = d.magnitude();
				auto dir = d.unit();

				float w[2] = {
					nodes[i].inv_mass,
					nodes[j].inv_mass,
				};
				float w0_w1 = w[0] + w[1];

				// auto force = (dist - distances[i][j]);
				vec<3> dp[2] = {
					dir * (w[0] / w0_w1) * -(dist - distances[i][j]),
					// dir * (w[1] / w0_w1) * (dist - distances[i][j]),
				};

				nodes[i].est_pos += dp[0];
				// nodes[j].est_pos += dp[1];

				// nodes[i].vel += dir * force * 0.9f;
				// nodes[i].pos += dir * force * 0.5f * sub_dt;
			}

		}
	}

	// correct velocities and positions
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		auto dp = nodes[i].est_pos - nodes[i].pos;
		nodes[i].vel = dp / dt;
		nodes[i].pos = nodes[i].est_pos;
		// nodes[i].pos += nodes[i].vel * sub_dt;
	}
	*/
	physics::pbd::Solver::instance().add_meshes({this});
	physics::pbd::Solver::instance().step(dt, state.world);
}

void State::Car::nudge(const vec<3>& delta)
{
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		nodes[i].pos += delta;
	}
}

vec<3> State::Car::position(std::optional<vec<3>> pos)
{
	if (pos)
	{
		auto delta = *pos - position();
		nudge(delta);

		return *pos;
	}

	vec<3> net_pos;
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		net_pos += nodes[i].pos;
	}

	return net_pos / (float)nodes.size();
}

void State::Car::rotate(const quat<>& Q)
{
	auto delta = position();
	for (unsigned i = 0; i < nodes.size(); i++)
	{
		nodes[i].pos = (Q.rotate(nodes[i].pos - delta)) + delta;
	}
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

vec<3> State::Car::velocity(std::optional<vec<3>> vel)
{
	if (vel)
	{
		for (unsigned i = 0; i < nodes.size(); i++)
		{
			nodes[i].vel = *vel;
		}

		return *vel;
	}

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

	odometer += amount;
}

void State::Car::steer(float amount)
{
	steer_angle += amount;

	steer_angle = std::max<float>(-M_PI/4.f, std::min<float>(M_PI/4.f, steer_angle));
}

void State::Car::flip()
{
	auto tilt = up().angle_to(vec<3>{0, 1, 0});

	if (tilt > M_PI / 4)
	{
		auto Q = quat<>::from_axis_angle(forward(), -tilt);
		rotate(Q);
		nudge(vec<3>{0, 4, 0});
		velocity(vec<3>{0, 0, 0});
	}
}