#include "physics.hpp"


using namespace ld54;
using namespace physics::pbd;
using namespace g::dyn::cd;


intersection physics::pbd::Mesh::ray_intersects(const ray& r) const
{
	// TODO
	return {};
}

std::vector<ray>& physics::pbd::Mesh::rays()
{
	m_rays.clear();
	for (auto& node : nodes)
	{
		m_rays.push_back({node.pos, node.vel});
	}
	return m_rays;
}

physics::pbd::Solver::Solver(unsigned solver_steps)
{
	m_solver_steps = solver_steps;
}

void physics::pbd::Solver::add_meshes(const std::initializer_list<Mesh*>& meshes)
{
	m_meshes.clear();
	unsigned node_count = 0;
	for (auto mesh : meshes)
	{
		m_mesh_start_index.push_back(node_count);
		m_meshes.push_back(mesh);
		node_count += mesh->nodes.size();
	}

	m_est_pos.reserve(node_count);
}

void physics::pbd::Solver::add_constraints(const std::initializer_list<Constraint>& constraint)
{
	m_transient_constraints.clear();
	for (auto c : constraint)
	{
		if (c.v[0].parent && c.v[1].parent)
		{
			m_transient_constraints.push_back(c);
		}
	}
}

static inline void project_constraint(const Constraint& c, const Node* n0, const Node* n1, vec<3>& est_p0, vec<3>& est_p1, unsigned iterations)
{
	auto d = est_p0 - est_p1;
	auto dist = d.magnitude();
	auto dir = d.unit();

	float w[2] = {
		n0->inv_mass,
		n1->inv_mass,
	};
	float w0_w1 = w[0] + w[1];

	vec<3> dp[2] = {
		dir * (w[0] / w0_w1) * -(dist - c.distance),
		dir * (w[1] / w0_w1) * (dist - c.distance),
	};

	auto k_prime = 1.f - powf((1.f - c.stiffness), 1 / (float)iterations);

	est_p0 += dp[0] * k_prime;
	est_p1 += dp[1] * k_prime;
}

void physics::pbd::Solver::step(
	float dt, 
	g::dyn::cd::collider& collider,
	std::function<vec<3> (const vec<3>& p)> force_field)
{
	m_est_pos.clear();
	m_node_ptrs.clear();

	// generate estimated positions for each mesh
	for (auto mesh : m_meshes)
	{
		for (unsigned i = 0; i < mesh->nodes.size(); i++)
		{
			auto& node = mesh->nodes[i];

			if (force_field)
			node.vel += force_field(node.pos) * node.inv_mass * dt;

			m_est_pos.push_back(node.pos + node.vel * dt);
			m_node_ptrs.push_back(&node);
		}
	}

	// generate collision contraints
	m_transient_constraints.clear();
	for (unsigned mi = 0; mi < m_meshes.size(); mi++)
	{
		auto& mesh = m_meshes[mi];
		auto& start_index = m_mesh_start_index[mi];

		for (unsigned i = 0; i < mesh->nodes.size(); i++)
		{
			auto& node = mesh->nodes[i];

			g::dyn::cd::ray r = {node.pos, node.vel * dt};

			auto intersect = collider.ray_intersects(r);
			if (intersect)
			{
				static Node static_node = {
					{},
					{},
					std::numeric_limits<float>::epsilon(),
				};

				m_est_pos.push_back(intersect.point);
				m_node_ptrs.push_back(&static_node);

				m_transient_constraints.push_back({
					0,
					1.f,
					{
						{i + start_index, mesh}, 
						{m_est_pos.size()-1, nullptr}
					}
				});
			}
		}
	}



	// Solve constraint for all meshes
	for (unsigned step = 0; step < m_solver_steps; step++)
	{
		// Solve constraints belonging to each mesh
		for (unsigned mi = 0; mi < m_meshes.size(); mi++)
		{
			auto& mesh = m_meshes[mi];
			auto& start_index = m_mesh_start_index[mi];

			for (unsigned ci = 0; ci < mesh->constraints.size(); ci++)
			{
				auto& c = mesh->constraints[ci];
				project_constraint(
					c, 
					m_node_ptrs[c.v[0].idx + start_index], 
					m_node_ptrs[c.v[1].idx + start_index], 
					m_est_pos[c.v[0].idx + start_index], 
					m_est_pos[c.v[1].idx + start_index],
					m_solver_steps
				);
			}
		}

		// Solve transient constraints (constraints between meshes / environment)
		for (auto& c : m_transient_constraints)
		{
			project_constraint(
				c, 
				m_node_ptrs[c.v[0].idx], 
				m_node_ptrs[c.v[1].idx], 
				m_est_pos[c.v[0].idx], 
				m_est_pos[c.v[1].idx],
				m_solver_steps
			);
		}
	}

	// correct velocities and positions
	for (unsigned mi = 0; mi < m_meshes.size(); mi++)
	{
		auto& mesh = m_meshes[mi];
		auto start_index = m_mesh_start_index[mi];

		for (unsigned i = 0; i < mesh->nodes.size(); i++)
		{
			auto& node = mesh->nodes[i];
			node.vel = (m_est_pos[start_index + i] - node.pos) / dt;
			node.pos = m_est_pos[start_index + i];
		}
	}
}

Solver& physics::pbd::Solver::instance()
{
	static Solver s;
	return s;
}
