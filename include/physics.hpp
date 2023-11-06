#pragma once
#include <g.h>

namespace ld54
{

namespace physics
{
namespace pbd
{

struct Node
{
	vec<3> pos;
	vec<3> vel;
	float inv_mass;
};

struct Mesh;

struct Constraint
{
	float distance;
	float stiffness;

	struct {
		unsigned idx;
		Mesh* parent;
	} v[2];
};

struct Mesh
{
	std::vector<Node> nodes;
	std::vector<Constraint> constraints;
};

struct Solver
{
	Solver(unsigned solver_steps=1);

	void add_meshes(const std::initializer_list<Mesh*>& meshes);
	void add_constraints(const std::initializer_list<Constraint>& constraint);
	void step(float dt, std::function<vec<3> (const vec<3>& p)> force_field=nullptr);

	static Solver& instance();

private:
	unsigned m_solver_steps;
	std::vector<vec<3>> m_est_pos;
	std::vector<Node*> m_node_ptrs;
	std::vector<Mesh*> m_meshes;
	std::vector<unsigned> m_mesh_start_index;
	std::vector<Constraint> m_transient_constraints;
};

} // namespace pbd
} // namespace physics
}


