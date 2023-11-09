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
	enum Type
	{
		Equality,
		Inequality
	};

	struct EqualityParams
	{
		float distance;
	};

	struct InequalityParams
	{
		float normal[3];
	};

	union Params
	{
		EqualityParams equality;
		InequalityParams inequality;
	};

	Type type;
	float stiffness;
	Params params;

	struct {
		unsigned idx;
		Mesh* parent;
	} v[2];
};

struct Mesh : public g::dyn::cd::collider
{
	std::vector<Node> nodes;
	std::vector<Constraint> constraints;

    intersection ray_intersects(const ray& r) const override;
    bool generates_rays() override { return true; }
    std::vector<ray>& rays() override;

private:
	std::vector<ray> m_rays;
	std::vector<intersection> m_intersections;
};

struct Solver
{
	Solver(unsigned solver_steps=1);

	void add_meshes(const std::initializer_list<Mesh*>& meshes);
	void add_constraints(const std::initializer_list<Constraint>& constraint);
	void step(
		float dt, 
		g::dyn::cd::collider& collider,
		std::function<vec<3> (const vec<3>& p)> force_field=nullptr);

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


