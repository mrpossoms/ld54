#include "state.hpp"

float ld54::State::World::height(vec<3> p, bool pixel, vec<3>* normal_out) const
{
	if (!pixel)
	{
		p += vec<3>{heightmap.size[0] * 0.5f, 0, heightmap.size[1] * 0.5f};
	}

	const int max_x = heightmap.size[0]-1;
	const int max_y = heightmap.size[1]-1;

	auto x_0 = std::clamp((int)p[0], 0, max_x); //std::max(0, std::min(max_x, (int)p[0]));
	auto y_0 = std::clamp((int)p[2], 0, max_y);

	auto x_1 = x_0+1;//std::clamp(x_0+1, 0, max_x);
	auto y_1 = y_0+1;//std::clamp(y_0+1, 0, max_y);

	auto h_00 = (float)heightmap.sample(x_0, y_0)[0] * (max_height / 255.f);
	auto h_01 = (float)heightmap.sample(x_0, y_1)[0] * (max_height / 255.f);
	auto h_10 = (float)heightmap.sample(x_1, y_0)[0] * (max_height / 255.f);
	auto h_11 = (float)heightmap.sample(x_1, y_1)[0] * (max_height / 255.f);

	if (normal_out)
	{
		auto O = vec<3>{(float)x_0, h_00, (float)y_0};
		vec<3> basis[] = {
			vec<3>{(float)x_0, h_01, (float)y_1} - O,
			vec<3>{(float)x_1, h_10, (float)y_0} - O					
		};

		*normal_out = vec<3>::cross(basis[0], basis[1]).unit();

		assert(fabs(normal_out->magnitude() - 1) < 0.001f);
	}

	float px = p[0] - (int)p[0];
	float py = p[2] - (int)p[2];

	auto h_0 = h_00 * (1 - px) + h_10 * px;
	auto h_1 = h_01 * (1 - px) + h_11 * px;
	auto h = h_0 * (1 - py) + h_1 * py;

	return h; // * 0.125f;
}

intersection ld54::State::World::ray_intersects(const ray& r) const
{
	auto& p0 = r.position;
	auto p1 = r.position + r.direction;
	vec<3> h0_norm;
	auto h0 = height(p0, false, &h0_norm);
	auto h1 = height(p1);
	
	auto origin_below = h0 > p0[1];
	auto ray_below = h1 > p1[1];

	if (origin_below)
	{
		h0_norm = {0, 1, 0};
		auto dy = h0_norm[1];
		auto t = (h0 - p1[1]) / dy;
		auto intersect = p1 + h0_norm * t;
		
		return {
			t,
			r.position,
			r.direction,
			intersect,
			h0_norm
		};
	}
	else if (ray_below)
	{
		auto dy = r.direction[1];
		auto t = (h0 - p0[1]) / dy;
		auto intersect = r.position + r.direction * t;
		vec<3> normal;
		height(intersect, false, &normal);
		return {
			t,
			r.position,
			r.direction,
			intersect,
			normal
		};
	}
	else
	{
		return {};
	}
}