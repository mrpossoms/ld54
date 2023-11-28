#include <g.h>
#include "state.hpp"

void test_normal_x()
{
	ld54::State::World world;

	world.heightmap = g::gfx::texture_factory{8, 8}
		.components(1)
		.type(GL_UNSIGNED_BYTE)
		.fill([](int x, int y, int z, unsigned char* pixel){
			pixel[0] = x;
		})
		.create();

	vec<3> basis[2] = {
		{ 8, 8, 0 },
		{ 0, 0, 8 },
	};

	auto exp_normal = vec<3>::cross(basis[1], basis[0]).unit();

	for (unsigned x = 1; x < 7; x++)
	{
		for (unsigned y = 1; y < 7; y++)
		{
			vec<3> normal;
			auto height = world.height({(float)x, 0, (float)y}, true, &normal);
			assert(fabs(height - x * 0.125f) < 0.0001);
			assert(normal.is_near(exp_normal));
		}
	}
}

void test_normal_y()
{
	ld54::State::World world;

	world.heightmap = g::gfx::texture_factory{8, 8}
		.components(1)
		.type(GL_UNSIGNED_BYTE)
		.fill([](int x, int y, int z, unsigned char* pixel){
			pixel[0] = y;
		})
		.create();

	vec<3> basis[2] = {
		{ 8, 0, 0 },
		{ 0, 8, 8 },
	};

	auto exp_normal = vec<3>::cross(basis[1], basis[0]).unit();

	for (unsigned x = 1; x < 7; x++)
	{
		for (unsigned y = 1; y < 7; y++)
		{
			vec<3> normal;
			assert(fabs(world.height({(float)x, 0, (float)y}, true, &normal) - y * 0.125f) < 0.0001);
			assert(normal.is_near(exp_normal));
		}
	}
}

void test_intersection()
{
	ld54::State::World world;

	world.heightmap = g::gfx::texture_factory{8, 8}
		.components(1)
		.type(GL_UNSIGNED_BYTE)
		.fill([](int x, int y, int z, unsigned char* pixel){
			pixel[0] = 0;
		})
		.create();

	{ // normal, intersection from above terrain
		auto inter = world.ray_intersects({
			{4, 1, 4},
			{0, -2, 0}
		});

		assert(fabs(inter.time - 0.5f) < 0.0001f);
	}

	{ // ray is below the terrain and facing deeper, find intersection point above
		auto inter = world.ray_intersects({
			{4, -1, 4},
			{0, -2, 0}
		});

		assert(fabs(inter.time - -0.5f) < 0.0001f);
	}

	{ // ray is below the terrain and facing deeper, find intersection point above
		auto inter = world.ray_intersects({
			{4, -4, 4},
			{0, -2, 0}
		});

		assert(fabs(inter.time - -2.0f) < 0.0001f);
	}

}

int main(void)
{
	g::gfx::api::opengl gl;
	gl.initialize({}, "");

	test_normal_x();
	test_normal_y();
	test_intersection();

	return 0;
}
