#include "g.h"
#include "renderer.hpp"
#include "controls.hpp"

#include "state.hpp"
#include "physics.hpp"

namespace ld54
{

struct Game : public g::core
{

	g::asset::store assets;
	State state;
	Renderer renderer;

	Game() : renderer(assets)
	{

	}

	~Game() = default;

	virtual bool initialize()
	{
		state.world.heightmap = assets.tex("heightmap.png");

		state.player.camera.position[1] = state.world.height(state.player.camera.position) + 1.f;

		state.world.cars.push_back(State::Car(state.player.camera.position + vec<3>{0, 20, 0}));

		glPointSize(10.f);
		return true;
	}

	virtual void update(float dt)
	{
		controls(state, dt);

		auto& car = state.world.cars[0];
		auto& cam = state.player.camera;
		auto cam_pos_targ = car.position() + car.forward() * -10 + car.up() * 4;

		if (cam_pos_targ[1] < state.world.height(cam_pos_targ))
		{
			cam_pos_targ[1] = state.world.height(cam_pos_targ) + 1;
		}

		cam.position += (cam_pos_targ - cam.position) * 10 * dt;
		auto cam_forward = (car.position()-cam.position).unit();
		auto cam_up = vec<3>::cross(cam_forward, car.right()).unit();
		// cam.look_at(cam.position, -cam_forward, cam_up);
		cam.look_at(car.position());
		// auto car_pos_proj = car.position().project_onto_plane({0, 1, 0});
		// auto cam_pos_proj = cam.position.project_onto_plane({0, 1, 0});
		// cam.orientation = quat<>::from_axis_angle({0, 1, 0}, -car_pos_proj.angle_to(cam_pos_proj));

		// cam.orientation = 

		for (auto& car : state.world.cars)
		{
			car.step(state, dt);
		}

		renderer.draw(state);
	}

};

}

#ifdef __EMSCRIPTEN__
EM_JS(int, canvas_get_width, (), {
  return document.getElementById('canvas').width;
});

EM_JS(int, canvas_get_height, (), {
  return document.getElementById('canvas').height;
});
#endif

int main (int argc, const char* argv[])
{
	ld54::Game game;

	g::core::opts opts;

	opts.name = "ld54";
	opts.gfx.fullscreen = false;

#ifdef __EMSCRIPTEN__
	auto monitor = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(monitor);

	opts.gfx.width = canvas_get_width();
	opts.gfx.height = canvas_get_height();
#else
	opts.gfx.fullscreen = false;
#endif

	game.start(opts);

	return 0;
}
