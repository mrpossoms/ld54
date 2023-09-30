#include "g.h"
#include "renderer.hpp"
#include "controls.hpp"

#include "state.hpp"

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

		return true;
	}

	virtual void update(float dt)
	{
		controls(state, dt);
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
