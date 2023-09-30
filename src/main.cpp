#include "g.h"
#include "state.hpp"

namespace ld54
{

struct Game : public g::core
{

	g::asset::store assets;
	State state;

	Game() = default;
	~Game() = default;

	virtual bool initialize()
	{
		assets.tex("heightmap.png");

		return true;
	}

	virtual void update(float dt)
	{

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
