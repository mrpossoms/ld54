#include <g.h>
#include "state.hpp"


int main(void)
{
	g::gfx::api::opengl gl;
	gl.initialize({}, "");

	for (unsigned itr = 0; itr < 100; itr++)
	{
		unsigned a = rand();
		unsigned b = rand();

		assert(ld54::State::Car::pair_id(a, b) == ld54::State::Car::pair_id(b, a));		
	}



	return 0;
}
