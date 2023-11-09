#pragma once
#include <g.h>

#include "state.hpp"

using namespace xmath;
using namespace g::gfx;

namespace ld54
{

void setup_camera(State& state, g::game::fps_camera& cam)
{
	cam.on_input = [&](fps_camera& cam, float dt){
	    static double xlast, ylast;
	    float sensitivity = 0.5f;
	    double xpos = 0, ypos = 0;
	    auto mode = glfwGetInputMode(g::gfx::GLFW_WIN, GLFW_CURSOR);

	    if (GLFW_CURSOR_DISABLED == mode)
	    {
	        glfwGetCursorPos(g::gfx::GLFW_WIN, &xpos, &ypos);
	    }

	    if (glfwGetInputMode(g::gfx::GLFW_WIN, GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
	    if (xlast != 0 || ylast != 0)
	    {
	        auto dx = xpos - xlast;
	        auto dy = ypos - ylast;
	        cam.pitch += (dy * dt * sensitivity);
	        cam.yaw += (-dx * dt * sensitivity);
	    }

	    xlast = xpos; ylast = ypos;

	    auto speed = cam.speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) speed *= (cam.touching_surface ? 5 : 1);
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_W) == GLFW_PRESS) cam.velocity += cam.body_forward() * speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_S) == GLFW_PRESS) cam.velocity += cam.body_forward() * -speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_A) == GLFW_PRESS) cam.velocity += cam.body_left() * speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_D) == GLFW_PRESS) cam.velocity += cam.body_left() * -speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_SPACE) == GLFW_PRESS) cam.velocity += cam.body_up() * speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) cam.velocity += cam.body_up() * -speed;
	    if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetInputMode(g::gfx::GLFW_WIN, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	    if (glfwGetMouseButton(g::gfx::GLFW_WIN, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	    {
	        glfwSetInputMode(g::gfx::GLFW_WIN, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	    }
	};

	cam.gravity = {};
}

void controls(State& state, float dt)
{
	auto& cam = state.player.camera;

	if (cam.on_input == nullptr)
	{
		setup_camera(state, cam);
	}

    // process input and update the velocities.
    // cam.pre_update(dt, 0);
    cam.aspect_ratio(g::gfx::aspect());
    // cam.velocity -= (cam.velocity * dt);

    // after velocities have been corrected, update the camera's position
    // cam.update(dt, 0);

	if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetInputMode(g::gfx::GLFW_WIN, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	auto& car = state.world.cars[0];
	auto forward = car.forward();
	if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_UP) == GLFW_PRESS)
	{
		car.accelerate(30.f * dt);
	}

	if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		car.accelerate(-30.f * dt);
	}

	if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_LEFT) == GLFW_PRESS)
	{
		car.steer(3.f * dt);
	}
	else if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_RIGHT) == GLFW_PRESS)
	{
		car.steer(-3.f * dt);
	}
	else
	{
		car.steer(-car.steer_angle * 3.f * dt);
	}

	if (glfwGetKey(g::gfx::GLFW_WIN, GLFW_KEY_R) == GLFW_PRESS)
	{
		car.flip();
	}
}

} // namespace ld54