#ifdef GL_ES
precision mediump float;
#endif

in vec3 a_position;
in vec2 a_uv;
in vec3 a_normal;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

out vec3 v_normal;
out vec2 v_uv;
out vec4 v_world_pos;

void main (void)
{
   v_world_pos = u_model * vec4(a_position, 1.0);
   v_normal = a_normal;
   v_uv = a_uv;
   gl_Position = u_proj * u_view * v_world_pos;
}
