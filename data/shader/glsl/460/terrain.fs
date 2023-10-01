in vec3 v_normal;
in vec3 v_tangent;
in vec3 v_up;
in vec4 v_world_pos;
in mat3 v_basis;

out vec4 color;

void main (void)
{
	color = vec4(v_normal * 0.5 + vec3(0.5), 1.0);
}
