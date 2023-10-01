in vec3 v_normal;
in vec2 v_uv;
in vec4 v_world_pos;

out vec4 color;



void main (void)
{
	float ndl = max(0, -dot(v_normal, normalize(vec3(0.0, -1.0, 1.0))));
	color = vec4(vec3(ndl) + 0.125, 1.0);
	// color = vec4(v_normal * 0.5 + vec3(0.5), 1.0);
}
