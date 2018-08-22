#version 330 core

out vec4 FragColor;

uniform int axis;

void main()
{
	vec3 axisColor;
	switch(axis)
	{
		case 0: axisColor = vec3(1.0, 0.0, 0.0); break;
		case 1: axisColor = vec3(0.0, 1.0, 0.0); break;
		case 2: axisColor = vec3(0.0, 0.1, 0.9); break;
		default:break;
	}
	FragColor = vec4(axisColor, 1);
	//FragColor = mix(texture(texture1, TexCoord), texture(texture2, vec2(1.0 - TexCoord.x, TexCoord.y)), 0.2);
};