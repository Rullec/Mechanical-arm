#version 330 core
layout (location = 0) in vec3 aPos;

out vec3 Normal;//output this point's normal vector
out vec3 FragPos;//nowday point position.

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	vec3 aNormal = normalize(aPos);
	gl_Position = projection * view * model * vec4(aPos, 1.0f);
	Normal = mat3(transpose(inverse(model))) * aNormal;
	FragPos = vec3(model * vec4(aPos, 1.0));
}