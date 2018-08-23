#version 330 core

struct Material{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};

out vec4 FragColor;

// it is the shader for objs, so we must have lightcolor and obj color, otherwise we can't calculate the lightning.
in vec3 Normal;
in vec3 FragPos;

uniform Material material;//材质
uniform vec3 cameraPos;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor; // obj color

void main()
{
	// 环境光照
	vec3 ambient  = lightColor * material.ambient;

	// 漫反射
	vec3 lightDir = normalize(lightPos - FragPos );//光线
	vec3 nor = normalize(Normal);//法向_忘记标准化是常见错误
	float diff = max(dot(lightDir, nor), 0);
	vec3 diffuse =  (diff * material.diffuse ) * lightColor; // 获得漫反射的光强

	// 镜面反射
	float specularStrength = 0.5; //镜面反射的力度
	vec3 cameraDir = normalize(cameraPos - FragPos); // 某点的相机防线
	vec3 reflectDir = reflect(-lightDir, nor);//出射光方向，注意API要求第一个向量是从光源指向片段位置，需要取反。
	float spec = pow(max(dot(cameraDir, reflectDir), 0), material.shininess);//计算反射光和视线的点积，保证为正，得到镜面反射强度
	vec3 specular = (spec * material.specular )* lightColor;
	//32是高光的反光度，反光度越高，反射光的能力越强，散射就越少，高光点越小。

	vec3 result = (ambient + diffuse + specular) * objectColor;

	FragColor = vec4(result, 1.0f);

}