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

uniform Material material;//����
uniform vec3 cameraPos;
uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor; // obj color

void main()
{
	// ��������
	vec3 ambient  = lightColor * material.ambient;

	// ������
	vec3 lightDir = normalize(lightPos - FragPos );//����
	vec3 nor = normalize(Normal);//����_���Ǳ�׼���ǳ�������
	float diff = max(dot(lightDir, nor), 0);
	vec3 diffuse =  (diff * material.diffuse ) * lightColor; // ���������Ĺ�ǿ

	// ���淴��
	float specularStrength = 0.5; //���淴�������
	vec3 cameraDir = normalize(cameraPos - FragPos); // ĳ����������
	vec3 reflectDir = reflect(-lightDir, nor);//����ⷽ��ע��APIҪ���һ�������Ǵӹ�Դָ��Ƭ��λ�ã���Ҫȡ����
	float spec = pow(max(dot(cameraDir, reflectDir), 0), material.shininess);//���㷴�������ߵĵ������֤Ϊ�����õ����淴��ǿ��
	vec3 specular = (spec * material.specular )* lightColor;
	//32�Ǹ߹�ķ���ȣ������Խ�ߣ�����������Խǿ��ɢ���Խ�٣��߹��ԽС��

	vec3 result = (ambient + diffuse + specular) * objectColor;

	FragColor = vec4(result, 1.0f);

}