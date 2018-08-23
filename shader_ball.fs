#version 330 core
out vec4 FragColor;

in vec3 Normal;  
in vec3 FragPos;  
  
uniform vec3 lightPos; 
uniform vec3 lightColor;
uniform vec3 objectColor;

void main()
{
    // ambient
    float ambientStrength = 0.5;
    vec3 ambient = ambientStrength * lightColor;
  	
    // diffuse 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
            
    vec3 result = (ambient + diffuse) * objectColor;
	result.x = (ambient.x + diffuse.x) * objectColor.x;
	result.y = (ambient.y + diffuse.y) * objectColor.y;
	result.z = (ambient.z + diffuse.z) * objectColor.z;

    //FragColor = vec4(result,  1.0);
	FragColor = vec4(vec3(255.0/255,222.0/255,173.0/255),  1.0);
} 