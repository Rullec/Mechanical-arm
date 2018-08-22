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
    //float ambientStrength = 0.5;
    //vec3 ambient = ambientStrength * lightColor;
  	FragColor = vec4(0.6f, 0.5f, 0.3f, 0.5f);
    // diffuse 

    //vec3 norm = normalize(Normal);
    //vec3 lightDir = normalize(lightPos - FragPos);
    //float diff = max(dot(norm, lightDir), 0.0);
    //vec3 diffuse = diff * lightColor;
            
    //vec3 result = (ambient + diffuse) * objectColor;
    //FragColor = vec4(result, 1.0);
	
} 