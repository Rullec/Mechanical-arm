#pragma once
#include <string>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
using namespace glm;

enum Camera_Movement {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
};

class Camera {
public:
	Camera()
	{
		 cameraPos = vec3(1.0, 2.0, 2.0); // 位置
		 cameraFront= vec3(0.0f, -1.0f, -1.0f);	//摄像机面向
		 cameraUp = vec3(0.0f, 1.0f, -1.0f);//上轴
		 Yaw = Pitch = 0;
		 cameraSpeed = 2.5f;
		 lastX = 400;
		 lastY = 300;
	}
	float GetSpeed()
	{
		return cameraSpeed;
	}
	vec3 & GetPos()
	{
		return cameraPos;
	}
	vec3 & GetFront()
	{
		return cameraFront;
	}
	vec3 & GetUp()
	{
		return cameraUp;
	}
	void SetSpeed(float deltaTime)
	{
		cameraSpeed = deltaTime * 2.5f;
	}
	void SetMouse(double xpos, double ypos)
	{
		float xoffset = xpos - lastX;
		float yoffset = lastY - ypos;//y坐标从底到顶依次增大。
		float sensitivity = 0.05f;

		lastX = xpos;
		lastY = ypos;
		xoffset *= sensitivity;
		yoffset *= sensitivity;

		Yaw += xoffset;
		Pitch += yoffset;
		Pitch = Pitch > 89.0f ? 89.0f : Pitch;
		Pitch = Pitch < -89.0f ? -89.0f : Pitch;

		cameraFront.x = cos(radians(Yaw))*cos(radians(Pitch));
		cameraFront.y = sin(radians(Pitch));
		cameraFront.z = sin(radians(Yaw))*cos(radians(Pitch));
	}
	void ChangePos(Camera_Movement op)
	{// used for ordinary FPS control
		if (FORWARD == op)
		{
			cameraPos += cameraSpeed * cameraFront;
		}
		else if (BACKWARD == op)
		{
			cameraPos -= cameraSpeed * cameraFront;
		}
		else if (LEFT == op)
		{
			cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
		}
		else if (RIGHT == op)
		{
			cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
		}
	}
	mat4 SetPos(vec3 pos, vec3 front, vec3 up)
	{
		cameraPos = pos;
		cameraFront = front;
		cameraUp = up;
		Yaw = 0;
		Pitch = 0;
		lastX = 400;
		lastY = 300;
		return lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	}
private:
	vec3 cameraPos;
	vec3 cameraFront;
	vec3 cameraUp;
	float Yaw;
	float Pitch;
	float cameraSpeed;
	float lastX;
	float lastY;

};