// 2018/8/13 11:25 于杭州
#include <fstream>
#include "shader.h"
#include "JointChain.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Paint.h"
//#include <openMesh/Core/IO/MeshIO.hh>
#define PI 3.1415926535
using namespace std;

// Function declare
// -------------------------------------------------
GLFWwindow* GL_Init();
JointChain * JointInit();
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
int processInput(GLFWwindow *window);
//GLfloat * DrawSphere( double radius);
double radians(double angle);
void class_test();

// Global varibles
// --------------------------------------------------
const int SCR_WIDTH = 800;
const int SCR_HEIGHT = 600;
const int JointNum = 7;
const int SLICE_Z = 20, SLICE_XY = 20;
float * JointLength = NULL;
float Yaw = 0;
float Pitch = 0;
float deltaTime = 0.0f;
float lastFrame = 0.0f;
float lastX = 400, lastY = 300;
VectorXd cameraPos = VectorXd::Zero(3);
VectorXd cameraFront = VectorXd::Zero(3);
VectorXd cameraUp = VectorXd::Zero(3);
GLfloat sphere[SLICE_Z * SLICE_XY * 3] = {};
GLfloat sphereIndices;

int main(int argc, char** argv)
{
	//class_test();

	// initialize OPENGL
	GLFWwindow * window = GL_Init();
	if (!window)
	{
		cout << "ERROR:GL_Init failed." << endl;
	}

	// calculate sphere


	// initialize Shader
	Shader OurShader = Shader("./shader.vs", "./shader.fs");

	// initialize JointChain
	JointChain *P = JointInit();
	if (!P)
	{
		cout << "ERROR:JointInit Failed!" << endl;
		return -1;
	}

	// MainLoop
	while (!glfwWindowShouldClose(window))
	{
		// process input & mouse 
		processInput(window);

		// clear buffer & depth test
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render
		cout << "render." << endl;

		// swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	return 0;
}

GLFWwindow * GL_Init()
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// glfw initialize window
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return NULL;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return NULL;
	}

	return window;
}

JointChain * JointInit()
{
	JointChain * p = NULL;
	VectorXd InitPos = VectorXd::Zero(4);

	// initialize JointLength
	JointLength = new float[JointNum];
	for (int i = 0; i < JointNum; i++) JointLength[i] = 1;

	// create new jointchain
	InitPos[3] = 1;
	p = new JointChain(InitPos, JointNum, (double *)JointLength);

	return p;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{//鼠标回调函数
 // 计算当前位置和现在位置的差，改变两个角度，然后计算方向向量。
	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;//y坐标从底到顶依次增大。
	lastX = xpos;
	lastY = ypos;
	float sensitivity = 0.05f;
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	Yaw += xoffset;
	Pitch += yoffset;
	Pitch = Pitch > 89.0f ? 89.0f : Pitch;
	Pitch = Pitch < -89.0f ? -89.0f : Pitch;

	// calculate front
	/*
	VectorXd front = VectorXd(0, 0, 0);
	front.x = cos(radians(Yaw))*cos(radians(Pitch));
	front.y = sin(radians(Pitch));
	front.z = sin(radians(Yaw))*cos(radians(Pitch));
	cameraFront = front;
	*/
}

int processInput(GLFWwindow *window)
{
	float cameraSpeed = deltaTime * 2.5f; // adjust accordingly
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(window, true);
		return 0;
	}
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)	return GLFW_KEY_UP;
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)	return GLFW_KEY_DOWN;
	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)	return GLFW_KEY_RIGHT;
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)	return GLFW_KEY_LEFT;

	return 0;
}

double radians(double angle)
{
	return angle / 180 * PI;
}

/*
GLfloat * DrawSphere(double radius)
{
float step_z = PI / SLICE_Z;
float step_xy = 2 * PI / SLICE_XY;

for (int i = 0; i < SLICE_Z; i++)
{
for (int j = 0; j < SLICE_XY; j++)
{
GLfloat thetaZ = PI / 2 - step_z * i;
GLfloat thetaxy = j * step_xy;

sphere[i*j + 0] = cos(thetaZ) * cos(thetaxy) * radius;
sphere[i*j + 1] = cos(thetaZ) * sin(thetaxy) * radius;
sphere[i*j + 2] = sin(thetaZ) * radius;
}
}
for (int i = 0; i < SLICE_Z - 1; i++)
{
for (int j = 0; j < SLICE_XY; j++)
{
sphereIndices[i*j + 0] = i * j + 0;
sphereIndices[i*j + 1] = i * j + 1;
sphereIndices[i*j + 2] = (i + 1) * j + 0;
sphereIndices[i*j + 3] = (i + 1) * j + 1;


}
}
}
*/
void class_test()
{
	cout << "AJoint class test begin..." << endl;
	// uniy test Ajoint class
	AJoint *p;
	p = new AJoint();
	cout << "length:" << p->GetLength() << endl;
	cout << "position:" << endl << *(p->GetPos()) << endl;
	cout << "rot matrix:" << endl << *(p->GetRot()) << endl;
	cout << "rotx matrix:" << endl << *(p->GetRot(0)) << endl;
	cout << "roty matrix:" << endl << *(p->GetRot(1)) << endl;
	cout << "rotz matrix:" << endl << *(p->GetRot(2)) << endl;
	cout << "x axis theta:" << p->GetTheta()[0] * 180 << endl;
	cout << "y axis theta:" << p->GetTheta()[1] * 180 << endl;
	cout << "z axis theta:" << p->GetTheta()[2] * 180 << endl;
	cout << "********************************" << endl;
	p->SetTheta(0, PI / 6);
	p->SetTheta(1, PI / 3);
	p->SetTheta(2, PI / 2);
	cout << "rot matrix:" << endl << *(p->GetRot()) << endl;
	cout << "rotx matrix:" << endl << *(p->GetRot(0)) << endl;
	cout << "roty matrix:" << endl << *(p->GetRot(1)) << endl;
	cout << "rotz matrix:" << endl << *(p->GetRot(2)) << endl;
	cout << "x axis theta:" << p->GetTheta()[0] * 180 / PI << endl;
	cout << "y axis theta:" << p->GetTheta()[1] * 180 / PI << endl;
	cout << "z axis theta:" << p->GetTheta()[2] * 180 / PI << endl;

	cout << "JointChain class test begin...";
	// unit test JointChain class
	double Len[4] = { 1, 1, 1, 1 };
	VectorXd InitPos(4);
	InitPos << 0, 0, 0, 1;
	JointChain J(InitPos, 4, Len);

	int num = J.GetNum();
	VectorXd *pos = J.GetState();
	for (int i = 0; i < num; i++)
	{
		cout << "J[" << i << "] state:" << endl << pos[i] << endl;
	}
	cout << "******************************************************" << endl;
	//J.SetTheta(1, 0, PI / 2); 不要把一个和x轴重合的向量绕x轴是旋转
	J.SetTheta(2, 1, PI / 6);
	double *q = J.GetTheta(1);
	for (int i = 0; i < 3; i++) cout << "link " << 1 << "'s theta[" << i << "]:" << q[i] << endl;
	for (int i = 0; i < num; i++)
	{
		cout << "J[" << i << "] state:" << endl << pos[i] << endl;
	}

	return;
}