// 2018/8/13 11:25 于杭州
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <fstream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "shader.h"
#include "JointChain.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define PI 3.1415926535
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

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
const float SCR_WIDTH = 800;
const float SCR_HEIGHT = 600;
const int JointNum = 7;
const int SLICE_Z = 20, SLICE_XY = 20;
float * JointLength = NULL;
float Yaw = 0;
float Pitch = 0;
float deltaTime = 0.0f;
float lastFrame = 0.0f;
float lastX = 400, lastY = 300;
vec3 cameraPos = vec3(0.0, 0.0, 0.0); // 位置
vec3 cameraFront = vec3(0.0f, 0.0f, -1.0f);	//摄像机面向
vec3 cameraUp = vec3(0.0f, 1.0f, 0.0f);//上轴

GLfloat sphere[72000];

int main(int argc, char** argv)
{
	//class_test();

	// initialize OPENGL
	GLFWwindow * window = GL_Init();
	if (!window)
	{
		cout << "ERROR:GL_Init failed." << endl;
	}

	// read model mesh 
	MyMesh mesh;
	int i = 0, j = 0;
	if (!OpenMesh::IO::read_mesh(mesh, "ball.obj"))
	{
		std::cerr << "read error\n";
		exit(1);
	}
	
	memset(sphere, 0, sizeof(GLfloat) * 72000);
	for (MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it, ++j) {
		i = 0;
		for (MyMesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it, ++i)
		{
			float * tmp = mesh.point(*fv_it).data();
			sphere[3 * (j * 3 + i) + 0] = tmp[0] / 50 - 0.2;
			sphere[3 * (j * 3 + i) + 1] = tmp[1] / 50 - 0.2;
			sphere[3 * (j * 3 + i) + 2] = tmp[2] / 50 - 0.2;
		}
		if( j == mesh.n_faces()) 
			cout << j << endl;
	}

	// initialize Shader
	Shader OurShader = Shader("./shader.vs", "./shader.fs");
	OurShader.use();

	// initialize JointChain
	VectorXd initPos = VectorXd::Zero(4);
	initPos[3] = 1;
	double *length = new double[JointNum];
	for (int i = 0; i < JointNum; i++) length[i] = 1;
	JointChain *J = new JointChain(initPos, JointNum, length);
	if (!J)
	{
		cout << "ERROR:JointInit Failed!" << endl;
		return -1;
	}

	// initilize VAO & VBO
	unsigned int VBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(sphere), sphere, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
	glEnableVertexAttribArray(0);

	// set view
	float screenWidth = 800, screenHeight = 600;
	mat4 model, view, projection;
	model = rotate(model, radians(0.0f), vec3(0.0f, 0.0f, 1.0f));// model matrix
	view = translate(view, vec3(0.0f, 0.0f, -1.0f));//观察矩阵
	projection = perspective(radians(45.0f), screenWidth / screenHeight, 0.1f, 100.0f);//投影矩阵
	OurShader.setMat4("view", view);
	OurShader.setMat4("projection", projection);
	OurShader.setMat4("model", model);

	// MainLoop
	while (!glfwWindowShouldClose(window))
	{
		// process input & mouse 
		processInput(window);

		view = lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		OurShader.setMat4("view", view);

		// clear buffer & depth test
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render
		//cout << "render." << endl;
		OurShader.use();
		glBindVertexArray(VAO);
		VectorXd * pos = J->GetState();
		J->SetTheta(1, 2, (float)glfwGetTime());
		//J->SetTheta(1, 1, (float)glfwGetTime());
		//J->SetTheta(3, 2, (float)glfwGetTime());

		for (int i = 0; i < JointNum; i++)
		{
			vec3 tmp;
			tmp.x = pos[i][0];
			tmp.y = pos[i][1];
			tmp.z = pos[i][2];
			cout << pos[i] << endl;
			mat4 model;
			model = translate(model, tmp);
			OurShader.setMat4("model", model);
			glDrawArrays(GL_TRIANGLES, 0, 72000);
		}
		
		
		// swap buffer
		glfwSwapBuffers(window);
		glfwPollEvents();

		// set Frame time & speed
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
	}

	// glfw: terminate, clearing all previously allocated GLFW resources.
	// ------------------------------------------------------------------
	glfwTerminate();
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
	vec3 front;
	front.x = cos(radians(Yaw))*cos(radians(Pitch));
	front.y = sin(radians(Pitch));
	front.z = sin(radians(Yaw))*cos(radians(Pitch));
	cameraFront = front;
	
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
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront; // 沿着摄像机指向往前走。
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
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