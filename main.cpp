// 2018/8/13 11:25 �ں���
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
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

// Function declare
// -------------------------------------------------
GLFWwindow* GL_Init();
JointChain * JointInit();
//GLfloat * DrawSphere( double radius);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
int processInput(GLFWwindow *window);
float radian_news(float angle);
void class_test();
void DrawLinks();

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
vec3 cameraPos = vec3(0.0, 0.0, 0.0); // λ��
vec3 cameraFront = vec3(0.0f, 0.0f, -1.0f);	//���������
vec3 cameraUp = vec3(0.0f, 1.0f, 0.0f);//����

GLfloat sphere[72000];
GLfloat cylinder[100000];
GLfloat axises[18] = {
0.0f, 0.0f, 0.0f,
3.0f, 0.0f, 0.0f,
0.0f, 0.0f, 0.0f,
0.0f, 3.0f, 0.0f,
0.0f, 0.0f, 0.0f,
0.0f, 0.0f, 3.0f
};
int main(int argc, char** argv)
{
	//class_test();

	// initialize OPENGL
	GLFWwindow * window = GL_Init();
	if (!window)
	{
		cout << "ERROR:GL_Init failed." << endl;
	}

	// read ball mesh 
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
	}

	// calculate links vertex
	DrawLinks();

	// initialize Shader
	Shader OurShader1 = Shader("./shader_ball.vs", "./shader_ball.fs"),\
		OurShader2 = Shader("./shader_cylinder.vs", "./shader_cylinder.fs"),\
		OurShader3 = Shader("./shader_axis.vs", "./shader_axis.fs");



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
	unsigned int VBO[3], VAO[3];//0-joints 1-links
	glGenVertexArrays(3, VAO);
	glGenBuffers(3, VBO);

	glBindVertexArray(VAO[0]);// bind joints
	glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(sphere), sphere, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
	glEnableVertexAttribArray(0);
	
	glBindVertexArray(VAO[1]); // bind links
	glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cylinder), cylinder, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
	glEnableVertexAttribArray(1);

	glBindVertexArray(VAO[2]); // bind axises
	glBindBuffer(GL_ARRAY_BUFFER, VBO[2]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(axises), axises, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
	glEnableVertexAttribArray(2);

	// set view
	float screenWidth = 800, screenHeight = 600;
	mat4 model, view, projection;

	model = rotate(model, radian_news(0.0f), vec3(0.0f, 0.0f, 1.0f));// model matrix
	view = translate(view, vec3(0.0f, 0.0f, -1.0f));//�۲����
	projection = perspective(radian_news(45.0f), screenWidth / screenHeight, 0.1f, 100.0f);//ͶӰ����

	OurShader1.use();
	OurShader1.setMat4("view", view);
	OurShader1.setMat4("projection", projection);
	OurShader1.setMat4("model", model);

	OurShader2.use();
	OurShader2.setMat4("view", view);
	OurShader2.setMat4("projection", projection);
	OurShader2.setMat4("model", model);
	
	OurShader3.use();
	OurShader3.setMat4("view", view);
	OurShader3.setMat4("projection", projection);
	OurShader3.setMat4("model", model);

	// MainLoop
	while (!glfwWindowShouldClose(window))
	{
		// process input & mouse 
		processInput(window);

		view = lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		OurShader1.use();
		OurShader1.setMat4("view", view);
		OurShader2.use();
		OurShader2.setMat4("view", view);
		OurShader3.use();
		OurShader3.setMat4("view", view);

		// clear buffer & depth test
		glEnable(GL_DEPTH_TEST);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render
		//cout << "render." << endl;
		OurShader1.use();

		// Draw joints
		float theta = glfwGetTime();
		J->SetTheta(1, 1, theta);
		J->SetTheta(0, 1, theta);
		J->SetTheta(2, 2, theta/2);
		J->SetTheta(3, 1, theta/2);

		glBindVertexArray(VAO[0]);
		VectorXd * pos = J->GetState();//opengl�ֲ�����ϵ������ϵ/����Y�����������Ƕ������ʱ��Z�����为���ҵ�Joint���ǰ�������ϵд�ġ������ˡ�
		for (int i = 0; i < JointNum; i++)
		{
			vec3 tmp;
			tmp.x = pos[i][0];
			tmp.y = pos[i][1];
			tmp.z = pos[i][2];
			//cout << pos[i] << endl;
			mat4 model;
			model = translate(model, tmp);
			
			OurShader1.setMat4("model", model);
			glDrawArrays(GL_TRIANGLES, 0, 72000);
		}

		// Draw links
		OurShader2.use();
		glBindVertexArray(VAO[1]);
		for (int i = 0; i < JointNum; i++)
		{
			vec3 tmp;
			tmp.x = pos[i][0];
			tmp.y = pos[i][1];
			tmp.z = pos[i][2];
			cout << pos[i] << endl;
			mat4 model;
			model = translate(model, tmp);
			
			double * theta = J->GetTheta(i);//ȡ�����ǻ�����
			model = rotate(model, float(theta[0]), vec3(1.0, 0.0, 0.0));// A
			model = rotate(model, float(theta[1]), vec3(0.0, 1.0, 0.0));// B
			model = rotate(model, float(theta[2]), vec3(0.0, 0.0, 1.0)); //C

			model = rotate(model, radian_news(90.0f), vec3(0.0, 1.0, 0.0));// D ת��90�ȣ�û��
			OurShader2.setMat4("model", model);

			glDrawArrays(GL_TRIANGLES, 0, 99998);
		}
		
		// Draw axises
		OurShader3.use();
		glBindVertexArray(VAO[2]);

		for (int i = 0; i < JointNum; i++)
		{
			OurShader3.setInt("axis", i);
			glDrawArrays(GL_LINES, 2*i, 2);
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
{//���ص�����
 // ���㵱ǰλ�ú�����λ�õĲ�ı������Ƕȣ�Ȼ����㷽��������
	
	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;//y����ӵ׵�����������
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
	front.x = cos(radian_news(Yaw))*cos(radian_news(Pitch));
	front.y = sin(radian_news(Pitch));
	front.z = sin(radian_news(Yaw))*cos(radian_news(Pitch));
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
		cameraPos += cameraSpeed * cameraFront; // ���������ָ����ǰ�ߡ�
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	return 0;
}

float radian_news(float angle)
{
	return angle / 180 * PI;
}

void DrawLinks()
{
	double * circle = NULL;
	double radius = 0.1;
	double length = 1;
	int circle_slice = 50, z_slice = 100;

	circle = new double[circle_slice * 2];
	for (int i = 0; i < circle_slice; i++)
	{
		circle[2 * i + 0] = cos(1.0 * i / circle_slice * 2 * PI) * radius;
		circle[2 * i + 1] = sin(1.0 * i / circle_slice * 2 * PI) * radius;
		//cout << circle[2 * i] << " " << circle[2 * i + 1] << endl;
	}
	for (int z = 0; z < z_slice; z++)
	{
		for (int i = 0; i < circle_slice; i++)
		{
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 0 * 3 + 0] = circle[2 * i + 0];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 0 * 3 + 1] = circle[2 * i + 1];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 0 * 3 + 2] = z / z_slice * length;
				
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 1 * 3 + 0] = circle[2 * i + 0];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 1 * 3 + 1] = circle[2 * i + 1];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 1 * 3 + 2] = (z+1) / z_slice * length;

			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 2 * 3 + 0] = circle[2 * (1 + i) + 0];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 2 * 3 + 1] = circle[2 * (1 + i) + 1];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 2 * 3 + 2] = z / z_slice * length;

			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 3 * 3 + 0] = circle[2 * i + 0];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 3 * 3 + 1] = circle[2 * i + 1];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 3 * 3 + 2] = (z + 1) / z_slice * length;

			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 4 * 3 + 0] = circle[2 * (1 + i) + 0];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 4 * 3 + 1] = circle[2 * (1 + i) + 1];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 4 * 3 + 2] = (z + 1) / z_slice * length;

			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 5 * 3 + 0] = circle[2 * (1 + i) + 0];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 5 * 3 + 1] = circle[2 * (1 + i) + 1];
			cylinder[z * (circle_slice * 6 * 3) + i * 6 * 3 + 5 * 3 + 2] = z / z_slice * length;

		}
	}
}
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
	//J.SetTheta(1, 0, PI / 2); ��Ҫ��һ����x���غϵ�������x������ת
	J.SetTheta(2, 1, PI / 6);
	double *q = J.GetTheta(1);
	for (int i = 0; i < 3; i++) cout << "link " << 1 << "'s theta[" << i << "]:" << q[i] << endl;
	for (int i = 0; i < num; i++)
	{
		cout << "J[" << i << "] state:" << endl << pos[i] << endl;
	}

	return;
}