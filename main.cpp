// 2018/8/13 11:25 于杭州
#include <iostream>
#include <fstream>
#include <ctime>
#include <Windows.h>
#include <GL/glut.h>
#include "JointChain.h"
#define PI 3.1415926535

using namespace std;
using namespace Eigen;

void class_test();
void PaintCircle(double x, double y, double r);
void display(void);
void idle(void);

// Global varibles
int Num = 7;
JointChain *J = NULL;
double *length = NULL;
int theta = 0;
int alpha = 0;

void OnTimer(int value)
{
	alpha++;
	alpha = (alpha % 256);
	glutPostRedisplay();
	glutTimerFunc(16, OnTimer, 1);
}

int main(int argc, char** argv)
{
	
	srand((unsigned)time(NULL));
	
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	glutInitWindowPosition(100, 100);

	glutInitWindowSize(700, 700);

	glutCreateWindow("第一个OpenGL程序");

	glutDisplayFunc(display);//窗口刷新时才调用的回调函数

	//glutIdleFunc(idle);//系统空闲时调用的回调函数
	//glutSpecialFunc(); //响应键盘特殊按键
	//glutKeyboardFunc();//响应键盘普通按键
	glutTimerFunc(16, OnTimer, 1);
	glutMainLoop();
	
	//class_test();
	int a;
	cin >> a;
	return 0;
}

void display(void)
{
	//重绘回调函数
	

	//Initialize
	int scales = Num;				//scaling 
	length = new double[Num];		//links length array
	VectorXd *p = NULL;				//used in paint

	//set initial state of jointchains
	for (int i = 0; i < Num; i++) length[i] = 1;
	VectorXd tmp = VectorXd::Zero(3);
	tmp[2] = 1;
	if (!J) J = new JointChain(tmp, Num, length);

	//paint links and joints
	cout << "重绘" << endl;
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glPointSize(5.0f);
	glLineWidth(3.0f);				//set line width
	glColor3f(0.0f, 0.2f, 0.0f);	//set line color 
	glBegin(GL_LINES);
	
	//J->SetTheta(0, theta * PI / 180);//change ith link's theta angle
	J->SetTheta(2, theta * PI / 180);//change ith link's theta angle
	//J->SetTheta(4, theta * PI / 180);//change ith link's theta angle
	//J->SetTheta(6, PI - theta * PI / 180);//change ith link's theta angle
	//J->SetTheta(3, theta * PI / 180);//change ith link's theta angle


	theta = theta + 1;
	for (int i = 0; i < Num; i++)
	{
		p = J->GetState();
		cout << "theta[" << i << "]:" << J->GetTheta(i) / PI * 180 << "°C" << endl;
		cout << "p["<<i<<"]:" << endl << p[i] << endl;
		
		//PaintCircle(p[i][0] / scales, p[i][1] / scales, length[i] / scales);
		glVertex2f(p[i][0] / scales, p[i][1] / scales);
		glVertex2f(p[i + 1][0] / scales, p[i + 1][1] / scales);
	}

	glEnd();
	glutSwapBuffers();
	/*
	for (int i = 0; i < Num; i++)
	{
		int times = rand() % (180 - 0);
		J->SetTheta(i, times * PI / 180);//change ith link's theta angle
		p = J->GetState();
		cout << "theta[" << i << "]:" << J.GetTheta(i) / PI * 180 << "°C" << endl;

		//PaintCircle(p[i][0] / scales, p[i][1] / scales, length[i] / scales);
		glVertex2f(p[i][0] / scales, p[i][1] / scales);
		glVertex2f(p[i + 1][0] / scales, p[i + 1][1] / scales);
	}
	*/
	return;
}

/*
void idle(void){
	//空闲回调函数
	theta++;
	if (theta >= 360) theta = 0;
	
	//Initialize
	int scales = Num;				//scaling 
	length = new double[Num];		//links length array
	VectorXd *p = NULL;				//used in paint

	//set initial state of jointchains
	for (int i = 0; i < Num; i++) length[i] = 1;
	VectorXd tmp = VectorXd::Zero(3);
	tmp[2] = 1;
	if(!J) J = new JointChain(tmp, Num, length);

	//paint links and joints
	cout << "重绘" << endl;
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glPointSize(5.0f);
	glLineWidth(3.0f);				//set line width
	glColor3f(0.0f, 0.2f, 0.0f);	//set line color 
	glBegin(GL_LINES);


	glEnd();
	glFlush();
	return;
}
*/

void PaintCircle(double x, double y, double r)
{
	//paint circle
	int n = 10;
	glBegin(GL_POLYGON);
	for (int i = 0; i<n; i++)
	{
		glVertex2f(x + r * cos(2 * PI*i / n), y + r * sin(2 * PI*i / n));
	}
	glEnd();
}

void class_test()
{
	cout << "AJoint class test begin...";
	// uniy test Ajoint class
	AJoint *p;
	p = new AJoint();
	cout << "length:" << p->GetLength() << endl;
	cout << "position:" << *(p->GetPos()) << endl;
	cout << "rot matrix:" << *(p->GetRot()) << endl;
	cout << "theta angle:" << p->GetTheta() << endl;


	cout << "JointChain class test begin...";
	// unit test JointChain class
	double Len[4] = { 1, 1, 1, 1 };
	VectorXd InitPos(3);
	InitPos << 1, 2, 1;
	JointChain J(InitPos, 4, Len);

	int num = J.GetNum();
	VectorXd *pos = J.GetState();
	for (int i = 0; i < num; i++)
	{
		cout << "J[" << i << "] state:" << endl << pos[i] << endl;
	}
	cout << "*******" << endl;
	J.SetTheta(1, PI / 2);
	J.SetTheta(2, PI / 6);
	for (int i = 0; i < num; i++)
	{
		cout << "J[" << i << "] state:" << endl << pos[i] << endl;
	}

	return;
}