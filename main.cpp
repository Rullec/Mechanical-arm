// 2018/8/13 11:25 �ں���
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <ctime>
#include <EIGEN/Dense>
#include <GL/glut.h>
#define PI 3.1415926535

using namespace std;
using namespace Eigen;

class AJoint {
	// represent a joint
	public:
		AJoint()
		{
			Rot = MatrixXd::Identity(2, 2);
			Pos = VectorXd::Zero(2);
			Length = 1;
			Theta = 0;
		}
		int SetTheta(double theta)// set rotation theta [0,2*Pi]
		{
			Theta = theta;
			// calculate rotation matrix
			Rot = MatrixXd(2, 2);
			Rot(0, 0) = cos(theta);
			Rot(0, 1) = -1*sin(theta);
			Rot(1, 0) = sin(theta);
			Rot(1, 1) = cos(theta);

			return 0;
		}	
		int SetPos(VectorXd pos) // set links' position, passed in a VectorXd pos
		{
			Pos = pos;
			return 0;
		}
		int SetLength(double length)// set link's length, a double scalar
		{
			Length = length;
			return 0;
		}
		double GetTheta() // get the joint's theta angle
		{
			return Theta;
		}
		double GetLength()// query the length of links. return double scalar
		{
			return Length;
		}
		MatrixXd *GetRot() // get rotation matrix, return a pointer to Rot MatrixXd
		{
			return &Rot;
		}
		VectorXd *GetPos()// get the pointer to position vector
		{
			return &Pos;
		}

		
	private:
		MatrixXd Rot; // rotation matrix 2*2
		VectorXd Pos; // joint position 2*1
		double Length, Theta;
};
class JointChain {
	//represent joints' chain.
	public:
		JointChain(VectorXd Pos0, int Num, double *length)
		{
			
			JointNum = Num;
			chain = new AJoint[Num];
			Pos = new VectorXd[Num+1];
			
			// add: revise joints' states. 
			for (int i = 0; i<Num; i++)
			{
				Pos[i] = Pos0;
				chain[i].SetPos(Pos0);
				chain[i].SetTheta(0);
				chain[i].SetLength(length[i]);
				VectorXd tmp(2);
				tmp(0) = length[i] * cos(0);
				tmp(1) = length[i] * sin(0);
				Pos0 += tmp;
			}
			Pos[Num] = Pos0;
		}
		
		int SetTheta(int id, double theta)
		{
			VectorXd Pos0 = Pos[id];
			chain[id].SetTheta(theta);

			for (int i = id; i<JointNum; i++)
			{
				//cout << Pos0;
				Pos[i] = Pos0;
				chain[i].SetPos(Pos0);
				double length = chain[i].GetLength();
				VectorXd tmp(2);
				tmp(0) = length * cos(theta);
				tmp(1) = length * sin(theta);
				//cout << tmp;
				Pos0 += tmp;
				theta = 0; // just used once
			}
			Pos[JointNum] = Pos0;
			return 0;
		}
		double GetTheta(int i)
		{
			return chain[i].GetTheta();
		}
		VectorXd * GetState()
		{
			return Pos;
		}
		int GetNum()
		{
			return JointNum;
		}
		
	private:
		int JointNum = 0;
		AJoint *chain = NULL;
		VectorXd *Pos = NULL; // Pos���ڴ洢�������꣬����˵Ӧ����JointNum+1��Ԫ�ء�
};

void class_test();
void display(void);
void PaintCircle(double x, double y, double r);

int main(int argc, char** argv)
{
	srand((unsigned)time(NULL));
	
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);

	glutInitWindowPosition(100, 100);

	glutInitWindowSize(700, 700);

	glutCreateWindow("��һ��OpenGL����");

	glutDisplayFunc(display);//callback

	glutMainLoop();

	return 0;
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
	VectorXd InitPos(2);
	InitPos << 1, 2;
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

void display(void) {
	//�ص�����

	//Initialize
	int Num = 7;
	int scales = Num;//scaling 
	double *length = new double[Num];//links length array
	VectorXd *p = NULL;//used in paint

					   //set initial state of jointchains
	for (int i = 0; i < Num; i++) length[i] = 1;
	JointChain J(VectorXd::Zero(2), Num, length);

	//paint links and joints
	cout << "�ػ�" << endl;
	glClear(GL_COLOR_BUFFER_BIT);
	glBegin(GL_LINES);
	for (int i = 0; i < Num; i++)
	{
		int times = rand() % (180 - 0);
		J.SetTheta(i, times * PI / 180);//change ith link's theta angle
		p = J.GetState();
		cout << "theta[" << i << "]:" << J.GetTheta(i) / PI * 180 << "��C" << endl;

		//PaintCircle(p[i][0] / scales, p[i][1] / scales, length[i] / scales);
		glVertex2f(p[i][0] / scales, p[i][1] / scales);
		glVertex2f(p[i + 1][0] / scales, p[i + 1][1] / scales);
	}
	glEnd();
	glFlush();

	return;
}

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
/*
note:
1. ÿ����ͼ�ı�ith���������ʱ����i����Ľ������궼��Ҫ���£�������ðѡ����Ľ������ꡱ��һ������ߵ�JointChain��������
2. ������һ�λ���ʱ������Ҫ���ǣ���һ�����������꣬�ڶ������������꣬���������������꣬���ĸ�����������etc��
���˶��ѣ�������˵ά���⼸�������������Ҫ��Ŀ�ġ�
3. ��Ӧ�ö�AJoint������setpos��setlength��һ��Ĳ������������ƻ������������ԣ����ҷ��������Ҳ�ǲ�Ӧ�ó�����Щ�޸ĵġ�
4. �������������ٶȣ�
*/

