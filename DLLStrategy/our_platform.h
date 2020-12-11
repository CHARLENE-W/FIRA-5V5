#pragma once
#include <cstdint>
#include "platform.h"

///////////////////////////////////////////////////////////////////////
/////////////////////////////////���ݽṹ//////////////////////////////
///////////////////////////////////////////////////////////////////////

using namespace Simuro;
typedef struct
{
	double x, y, z;
} Vector3D;

typedef struct
{
	int x, y;
} Block;

typedef struct
{
	long left, right, top, bottom;
} Bounds;



struct MyRobot
{
	Vector3D position;
	float rotation;
	Wheel wheel;
};//==Robot

typedef struct
{
	Vector3D my_old_pos[5];//�ҷ���Ա�ϴε�����
	Vector3D my_speed[5];//�ҷ���Ա���ٶ�
	Vector3D my_old_velocity[5];//�ҷ���Ա�ϴε�����

	Vector3D op_old_pos[5];//�Է���Ա�ϴε�����
	Vector3D op_speed[5];//�Է���Ա���ٶ�

	MyRobot robot[5];//�ҷ���Ա 
	MyRobot opp[5];//�Է���Ա                //Ψһ�ĵĵط�

	Vector3D ball_old;//���ϴε�����
	Vector3D ball_cur;//�����ڵ�����
	Vector3D ball_pre;//��Ԥ�������
	Vector3D ball_speed;//���ٶȵ�����


	long time[2];//time[1]ȡ������//time[0]ȡ������
	bool mygrand;//�� = �ƶ�//�� = ����
	bool locked;//�Ƿ��Ѿ��жϳ���	
	int WIB;//����

	Vector3D block[100][100];//��ǰ�����������
	int block_my[100][100];//�ҷ���Ա����
	int block_op[100][100];//�Է���Ա����
	int block_ball[100][100];//�����
	Block block_min;//��С��xֵ
	Block block_max;//����xֵ
	Block my_block_pos[5];//�ҷ���Ա�Ŀ����� //-1 = ����դ�������� //ֵ >= 0 = ����
	Block op_block_pos[5];//�Է���Ա�Ŀ����� //-1 = ����դ�������� //ֵ >= 0 = ����
	Block ball_block_pos;//��Ŀ�����

	int bgoalball;//������
	int nfreeball;//������
	int nplaceball;
	int npenaltyball;//��Է���
	int chooserobot;//ѡ������Ա

	int ActiveAttacker;//��Ա
	int NegativeAttacker;
	int Attacker;
	int Defender;
	int Keeper;

	long gameState;	//0,1,2,3,4,5
	long whoseBall; //0,1,2

	long n;
	bool B_N;				//these two veriaty is for the test funtion!
	//	Bounds field, goal;

	bool debug;//�Ƿ��ǵ���


}Mydata;


///////////////////////////////////////////////////////////////////////
///////////////////////////////////����////////////////////////////////
///////////////////////////////////////////////////////////////////////



/*****************************��������***********************************/
//�򳡷�Χ
//const double FTOP = 90;
//const double FBOT = -90;
//const double GTOP = 20;
//const double GBOT = -20;
//const double GRIGHT = 125;
//const double GLEFT = -125;
//const double FRIGHTX = 110;
//const double FLEFTX = -110;

const double FTOP = 180;
const double FBOT = 0;
const double GTOP = 110;
const double GBOT = 70;
const double GRIGHT = 125 + 110;
const double GLEFT = -125 + 110;
const double FRIGHTX = 110 +110;
const double FLEFTX = -110 + 110;


/**********************************��������***************************/

const long   CAR = 8;	                            //С���߳� 2.95						
const double ROBOTWITH = 3.14 * 2.54;
const double BALLWITH = 1.5 * 2.54;




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++��û��
//������

const double BD_TOP = 110;                 // �����ϱ߽�
const double BD_BOT = 70;                 // �����±߽�
/*��볡*/const double BD_LEFT = -15;       // ������߽�//
/*�Ұ볡*/const double BD_RIGHT = 235;      // �����ұ߽�//

//С����
const double SRG_TOP = 115;                      //С�����ϱ߽� 
const double SRG_BOT = 75;                      //С�����±߽�
/*��볡*/const double SRG_LEFT = 7.5;           //С��������
/*�Ұ볡*/const double SRG_RIGHT = 212.5;          //С��������

//������С����

//�����
const double BRG_TOP = 130;                      //�������  
const double BRG_BOT = 50;                      //������� 
/*��볡*/const double BRG_LEFT = 37.5;           //������� 
/*�Ұ볡*/const double BRG_RIGHT = 182.5;         //�������


//����
const double GBLEFT = -15;//�ܵ���� =  + ���뾶
const double GBRIGHT = 235;	//�ܵ���� =  - ���뾶		

const double CORNER = 5;

/*************************�������******************************/

//����
const double FBLEFT = 29.262907;
const double FBRIGHT = 71.798508;
const double FREETOP = 64.428193;
const double FREEBOT = 18.184305;

//����
const double PK_X = 22.216028;
const double PK_Y = 79.3394;

//����
const double GK_X = 50.1189;
const double GK_Y = 41.8061;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++��û��

/*************************��������***********************************/
//����ת��ʱ
//const double CORRECTX = 0.8902;
//const double CORRECTY = 0.8957;

//�ٶ�
const double SPEED_ODD = 0.662;	    //0.338;��������Ϊ0ʱ�ļ��ٲ���
const double SPEED_ZERO = 0.1896;	// 0 ���ٶ� �� 125���ٶȵ��ٽ�ֵ

const double SPEED_TANGENT = 0.81;
const double SPEED_NORMAL = 0.27;

const double SPEED_A = 0.060;
const double SPEED_B = 0.015222305;

//�Ƕ�
const double ANGLE_A = 0.273575;
const double ANGLE_B = 0.534262;
const double ANGLE_K = 0.000294678;

/**********************************************************************/
const double PI = 3.1415926;



/*****************************���������*******************************/
const Vector2 CONSTGATE = { GRIGHT,(GTOP + GBOT) / 2.0 };
const Vector2 TOPGATE = { 220,165 };
const Vector2 BOTGATE = { 220,5.5 };