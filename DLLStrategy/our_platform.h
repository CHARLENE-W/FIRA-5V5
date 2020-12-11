#pragma once
#include <cstdint>
#include "platform.h"

///////////////////////////////////////////////////////////////////////
/////////////////////////////////数据结构//////////////////////////////
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
	Vector3D my_old_pos[5];//我方队员上次的坐标
	Vector3D my_speed[5];//我方队员的速度
	Vector3D my_old_velocity[5];//我方队员上次的轮速

	Vector3D op_old_pos[5];//对方队员上次的坐标
	Vector3D op_speed[5];//对方队员的速度

	MyRobot robot[5];//我方球员 
	MyRobot opp[5];//对方球员                //唯一改的地方

	Vector3D ball_old;//球上次的坐标
	Vector3D ball_cur;//球现在的坐标
	Vector3D ball_pre;//球预测的坐标
	Vector3D ball_speed;//球速度的坐标


	long time[2];//time[1]取样周期//time[0]取样次数
	bool mygrand;//是 = 黄队//否 = 兰队
	bool locked;//是否已经判断场地	
	int WIB;//分区

	Vector3D block[100][100];//当前块的中心坐标
	int block_my[100][100];//我方队员分区
	int block_op[100][100];//对方队员分区
	int block_ball[100][100];//球分区
	Block block_min;//最小块x值
	Block block_max;//最大块x值
	Block my_block_pos[5];//我方队员的块坐标 //-1 = 不在栅格区域内 //值 >= 0 = 存在
	Block op_block_pos[5];//对方队员的块坐标 //-1 = 不在栅格区域内 //值 >= 0 = 存在
	Block ball_block_pos;//球的块坐标

	int bgoalball;//球门球
	int nfreeball;//自由球
	int nplaceball;
	int npenaltyball;//点对方球
	int chooserobot;//选择点球队员

	int ActiveAttacker;//队员
	int NegativeAttacker;
	int Attacker;
	int Defender;
	int Keeper;

	long gameState;	//0,1,2,3,4,5
	long whoseBall; //0,1,2

	long n;
	bool B_N;				//these two veriaty is for the test funtion!
	//	Bounds field, goal;

	bool debug;//是否是调试


}Mydata;


///////////////////////////////////////////////////////////////////////
///////////////////////////////////常量////////////////////////////////
///////////////////////////////////////////////////////////////////////



/*****************************场地数据***********************************/
//球场范围
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


/**********************************比赛数据***************************/

const long   CAR = 8;	                            //小车边长 2.95						
const double ROBOTWITH = 3.14 * 2.54;
const double BALLWITH = 1.5 * 2.54;




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++还没改
//球门区

const double BD_TOP = 110;                 // 球门上边界
const double BD_BOT = 70;                 // 球门下边界
/*左半场*/const double BD_LEFT = -15;       // 球门左边界//
/*右半场*/const double BD_RIGHT = 235;      // 球门右边界//

//小禁区
const double SRG_TOP = 115;                      //小禁区上边界 
const double SRG_BOT = 75;                      //小禁区下边界
/*左半场*/const double SRG_LEFT = 7.5;           //小禁区左线
/*右半场*/const double SRG_RIGHT = 212.5;          //小禁区右线

//不进入小禁区

//大禁区
const double BRG_TOP = 130;                      //大禁上线  
const double BRG_BOT = 50;                      //大禁下线 
/*左半场*/const double BRG_LEFT = 37.5;           //大禁左线 
/*右半场*/const double BRG_RIGHT = 182.5;         //大禁右线


//球网
const double GBLEFT = -15;//能到达的 =  + 车半径
const double GBRIGHT = 235;	//能到达的 =  - 车半径		

const double CORNER = 5;

/*************************点的坐标******************************/

//争球
const double FBLEFT = 29.262907;
const double FBRIGHT = 71.798508;
const double FREETOP = 64.428193;
const double FREEBOT = 18.184305;

//点球
const double PK_X = 22.216028;
const double PK_Y = 79.3394;

//门球
const double GK_X = 50.1189;
const double GK_Y = 41.8061;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++还没改

/*************************修正数据***********************************/
//坐标转换时
//const double CORRECTX = 0.8902;
//const double CORRECTY = 0.8957;

//速度
const double SPEED_ODD = 0.662;	    //0.338;左右轮速为0时的减速参数
const double SPEED_ZERO = 0.1896;	// 0 减速度 和 125减速度的临界值

const double SPEED_TANGENT = 0.81;
const double SPEED_NORMAL = 0.27;

const double SPEED_A = 0.060;
const double SPEED_B = 0.015222305;

//角度
const double ANGLE_A = 0.273575;
const double ANGLE_B = 0.534262;
const double ANGLE_K = 0.000294678;

/**********************************************************************/
const double PI = 3.1415926;



/*****************************几个特殊点*******************************/
const Vector2 CONSTGATE = { GRIGHT,(GTOP + GBOT) / 2.0 };
const Vector2 TOPGATE = { 220,165 };
const Vector2 BOTGATE = { 220,5.5 };