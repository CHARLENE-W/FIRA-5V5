#include "stdafx.h"
#include "platform.h"
#include "adapter.h"
#include "our_platform.h"
#include<math.h>
#include<algorithm>
using namespace std;
using namespace Simuro;
using namespace Adapter;
JudgeType whichType;
int Eventstate;
int Judgestate;

void Init(Field* field);//初始化 放在getplacement
void See(Field* field); //预处理

void Action(Field* field);
void End(Field* field);//放在getinstruction

/************************数据处理*********************************/
void RegulateAngle(double& angle);
void RegulateAngle(float& angle);//规范angle的大小在（-180，+180）之间 
double Atan(double y, double x);                                   //在坐标平面与水平面的夹角
double Atan(Vector3D begin, Vector2 end);
double Atan(Vector3D begin, Vector3D end); //求两个点之间的夹角(-180~180)
double Distance(Vector3D pos1, Vector3D pos2);                    //求两点之间的距离
double Distance(Vector3D pos1, Vector2 pos2);
/************************辅助函数*********************************/
double AngleOne(double omiga, double vl, double vr);                //计算在当前角速度omiga的基础上以左右轮速vl,vr控制，下一个周期达到的角速度
double VelocityOne(double speed, double vl, double vr);             //计算在当前速度speed的基础上以左右轮速vl,vr控制，下一个周期达到的速度 返回下一个周期达到的速度
void Velocity(Field* field, int robot, double vl, double vr);   //修改机器人左右轮速
Vector3D Meetball_p(Field* field, int robot);



/************************运动处理*********************************/
void Angle(Field* field, int robot, double angle);//转向angle
void Angle(Field* field, int robot, Vector3D pos);//车转向目标点
void PAngle(Field* field, int robot, double angle, double speed = 0);//让robot朝angle的方向跑，并且speed控制它的最大轮速
void PositionAndStop(Field* field, int  robot, Vector3D pos, double bestangle = 90, double limit = 1);//(一般队员)让robot 跑到pos，并且停下来， bestangle 是停下来之后的朝向，limit	控制停在pos附近的距离
void GoaliePosition(Field* field, int  robot, Vector3D pos, double bestangle = 90, double limit = 1.5);//（守门员）同上
void PositionAndStopX(Field* field, int  robot, Vector3D pos, double Xangle = 90, double limit = 2); //让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度，limit	控制停在pos附近的距离
void PositionBallX(Field* field, int  robot, Vector3D pos, double Xangle = 90, double limit = 3.5);  //让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度,limit	控制球和robot的距离,如果球和队员的距离大于limit则不旋转
void PositionAndThrough(Field* field, int robot, Vector3D pos, double MAX = 125);//让robot以最快MAX 冲向pos，中间没有减速控制



/************************踢球动作*********************************/
//9：56

void Kick(Field* field, int  robot, Vector2 ToPos);       //让robot把球踢到ToPos的位置
void Kick(Field* field, int  robot, Vector3D ToPos);
void Kick(Field* field, int  robot, int robot1);             //让robot 以与robot1的角度跑向它
void Kick(Field* field, int robot, int steps, double limits);    //向着steps个周期后球的位置踢
void shoot(Field* field, int robot);                           //射门


// wan
/************************防止犯规*********************************/
bool Within(Field* field, int robot, double LENGTH);            //判断robot队员和球的距离是否再LENGTH规定的范围内返回true  or false

/**************************比赛*********************************/
 //void NormalGame ( Field *field );
void FreeBallGame(Field* field);
void PlaceBallGame(Field* field);
void PenaltyBallGame(Field* field);
void FreeKickGame(Field* field);
void GoalKickGame(Field* field);

/****************************补充**********************************/
void Sweep(Field* field, int robot);
int  WhoseBall(Field* field);


void Keeper(Field* field, int robot);
void Order(Field* field);

int CheckBall(Field* field);
void CheckBlockInfo(Field* field);
void PredictBall(Field* field, int steps = 1);

/*main*/
void OnEvent(EventType type, void* argument) {
	SendLog(L"V/DLLStrategy:OnEvent()");
	switch (type) {
	case EventType::FirstHalfStart: {
		SendLog(L"First Half Start");
		Eventstate = 3;
		break;
	}
	case EventType::SecondHalfStart: {
		SendLog(L"Second Half Start");
		Eventstate = 4;
		break;
	}
	case EventType::OvertimeStart: {
		SendLog(L"Overtime Start");
		Eventstate = 5;
		break;
	}
	case EventType::PenaltyShootoutStart: {
		SendLog(L"Penalty Shootout Start");
		Eventstate = 6;
		break;
	}
	case EventType::JudgeResult: {
		JudgeResultEvent* judgeResult = static_cast<JudgeResultEvent*>(argument);
		switch (judgeResult->type) {
		case JudgeType::PlaceKick:
			SendLog(L"Place Kick");
			Judgestate = 0;
			break;
		case JudgeType::PenaltyKick:
			SendLog(L"Penalty Kick");
			Judgestate = 2;
			break;
		case JudgeType::GoalKick:
			SendLog(L"Goal Kick");
			Judgestate = 1;
			break;
		case JudgeType::FreeKickLeftBot:
			SendLog(L"Free Kick");
			Judgestate = 6;
			break;
		case JudgeType::FreeKickLeftTop:
			SendLog(L"Free Kick");
			Judgestate = 5;
			break;
		case JudgeType::FreeKickRightBot:
			SendLog(L"Free Kick");
			Judgestate = 4;
			break;
		case JudgeType::FreeKickRightTop:
			SendLog(L"Free Kick");
			Judgestate = 3;
			break;
		}
		break;
	}
	default:
		break;
	}
}


void GetTeamInfo(TeamInfo* teamInfo) {
	SendLog(L"V/DLLStrategy:GetTeamInfo()");
	static const wchar_t teamName[] = L"BITCS1";
	static constexpr size_t len = sizeof(teamName);
	memcpy(teamInfo->teamName, teamName, len);
}

void GetInstruction(Field* field) {
	SendLog(L"V/DLLStrategy:GetInstruction()");
	/**/
	Mydata* p;
	p = (Mydata*)env->userData;

	if (!p->locked)		// 是 判断场地了 ??
	{//确定区域,blue or yellow
		if (env->home[0].pos.x < 50.0)
			p->mygrand = true; /// 是 = 黄队??
		else
			p->mygrand = false;
		p->locked = true;
	}
	/**/
	See(field);
	Action(field);
}

void GetPlacement(Field* field) {
	SendLog(L"V/DLLStrategy:GetPlacement()");
	double bx = field->ball.position.x;
	double by = field->ball.position.y;
	if (Judgestate == 0)
	{
		field->selfRobots[4].position.x = 0;
		field->selfRobots[4].position.y = 20;

		field->selfRobots[3].position.x = 40;
		field->selfRobots[3].position.y = 9;

		field->selfRobots[2].position.x = 40;
		field->selfRobots[2].position.y = -9;

		field->selfRobots[1].position.x = 40;
		field->selfRobots[1].position.y = 0;
	}
	else if (Judgestate == 2)
	{
		field->selfRobots[4].position.x = 50;
		field->selfRobots[4].position.y = 50;

		field->selfRobots[0].position.y = 0;
	}
	else if (Judgestate == 1)
	{
		field->ball.position.y = 23;
		field->ball.position.x = 97;

		field->selfRobots[0].position.y = 0;
	}
}





/*main*/




/*******************************************************************************************
********************************************具体实现****************************************/

void Init(Field * field) {
	//Init里不判断黄蓝，但需要变换原点

	//env->userData = (void*) new Mydata;
	Mydata* p;
	p = (Mydata*)env->userData;

	p->n = 125;			//these two veriaty is for the test funtion!
	p->B_N = true;		//

	p->debug = true;//是 = 调试

	p->time[1] = 0;//初始化时间
	p->time[0] = 125;//初始化时间

	p->bgoalball = 0;//初始化非求门球
	p->nfreeball = 0;//初始化非求门球
	p->nplaceball = 0;

	p->ActiveAttacker = -1;
	p->NegativeAttacker = -1;
	p->Attacker = -1;
	p->chooserobot = 0;

	for (int i = 0; i < 5; i++)
	{//我方
		p->robot[i].position.x = field->selfRobots[i].position.x + 110;
		p->robot[i].position.y = field->selfRobots[i].position.y + 90;
		p->robot[i].position.z = 0;

		p->my_old_pos[i].x = field->selfRobots[i].position.x + 110;
		p->my_old_pos[i].y = field->selfRobots[i].position.y + 90;
		p->my_old_pos[i].z = 0;

		p->my_speed[i].x = 0;
		p->my_speed[i].y = 0;
		p->my_speed[i].z = 0;

		p->my_old_velocity[i].x = 0;
		p->my_old_velocity[i].y = 0;
		p->my_old_velocity[i].z = 0;
		//对方
		p->opp[i].position.x = field->opponentRobots[i].position.x + 110;
		p->opp[i].position.y = field->opponentRobots[i].position.y + 90;
		p->opp[i].position.z = 0;

		p->op_old_pos[i].x = field->opponentRobots[i].position.x + 110;
		p->op_old_pos[i].y = field->opponentRobots[i].position.y + 90;
		p->op_old_pos[i].z = 0;

		p->op_speed[0].x = 0;
		p->op_speed[0].y = 0;
		p->op_speed[0].z = 0;

	}

	p->locked = false;// 是否判断了场地 ??
	p->mygrand = true;// 是 = 黄队//否 = 兰队 ??

	p->ball_old.x = field->ball.position.x + 110;
	p->ball_old.y = field->ball.position.y + 90;
	p->ball_old.z = 0;

	p->ball_cur.x = field->ball.position.x + 110;
	p->ball_cur.y = field->ball.position.y + 90;
	p->ball_cur.z = 0;

	p->ball_pre.x = field->ball.position.x + 110;
	p->ball_pre.y = field->ball.position.y + 90;
	p->ball_pre.z = 0;

	p->ball_speed.x = 0;
	p->ball_speed.y = 0;
	p->ball_speed.z = 0;

	//if (p->debug)
	//{
	//	p->debugfile = fopen("c:\\strategy\\SCU_introduction.txt", "w");
	//	if (!p->debugfile)
	//		p->debugfile = fopen("c:\\strategy\\SCU_introduction.doc", "w");
	//	if (!p->debugfile)
	//		p->debug = false;
	//	//		fclose(p->debugfile); 
	//}

}


void See(Field* field) {
	Mydata* p;
	p = (Mydata*)field->userData;

	int i = 0;

	if (p->mygrand)//如果，我方是黄队的话
	{///场地变换，球坐标变换
		//我方是黄队

		p->gameState = whichType;

		p->ball_cur.x = field->ball.position.x + 110;	//球坐标变化
		p->ball_cur.y = field->ball.position.y + 90;


		for (i = 0; i < 5; i++)
		{
			p->robot[i].position.x = field->selfRobots[i].position.x + 110;	//我方队员坐标变换
			p->robot[i].position.y = field->selfRobots[i].position.y + 90;
			p->robot[i].rotation = field->selfRobots[i].rotation;

			p->opp[i].position.x = field->opponentRobots[i].position.x + 110;	//对方坐标变换
			p->opp[i].position.y = field->opponentRobots[i].position.y + 90;
			p->opp[i].rotation = field->opponentRobots[i].rotation;
			RegulateAngle(p->opp[i].rotation);

		}
	}
	else
	{
		p->gameState = whichType;

		p->ball_cur.x = -field->ball.position.x + 110;	//球坐标变化
		p->ball_cur.y = -field->ball.position.y + 90;


		for (i = 0; i < 5; i++)
		{
			p->robot[i].position.x = -field->selfRobots[i].position.x + 110;	//我方队员坐标变换
			p->robot[i].position.y = -field->selfRobots[i].position.y + 90;
			p->robot[i].rotation = field->selfRobots[i].rotation + 180;

			p->opp[i].position.x = -field->opponentRobots[i].position.x + 110;	//对方坐标变换
			p->opp[i].position.y = -field->opponentRobots[i].position.y + 90;
			p->opp[i].rotation = field->opponentRobots[i].rotation + 180;
			RegulateAngle(p->opp[i].rotation);

		}
	}

	////第一次处理速度  (上次)
	for (i = 0; i < 5; i++)
	{///speed
		p->my_speed[i].x = (p->robot[i].position.x - p->my_old_pos[i].x);//70为比例系数，有待调整
		p->my_speed[i].y = (p->robot[i].position.y - p->my_old_pos[i].y);
		p->my_speed[i].z = Atan(p->my_speed[i].y, p->my_speed[i].x);//得到我方机器人的 运动方向 和转角速度,考虑中

		p->op_speed[i].x = (p->opp[i].position.x - p->op_old_pos[i].x);
		p->op_speed[i].y = (p->opp[i].position.y - p->op_old_pos[i].y);
		p->op_speed[i].z = Atan(p->op_speed[i].y, p->op_speed[i].x);//得到敌方机器人
	}

	p->ball_speed.x = p->ball_cur.x - p->ball_old.x;
	p->ball_speed.y = p->ball_cur.y - p->ball_old.y;
	p->ball_speed.z = Atan(p->ball_speed.y, p->ball_speed.x); //得到球的信息量
////以上部不可直接作为当前数据
///下面开始处理当前的真实数据
//////处理robot坐标
	double v, a, b, c, omiga, angle;
	for (i = 0; i < 5; i++)
	{
		omiga = p->robot[i].rotation - p->my_old_pos[i].z;
		RegulateAngle(omiga);
		omiga = AngleOne(omiga, p->my_old_velocity[i].x, p->my_old_velocity[i].y);
		c = p->robot[i].rotation;
		p->robot[i].rotation += omiga;
		RegulateAngle(p->robot[i].rotation);

		v = sqrt((p->my_speed[i].x * p->my_speed[i].x) + (p->my_speed[i].y * p->my_speed[i].y));
		angle = p->robot[i].rotation - p->my_speed[i].z;
		RegulateAngle(angle);
		if (angle > -90 && angle < 90)
			v = v;
		else
			v = -v;

		v = VelocityOne(v, p->my_old_velocity[i].x, p->my_old_velocity[i].y);
		a = p->robot[i].position.x;
		b = p->robot[i].position.y;

		p->robot[i].position.x += v * cos(p->robot[i].rotation * PI / 180);
		p->robot[i].position.y += v * sin(p->robot[i].rotation * PI / 180);
		///处理撞墙
		//不处理最好

		////处理撞墙		
		p->my_old_pos[i].x = a;
		p->my_old_pos[i].y = b;
		p->my_old_pos[i].z = c;

		p->my_speed[i].x = (p->robot[i].position.x - p->my_old_pos[i].x);	//70为比例系数，有待调整
		p->my_speed[i].y = (p->robot[i].position.y - p->my_old_pos[i].y);
		p->my_speed[i].z = Atan(p->my_speed[i].y, p->my_speed[i].x);
	}

	/////////	 预测球的坐标

	double x, y;
	x = p->ball_cur.x;
	y = p->ball_cur.y;

	PredictBall(field);		//求到现在球的位置
	p->ball_cur = p->ball_pre;

	p->ball_old.x = x;
	p->ball_old.y = y;

	PredictBall(field);		//预测下一步球的位置

	p->ball_speed.x = p->ball_cur.x - p->ball_old.x;
	p->ball_speed.y = p->ball_cur.y - p->ball_old.y;
	p->ball_speed.z = Atan(p->ball_speed.y, p->ball_speed.x);
	/////////	 预测球的坐标

	p->WIB = CheckBall(field);
	CheckBlockInfo(field);
	///计时
	///很有用处的
	p->time[1]++;
	if (p->time[1] == 60) {
		p->time[1] = 0;
		p->time[0]++;
	}
	if (p->ball_cur.y > 42) {
		p->ActiveAttacker = 3;
		p->Attacker = 1;
		p->NegativeAttacker = 4;
		p->Defender = 2;
	}
	else {
		p->ActiveAttacker = 4;
		p->Attacker = 2;
		p->NegativeAttacker = 3;
		p->Defender = 1;
	}


}


void End(Field* field) {
	//做一些清扫的工作
	//做一些记录整理工作

	Mydata* p;
	p = (Mydata*)env->userData;

	int i = 0;

	for (i = 0; i < 5; i++) {//速度
		field->selfRobots[i].wheel.leftSpeed = p->robot[i].wheel.velocityLeft;
		field->selfRobots[i].wheel.rightSpeed = p->robot[i].wheel.velocityRight;

		p->my_old_velocity[i].x = p->robot[i].wheel.velocityLeft;
		p->my_old_velocity[i].y = p->robot[i].wheel.velocityRight;
	}

	if (p->mygrand) {///场地变换，球坐标变换
		p->ball_old.x = field->ball.position.x + 110;		//球坐标变化
		p->ball_old.y = field->ball.position.y + 90;
		//p->ball_cur.z = env->currentBall.pos.z;

		for (i = 0; i < 5; i++) {
			p->my_old_pos[i].x = env->home[i].pos.x + 110;	//我方队员坐标变换
			p->my_old_pos[i].y = env->home[i].pos.y + 90;
			//p->robot[i].pos.z = env->home[i].pos.z ;
			p->my_old_pos[i].z = env->home[i].rotation;

			p->op_old_pos[i].x = env->opponent[i].pos.x + 110;	//对方坐标变换
			p->op_old_pos[i].y = env->opponent[i].pos.y + 90;
			//p->opp[i].pos.z = env->opponent[i].pos.z;
			p->op_old_pos[i].z = env->opponent[i].rotation;
			RegulateAngle(p->op_old_pos[i].z);
		}
	}
	else {
		p->ball_old.x = 110 - field->ball.position.x;		//球坐标变化
		p->ball_old.y = 90  - field->ball.position.y;
		//p->ball_cur.z = env->currentBall.pos.z;

		for (i = 0; i < 5; i++) {
			p->my_old_pos[i].x = 110 - field->selfRobots[i].position.x;	//我方队员坐标变换
			p->my_old_pos[i].y = 90  - field->selfRobots[i].position.y;
			//p->robot[i].pos.z = env->home[i].pos.z ;
			p->my_old_pos[i].z = 180.0 + env->home[i].rotation;
			RegulateAngle(p->my_old_pos[i].z);

			p->op_old_pos[i].x = 110 - field->opponentRobots[i].position.x;	//对方坐标变换
			p->op_old_pos[i].y = 90  - field->opponentRobots[i].position.y;
			p->op_old_pos[i].z = 180.0 + env->opponent[i].rotation;
			RegulateAngle(p->op_old_pos[i].z);
		}
	}

	/*if (p->debug) {
		fprintf(p->debugfile, "\n");
	}*/
}


int CheckBall(Field* field)
{
	Mydata* p;
	p = (Mydata*)field->userData;
	int k;
	int WIB;

	double x1 = 21.5;
	double x2 = 50.1;
	double x3 = 78.6;

	double y1 = 25.1;
	double y2 = 41.8;
	double y3 = 58.6;

	Vector3D ball;
	ball.x = p->ball_cur.x;
	ball.y = p->ball_cur.y;

	if (ball.x <= x1)
		k = 0;
	else if (ball.x > x1 && ball.x <= x2)
		k = 4;
	else if (ball.x > x2 && ball.x <= x3)
		k = 8;
	else if (ball.x > x3)
		k = 12;
	if (ball.y <= y1)
		WIB = 1 + k;
	else if (ball.y > y1 && ball.y <= y2)
		WIB = 2 + k;
	else if (ball.y > y2 && ball.y <= y3)
		WIB = 3 + k;
	else if (ball.y > y3)
		WIB = 4 + k;
	return WIB;
}

void CheckBlockInfo(Field* field)
{
	Mydata* p;
	p = (Mydata*)field->userData;
	int i, j, k;
	double x = 0;//相对块x值
	double x_min = 0;//当前块x最小值
	double x_max = 0;//当前块x最大值 
	//double x1=6.8118;//区域x最小值
	double x1 = 21.5;//区域x最小值 //bug fixed: 存在区域判断出错的问题
	//double x2=78.6;//区域x最大值
	double x2 = 93.4259;//区域x最大值 //bug fixed: 存在区域判断出错的问题

	double y = 0;//相对块y值
	double y_min = 0;//当前块y最小值
	double y_max = 0;//当前块y最大值
	double y1 = 6.3730;//区域y最小值
	double y2 = 77.2392;//区域y最大值
	int block_size = 2;//块大小
	int block_x_num = 0;//x块总数量
	int block_y_num = 0;//y块总数量
	int block_x_num_judge = 0;//判断x用
	int block_y_num_judge = 0;//判断y用

	block_x_num = (x2 - x1) / block_size;
	block_x_num++;//算入边界块
	block_y_num = (y2 - y1) / block_size;
	block_y_num++;//算入边界块

	//循环赋初值队员的块编号
	for (k = 0; k < 5; k++) {
		p->my_block_pos[k].x = -1;
		p->my_block_pos[k].y = -1;
		p->op_block_pos[k].x = -1;
		p->op_block_pos[k].y = -1;
	}

	for (i = 0; i < block_x_num; i++) {
		for (j = 0; j < block_y_num; j++) {
			x = i * block_size;
			x_min = x + x1;
			x_max = x_min + block_size;

			y = j * block_size;
			y_min = y + y1;
			y_max = y_min + block_size;

			//将当前块的中心x和y坐标，存入block当中
			p->block[i][j].x = x_min + (block_size / 2);
			if (block_y_num != 0) {
				block_y_num_judge = block_y_num;//fix bug
				block_y_num_judge--;//fix bug
				if (j == block_y_num_judge) {//fix bug
				//if(j == (block_y_num--)){//bug
					p->block[i][j].y = y2;
				}
				else {
					p->block[i][j].y = y_min + (block_size / 2);
				}
			}
			else {
				p->block[i][j].y = y2;
			}

			//循环读取队员的块编号
			for (k = 0; k < 5; k++) {
				if ((p->robot[k].position.x >= x_min) && (p->robot[k].position.y >= y_min) && (p->robot[k].position.x < x_max) && (p->robot[k].position.y < y_max)) {
					p->block_my[i][j] = 1;
					p->my_block_pos[k].x = i;
					p->my_block_pos[k].y = j;
				}
				else {
					p->block_my[i][j] = 0;
				}
				if ((p->opp[k].position.x >= x_min) && (p->opp[k].position.y >= y_min) && (p->opp[k].position.x < x_max) && (p->opp[k].position.y < y_max)) {
					p->block_op[i][j] = 1;
					p->op_block_pos[k].x = i;
					p->op_block_pos[k].y = j;
				}
				else {
					p->block_op[i][j] = 0;
				}
			}

			//读取球的块坐标
			if ((p->ball_cur.x >= x_min) && (p->ball_cur.y >= y_min) && (p->ball_cur.x < x_max) && (p->ball_cur.y < y_max)) {
				p->block_ball[i][j] = 1;
				p->ball_block_pos.x = i;
				p->ball_block_pos.y = j;
			}
			else {
				p->block_ball[i][j] = 0;
			}
		}
	}

	//存入块信息
	p->block_min.x = 0;
	p->block_min.y = 0;
	p->block_max.x = block_x_num;
	p->block_max.y = block_y_num;
}

void PredictBall(Field* field, int steps)
{
	Mydata* p;
	p = (Mydata*)field->userData;

	Vector3D predictball;
	Vector3D ball_speed;
	int i = 0;

	predictball.x = p->ball_cur.x;			//赋初值
	predictball.y = p->ball_cur.y;
	ball_speed.x = p->ball_speed.x;
	ball_speed.y = p->ball_speed.y;
	ball_speed.z = p->ball_speed.z;

	for (i = 0; i < steps; i++) {
		predictball.x += ball_speed.x;
		predictball.y += ball_speed.y;
		//处理撞墙
		if (predictball.x > GRIGHT) {
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			ball_speed.x *= -SPEED_NORMAL;	//loose 
			ball_speed.y *= SPEED_TANGENT;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		else if (predictball.x < GLEFT) {
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			ball_speed.x *= -SPEED_NORMAL;	//loose 
			ball_speed.y *= SPEED_TANGENT;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		else if (predictball.y < GBOT) {
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			ball_speed.x *= SPEED_TANGENT;	//loose 
			ball_speed.y *= -SPEED_NORMAL;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		else if (predictball.y > GTOP) {
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			ball_speed.x *= SPEED_TANGENT;	//loose 
			ball_speed.y *= -SPEED_NORMAL;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		/////////////////对于边界时的设置
		if (predictball.x + predictball.y > GRIGHT + GTOP - CORNER) {//右上
			double vx, vy;
			vy = 0.7071 * ball_speed.y + 0.7071 * ball_speed.x;	//变换1
			vx = -0.7071 * ball_speed.y + 0.7071 * ball_speed.x;

			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			vx *= SPEED_TANGENT;	//loose 
			vy *= -SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy - 0.7071 * vx;	//变换2
			ball_speed.x = 0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;

		}
		else if (predictball.x + predictball.y < GLEFT + GBOT + CORNER) {//左下
			double vx, vy;
			vy = 0.7071 * ball_speed.y + 0.7071 * ball_speed.x;	//变换1
			vx = -0.7071 * ball_speed.y + 0.7071 * ball_speed.x;
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			vx *= SPEED_TANGENT;	//loose 
			vy *= -SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy - 0.7071 * vx;	//变换2
			ball_speed.x = 0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		else if (predictball.x - predictball.y > GRIGHT - GBOT - CORNER) {//右下
			double vx, vy;
			vy = 0.7071 * ball_speed.y - 0.7071 * ball_speed.x;	//变换1
			vx = 0.7071 * ball_speed.y + 0.7071 * ball_speed.x;
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			vx *= SPEED_TANGENT;	//loose 
			vy *= -SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy + 0.7071 * vx;	//变换2
			ball_speed.x = -0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		else if (predictball.y - predictball.x > GTOP - GLEFT - CORNER) {//左上
			double vx, vy;
			vy = 0.7071 * ball_speed.y - 0.7071 * ball_speed.x;	//变换1
			vx = 0.7071 * ball_speed.y + 0.7071 * ball_speed.x;
			predictball.x -= ball_speed.x;	//retern
			predictball.y -= ball_speed.y;
			vx *= SPEED_TANGENT;	//loose 
			vy *= -SPEED_NORMAL;
			ball_speed.y = 0.7071 * vy + 0.7071 * vx;	//变换2
			ball_speed.x = -0.7071 * vy + 0.7071 * vx;
			predictball.x += ball_speed.x;	//go on
			predictball.y += ball_speed.y;
		}
		//处理四角		
	}
	p->ball_pre.x = predictball.x;
	p->ball_pre.y = predictball.y;
	p->ball_pre.z = Atan(ball_speed.y, ball_speed.x);
}





















void RegulateAngle(float& angle) {
	while (angle >= 180.0)angle -= 360.0;
	while (angle < -180.0)angle += 360.0;
}

void RegulateAngle(double& angle) {
	while (angle >= 180.0)angle -= 360.0;
	while (angle < -180.0)angle += 360.0;
}

double Atan(double y, double x) {
	if (x != 0.0 || y != 0.0)
		return 180 * atan2(y, x) / PI;
	else return 0.0;
}

double Atan(Vector3D begin, Vector3D end) {
	double y, x;
	y = end.y - begin.y;
	x = end.x - begin.x;
	return Atan(y, x);
}
double Atan(Vector3D begin, Vector2 end) {
	double y, x;
	y = end.y - begin.y;
	x = end.x - begin.x;
	return Atan(y, x);
}


/****************************直线处理*********************************/
//1.Distance  两点之间距离
/*******************************************************************/
double Distance(Vector3D pos1, Vector3D pos2) {
	return sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x) + (pos1.y - pos2.y) * (pos1.y - pos2.y));
}
double Distance(Vector3D pos1, Vector2 pos2) {
	return sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x) + (pos1.y - pos2.y) * (pos1.y - pos2.y));
}


/************************辅助函数*********************************/
//1.Velocity 修改左右轮速
//2.CheckBall返回分区号
//3.PredictBall 预测经过 steps 个周期之后球的位置
//4.Meetball_p 求出robot追到球的位置
//5.order 角色分配
//6.AngleOne计算在当前角速度omiga的基础上以左右轮速vl,vr控制，下一个周期达到的角速度
//7.VelocityOne 计算在当前速度speed的基础上以左右轮速vl,vr控制，下一个周期达到的速度 返回下一个周期达到的速度
/*****************************************************************/
void Velocity(Field* field, int robot, double vl, double vr)
{
	Mydata* p;
	p = (Mydata*)field->userData;

	//vl,vr都有取值范围的!!!
	if (vl > 125)vl = 125;
	if (vl < -125)vl = -125;
	if (vr > 125)vr = 125;
	if (vr < -125)vr = -125;

	if (true) {//速度的特别控制//重要，否则小车是颤抖的
		if (vl == 0 && vr != 0)
			vl = 0.00001;
		if (vr == 0 && vl != 0)
			vr = 0.00001;
	}
	p->robot[robot].wheel.leftSpeed = vl;
	p->robot[robot].wheel.rightSpeed = vr;
}

Vector3D Meetball_p(Field* field, int robot)
{//求出robot追到球的位置
	Mydata* p;
	p = (Mydata*)field->userData;
	Vector3D meetpoint = { 0,0,-1 };
	double dis = Distance(p->ball_cur, p->robot[robot].position);

	double t = 0;
	double vb = 0;
	double v = 1.9;		//按照最大速度计算
	double pos_angle, b_sp_angle;

	pos_angle = Atan(p->ball_cur.y - p->robot[robot].position.y, p->ball_cur.x - p->robot[robot].position.x);
	b_sp_angle = p->ball_speed.z;
	vb = (p->ball_speed.y * p->ball_speed.y + p->ball_speed.x * p->ball_speed.x);
	t = sin((b_sp_angle - pos_angle) * PI / 180);
	t = vb * t * t;
	v = v * v;
	if (v > t)
	{
		v = sqrt(v - t) + sqrt(vb) * cos((b_sp_angle - pos_angle) * PI / 180);
		if (v > 0.1)
		{
			t = dis / v;	//得到步数
			meetpoint.x = p->ball_speed.x * t + p->ball_cur.x;
			meetpoint.y = p->ball_speed.y * t + p->ball_cur.y;
			meetpoint.z = t;
		}
	}
	return meetpoint;
}


double AngleOne(double omiga, double vl, double vr)
{
	//		omiga = p->robot[i].rotation - p->my_old_pos[i].z ;
	//		RegulateAngle(omiga);
	if (vl > 125)vl = 125;
	if (vl < -125)vl = -125;
	if (vr > 125)vr = 125;
	if (vr < -125)vr = -125;
	double angle = (vr - vl) / 2;

	RegulateAngle(omiga);
	omiga += ANGLE_A * (ANGLE_B * angle - omiga);
	if (vr > vl) {
		if (vl >= 0 || vr <= 0) {
			omiga -= 4 * ANGLE_K * angle * angle;
		}
	}
	else if (vr < vl) {
		if (vr >= 0 || vl <= 0) {
			omiga += 4 * ANGLE_K * angle * angle;
		}
	}
	RegulateAngle(omiga);		//应该没有大于180 的角速度罢
	return omiga;
}

double VelocityOne(double speed, double vl, double vr) {
	if (vl > 125)vl = 125;
	if (vl < -125)vl = -125;
	if (vr > 125)vr = 125;
	if (vr < -125)vr = -125;

	if (speed > 3 || speed < -3)
		speed = 0;
	if (vl == 0 && vr == 0)
		speed += -SPEED_ODD * speed;
	else
		speed += SPEED_A * (SPEED_B * (vl + vr) / 2 - speed);
	return speed;
}




/****************************基本动作*********************************/
//1.Angle 转向angle
//2.Angle让robot转到正对pos的方向
//3.PAngle 让robot朝angle的方向跑，并且speed控制它的最大轮速
//4.PositionAndStop 让robot 跑到pos，并且停下来， bestangle 是停下来之后的朝向，limit	控制停在pos附近的距离
//5.GoaliePosition（守门员）同上
//6.PositionAndStopX 让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度，limit	控制停在pos附近的距离
//7.PositionBallX 让robot 跑到pos，并且停下来原地旋转，Xangle	旋转的角速度,limit	控制球和robot的距离,如果球和队员的距离大于limit则不旋转
//8.PositionAndThrough 让robot以最快MAX 冲向pos，中间没有减速控制
/*******************************************************************/

void Angle(Field* field, int robot, double angle) {
	Mydata* p;
	p = (Mydata*)field->userData;

	double speed = 0;		//和pangle接轨
	double accuracy = 1;
	double turnangle = 0, nextangle = 0;
	double FF = 125;		//最大减速度

	turnangle = angle - p->robot[robot].rotation;
	RegulateAngle(turnangle);
	if (turnangle < 1 && turnangle >-1) {
		Velocity(field, robot, 0, 0);
		return;
	}
	else if (turnangle < 2 && turnangle >-2)
		FF = 10;
	else if (turnangle > -3 && turnangle < 3)
		FF = 15;
	else if (turnangle > -5 && turnangle < 5)
		FF = 30;

	double v = p->robot[robot].rotation - p->my_old_pos[robot].z;
	RegulateAngle(v);

	double v1 = v;
	double f = 0;	//相当于减速时,右轮速度，
//	int n=0;
	bool turnleft = true;			//判断小车是否是该向左转
	double a = ANGLE_A;
	double b = ANGLE_B;

	if (turnangle > 90) {
		turnleft = false;
		turnangle -= 180;
	}
	else if (turnangle > 0) {
		turnleft = true;
	}
	else if (turnangle > -90) {
		turnleft = false;
	}
	else { //<-90时
		turnleft = true;
		turnangle += 180;
	}

	if (turnleft) {//
		f = -FF;
		v1 = AngleOne(v1, speed + f, speed - f);		//v1+=a *( -b *f-v1);
		nextangle += v1;
		do {//whether to reduce
			//收敛!!
			v1 = AngleOne(v1, speed - f, speed + f);//+= a *( b *f-v1);		// v1   
			nextangle += v1;
		} while (v1 > 0);
		nextangle -= v1;
		if (nextangle < turnangle) {//不满足减速条件 所以 f 取相反数
			Velocity(field, robot, speed + f, speed - f);
		}
		else {//reduce
			v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
			if (v1 < 0) {
				do {//该降低功率了
					f++;
					v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
				} while (v1 < turnangle && f < FF);
			}
			Velocity(field, robot, speed - f, speed + f);
		}
	}
	else {//
		f = FF;
		v1 = AngleOne(v1, speed + f, speed - f);		//v1+=a *( -b *f-v1);
		nextangle += v1;
		do {//whether to reduce
			v1 = AngleOne(v1, speed - f, speed + f);//+= a *( b *f-v1);		// v1   
			nextangle += v1;
		} while (v1 < 0);
		nextangle -= v1;
		if (nextangle > turnangle) {//不满足减速条件 所以 f 取相反数
			Velocity(field, robot, speed + f, speed - f);
		}
		else {//reduce
			v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
			if (v1 > 0) {
				do {//该降低功率了
					f--;
					v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
				} while (v1 > turnangle && f > -FF);
			}
			Velocity(field, robot, speed - f, speed + f);
		}
	}

}

void Angle(Field* field, int robot, Vector3D pos) {
	Mydata* p;
	p = (Mydata*)field->userData;

	double speed = 0;		//和pangle接轨
	double accuracy = 1;
	double turnangle = 0, nextangle = 0;
	double FF = 125;		//最大减速度
	double angle = 0;
	angle = Atan(p->robot[robot].position, pos);

	turnangle = angle - p->robot[robot].rotation;
	RegulateAngle(turnangle);

	if (turnangle < 1 && turnangle >-1) {
		Velocity(field, robot, 0, 0);
		return;
	}
	else if (turnangle < 2 && turnangle >-2)
		FF = 10;
	else if (turnangle > -3 && turnangle < 3)
		FF = 15;
	else if (turnangle > -5 && turnangle < 5)
		FF = 30;

	double v = p->robot[robot].rotation - p->my_old_pos[robot].z;
	RegulateAngle(v);
	double v1 = v;
	double f = 0;	//相当于减速时,右轮速度，
//	int n=0;
	bool turnleft = true;			//判断小车是否是该向左转
	double a = ANGLE_A;
	double b = ANGLE_B;

	if (turnangle > 90) {
		turnleft = false;
		turnangle -= 180;
	}
	else if (turnangle > 0) {
		turnleft = true;
	}
	else if (turnangle > -90) {
		turnleft = false;
	}
	else {
		turnleft = true;
		turnangle += 180;
	}

	if (turnleft) {//
		f = -FF;
		v1 = AngleOne(v1, speed + f, speed - f);		//v1+=a *( -b *f-v1);
		nextangle += v1;
		do {//whether to reduce
			//收敛!!
			v1 = AngleOne(v1, speed - f, speed + f);//+= a *( b *f-v1);		// v1   
			nextangle += v1;
		} while (v1 > 0);
		nextangle -= v1;
		if (nextangle < turnangle) {//不满足减速条件 所以 f 取相反数
			Velocity(field, robot, speed + f, speed - f);
		}
		else {//reduce	
			v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
			if (v1 < 0) {
				do {//该降低功率了
					f++;
					v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
				} while (v1 < turnangle && f < FF);
			}
			Velocity(field, robot, speed - f, speed + f);
		}
	}
	else {//
		f = FF;
		v1 = AngleOne(v1, speed + f, speed - f);		//v1+=a *( -b *f-v1);
		nextangle += v1;
		do {//whether to reduce
			v1 = AngleOne(v1, speed - f, speed + f);//+= a *( b *f-v1);		// v1   
			nextangle += v1;
		} while (v1 < 0);
		nextangle -= v1;
		if (nextangle > turnangle) {//不满足减速条件 所以 f 取相反数
			Velocity(field, robot, speed + f, speed - f);
		}
		else {//reduce
			v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
			if (v1 > 0) {
				do {//该降低功率了
					f--;
					v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
				} while (v1 > turnangle && f > -FF);
			}
			Velocity(field, robot, speed - f, speed + f);
		}
	}
}

void PAngle(Field* field, int robot, double angle, double speed)
{
	Mydata* p;
	p = (Mydata*)field->userData;

	double accuracy = 1;
	double turnangle = 0, nextangle = 0;
	turnangle = angle - p->robot[robot].rotation;
	RegulateAngle(turnangle);
	double v = p->robot[robot].rotation - p->my_old_pos[robot].z;
	RegulateAngle(v);
	double v1 = v;
	double FF = 125;		//最大减速度
	double f = 0;	//相当于减速时,右轮速度，
//	int n=0;
	bool turnleft = true;			//判断小车是否是该向左转
	double a = ANGLE_A;
	double b = ANGLE_B;

	bool face;
	if (turnangle < 90 && turnangle > -90) {//检查是否正面跑位	
		face = true;
		speed = speed;
	}
	else {
		face = false;
		speed = -speed;
	}
	if (turnangle > 90) {
		turnleft = false;
		turnangle -= 180;
	}
	else if (turnangle > 0) {
		turnleft = true;
	}
	else if (turnangle > -90) {
		turnleft = false;
	}
	else {
		turnleft = true;
		turnangle += 180;
	}

	if (turnleft)
	{//
		f = -FF;
		v1 = AngleOne(v1, speed + f, speed - f);		//v1+=a *( -b *f-v1);
		nextangle += v1;
		do {//whether to reduce
			//收敛!!
			v1 = AngleOne(v1, speed - f, speed + f);//+= a *( b *f-v1);		// v1   
			nextangle += v1;
		} while (v1 > 0);
		nextangle -= v1;
		if (nextangle < turnangle)
		{//不满足减速条件 所以 f 取相反数
			Velocity(field, robot, speed + f, speed - f);
		}
		else {//reduce	
			v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
			if (v1 < 0) {
				do {//该降低功率了
					f++;
					v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
				} while (v1 < turnangle && f < 125);
			}
			Velocity(field, robot, speed - f, speed + f);
		}
	}
	else {//
		f = FF;
		v1 = AngleOne(v1, speed + f, speed - f);		//v1+=a *( -b *f-v1);
		nextangle += v1;
		do {//whether to reduce
			v1 = AngleOne(v1, speed - f, speed + f);//+= a *( b *f-v1);		// v1   
			nextangle += v1;
		} while (v1 < 0);
		nextangle -= v1;
		if (nextangle > turnangle)
		{//不满足减速条件 所以 f 取相反数
			Velocity(field, robot, speed + f, speed - f);
		}
		else {//reduce
			v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
			if (v1 > 0) {
				do {//该降低功率了
					f--;
					v1 = AngleOne(v, speed - f, speed + f);  //v + a *( b *f-v);
				} while (v1 > turnangle && f > -125);
			}
			Velocity(field, robot, speed - f, speed + f);
		}
	}
}

void PositionAndStop(Field* field, int  robot, Vector3D pos, double bestangle, double limit) {
	Mydata* p;
	p = (Mydata*)field->userData;

	double anglespeedmax = 0;//控制转交速度的变量
	double vmax = 125;//默认的跑位加速度
	double Limitedangle = 2;//默认减速范围

	if (limit < 0.5)
		limit = 0.5;
	double Limiteddis = limit;//减速范围有一个下限，保证不会来回跑动

	double  distance;//robot和目标点的距离
	double turnangle, posangle, vangle;//转动角度 ，目标点相对robot的角度，速度的绝对角度
	double dx, dy;//pos  和robot的坐标差
	double a = SPEED_A;//参数
	double b = SPEED_B;
	double v, v1;//临时保存速度的大小!!!
	double f = vmax;//加速度变量
	double s = 0;	//预测的减速位移(路程)
	int n = 0;//跑位的步数
	bool face = true;//判断小车是否是正面前进

	v = sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y * p->my_speed[robot].y);
	//临时保存速度的大小!!!
	dx = pos.x - p->robot[robot].position.x;		//pos  和robot的坐标差
	dy = pos.y - p->robot[robot].position.y;

	distance = Distance(p->robot[robot].position, pos);
	posangle = Atan(dy, dx);

	turnangle = p->robot[robot].rotation - posangle;		//转动角度 
	RegulateAngle(turnangle);

	if (turnangle > 90) {//判断小车是否是正面前进
		face = false;
		turnangle -= 180;
	}
	else if (turnangle < -90) {
		face = false;
		turnangle += 180;
	}
	else {
		face = true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;		//速度的方向和robot正面的夹角
	RegulateAngle(vangle);					//主要用于最后控制减速度的大小
	if (vangle < -90 || vangle > 90)//同时判断v的正负
		v = -v;

	if (face) {//forward	跑位，如果后退的话  就v=0
		//设vl,vr=0 还是vl,vr=125 有一个条件有一个临界条件那就是 
		//v = SPEED_ZERO
		if (v < -SPEED_ZERO) {
			Velocity(field, robot, 0, 0);
			return;
		}
	}
	else if (v > SPEED_ZERO) {//back	跑位，如果后退的话  就v=0
		Velocity(field, robot, 0, 0);
		return;
	}

	v1 = v;	//v1 is changing while program running 
			//whlie, v is not

	if (distance > Limiteddis) {//it is too early to count the steps
		//but the Limiteddis should be tested!!	to do...
		if (turnangle > Limitedangle || turnangle < -Limitedangle) {//adjust angle
			/////////////////测试这一段
			//对于goalie这一段应该特别注意
			//发生变向	1.knock the robot,especially the opponent
			//	2.knock the wall
			// so the anglespeedmax is allowed ++ more!!
			if (turnangle > 20 || turnangle < -20)
				anglespeedmax = 0;
			else if (turnangle > 10 || turnangle < -10)
				anglespeedmax = 125;
			else if (turnangle > 5 || turnangle < -5)
				anglespeedmax = 180;
			else
				anglespeedmax = 200;
			///////////////测试这一段
			PAngle(field, robot, posangle, anglespeedmax);
		}
		else {
			if (face)
				Velocity(field, robot, f, f);
			else
				Velocity(field, robot, -f, -f);
		}//it is time to rush
	}
	else {
		if (distance > 1) {		//调整角度	return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if (turnangle > Limitedangle || turnangle < -Limitedangle) {
				Angle(field, robot, posangle);
				return;
			}
		}

		if (distance < 0.4) {	//停止并转向		return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if (v<0.1 && v>-0.1) {	//the range of v shoud be tested 
				if (bestangle == 0)
					Velocity(field, robot, 0, 0);
				else
					Angle(field, robot, bestangle);
				return;
			}
		}

		if (true) {
			vmax = 125;
			if (face) {
				f = -vmax;		//减速度  为  0000000
				v1 = VelocityOne(v1, -f, -f);		//加速一步
				s = v1;
				do {//whether to reduce
					if (v1 > SPEED_ZERO)	//as i said,this is limited
						v1 = VelocityOne(v1, 0, 0);
					else
						v1 = VelocityOne(v1, f, f);
					s += v1;
				} while (v1 > 0);

				s -= v1;

				if (s < distance) {//不满足减速条件加速
					Velocity(field, robot, -f, -f);
				}
				else {
					if (v > SPEED_ZERO)
						Velocity(field, robot, 0, 0);
					else {
						v1 = VelocityOne(v, f, f);		//减速一步
						if (v1 < 0)
						{
							do {//该降低功率了
								f++;		//f=-vmax;
								v1 = VelocityOne(v, f, f);
							} while (v1 < distance && f < vmax);
						}
						Velocity(field, robot, f, f);
					}
				}
			}
			else {
				f = vmax;		//减速度!!!!!
				v1 = VelocityOne(v1, -f, -f);
				s = v1;
				do {//whether to reduce
					if (v1 < -SPEED_ZERO)	//as i said,this is limited
						v1 = VelocityOne(v1, 0, 0);
					else
						v1 = VelocityOne(v1, f, f);
					s += v1;
				} while (v1 < -0.1);

				s -= v1;

				if (s > -distance) {//不满足减速条件加速
					Velocity(field, robot, -f, -f);
				}
				else {
					if (v < -SPEED_ZERO)
						Velocity(field, robot, 0, 0);
					else {
						v1 = VelocityOne(v, f, f);		//减速一步
						if (v1 > 0) {
							do {//该降低功率了
								f--;		//f=-vmax;
								v1 = VelocityOne(v, f, f);
							} while (v1 > -distance && f > -vmax);
						}
						Velocity(field, robot, f, f);
					}
				}
			}
		}
	}
}

void GoaliePosition(Field* field, int  robot, Vector3D pos, double bestangle, double limit) {	//考虑到可能的	急停和 急快速加速
	//特别作了优化
	//还有就是 被碰转后的转角过程 不能耽搁时间!!!
	//转角是最危险的过程

	Mydata* p;
	p = (Mydata*)field->userData;

	double anglespeedmax = 0;		//控制转交速度的变量
	double vmax = 125;			//默认的跑位加速度
	double Limitedangle = 2;		//默认减速范围

	if (limit < 0.5)
		limit = 0.5;
	double Limiteddis = limit;	//减速范围有一个下限，保证不会来回跑动

	double  distance;			//robot和目标点的距离
	double turnangle, posangle, vangle;	//转动角度 ，目标点相对robot的角度，速度的绝对角度
	double dx, dy;				//pos  和robot的坐标差
	double a = SPEED_A;			//参数
	double b = SPEED_B;
	double v, v1;				//临时保存速度的大小!!!
	double f = vmax;				//加速度变量
	double s = 0;					//预测的减速位移(路程)
	int n = 0;					//跑位的步数
	bool face = true;			//判断小车是否是正面前进

	v = sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y * p->my_speed[robot].y);
	//临时保存速度的大小!!!
	dx = pos.x - p->robot[robot].position.x;		//pos  和robot的坐标差
	dy = pos.y - p->robot[robot].position.y;

	distance = Distance(p->robot[robot].position, pos);
	posangle = Atan(dy, dx);

	turnangle = p->robot[robot].rotation - posangle;		//转动角度 
	RegulateAngle(turnangle);

	if (turnangle > 90) {//判断小车是否是正面前进
		face = false;
		turnangle -= 180;
	}
	else if (turnangle < -90) {
		face = false;
		turnangle += 180;
	}
	else {
		face = true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;		//速度的方向和robot正面的夹角
	RegulateAngle(vangle);					//主要用于最后控制减速度的大小
	if (vangle < -90 || vangle > 90)		//同时判断v的正负
		v = -v;

	if (face) {//forward	跑位，如果后退的话  就v=0
		//设vl,vr=0 还是vl,vr=125 有一个条件有一个临界条件那就是 
		//v = SPEED_ZERO
		if (v < -SPEED_ZERO) {
			Velocity(field, robot, 0, 0);
			return;
		}
	}
	else if (v > SPEED_ZERO) {//back		跑位，如果后退的话  就v=0
		Velocity(field, robot, 0, 0);
		return;
	}

	v1 = v;	//v1 is changing while program running 
			//whlie, v is not

	if (distance > Limiteddis) {//it is too early to count the steps
		//but the Limiteddis should be tested!!	to do...
		if (turnangle > Limitedangle || turnangle < -Limitedangle) {//adjust angle
			/////////////////测试这一段
			//对于goalie这一段应该特别注意
			//发生变向	1.knock the robot,especially the opponent
			//	2.knock the wall
			// so the anglespeedmax is allowed ++ more!!
			if (turnangle > 50 || turnangle < -50)
				anglespeedmax = 0;
			else if (turnangle > 30 || turnangle < -30)
				anglespeedmax = 80;
			else if (turnangle > 10 || turnangle < -10)
				anglespeedmax = 125;
			else if (turnangle > 5 || turnangle < -5)
				anglespeedmax = 180;
			else
				anglespeedmax = 200;
			///////////////测试这一段

			PAngle(field, robot, posangle, anglespeedmax);
		}
		else {
			if (face)
				Velocity(field, robot, f, f);
			else
				Velocity(field, robot, -f, -f);
		}//it is time to rush
	}
	else {
		if (distance > 1) {		//调整角度	return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if (turnangle > Limitedangle || turnangle < -Limitedangle) {
				Angle(field, robot, posangle);
				return;
			}
		}
		if (distance < 0.4) {	//停止并转向		return!!!!!!
			//radious of robot is about 1.5 ,so the distance is very short
			if (v<0.1 && v>-0.1) {	//the range of v shoud be tested 
				Angle(field, robot, bestangle);
				return;
			}
		}
		if (true) {
			vmax = 125;
			if (face) {
				f = -vmax;		//减速度  为  0000000
				v1 = VelocityOne(v1, -f, -f);		//加速一步
				s = v1;
				do {//whether to reduce
					if (v1 > SPEED_ZERO)	//as i said,this is limited
						v1 = VelocityOne(v1, 0, 0);
					else
						v1 = VelocityOne(v1, f, f);
					s += v1;
				} while (v1 > 0);

				s -= v1;

				if (s < distance) {//不满足减速条件加速
					Velocity(field, robot, -f, -f);
				}
				else {
					if (v > SPEED_ZERO)
						Velocity(field, robot, 0, 0);
					else {
						v1 = VelocityOne(v, f, f);		//减速一步
						if (v1 < 0) {
							do {//该降低功率了
								f++;		//f=-vmax;
								v1 = VelocityOne(v, f, f);
							} while (v1 < distance && f < vmax);
						}
						Velocity(field, robot, f, f);
					}
				}
			}
			else {
				f = vmax;		//减速度!!!!!
				v1 = VelocityOne(v1, -f, -f);
				s = v1;
				do {//whether to reduce
					if (v1 < -SPEED_ZERO)	//as i said,this is limited
						v1 = VelocityOne(v1, 0, 0);
					else
						v1 = VelocityOne(v1, f, f);
					s += v1;
				} while (v1 < -0.1);

				s -= v1;

				if (s > -distance) {//不满足减速条件加速
					Velocity(field, robot, -f, -f);
				}
				else {
					if (v < -SPEED_ZERO)
						Velocity(field, robot, 0, 0);
					else {
						v1 = VelocityOne(v, f, f);		//减速一步
						if (v1 > 0) {
							do {//该降低功率了
								f--;		//f=-vmax;
								v1 = VelocityOne(v, f, f);
							} while (v1 > -distance && f > -vmax);
						}
						Velocity(field, robot, f, f);
					}
				}
			}
		}
	}
}

void PositionAndStopX(Field* field, int  robot, Vector3D pos, double Xangle, double limit) {
	Mydata* p;
	p = (Mydata*)field->userData;

	double anglespeedmax = 0;		//控制转交速度的变量
	double vmax = 125;
	double Limitedangle = 2;

	if (limit < 2)
		limit = 2;
	double Limiteddis = limit;

	double  distance;
	double turnangle, posangle, vangle;
	double dx, dy;
	double a = SPEED_A;
	double b = SPEED_B;
	double v, v1;
	double f = vmax;
	double s = 0;
	int n = 0;
	bool face = true;			//判断小车是否是正面前进

	v = sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y * p->my_speed[robot].y);

	dx = pos.x - p->robot[robot].position.x;
	dy = pos.y - p->robot[robot].position.y;

	distance = Distance(p->robot[robot].position, pos);
	posangle = Atan(dy, dx);

	turnangle = p->robot[robot].rotation - posangle;		//think more!!
	RegulateAngle(turnangle);

	if (turnangle > 90) {
		face = false;
		turnangle -= 180;
	}
	else if (turnangle < -90) {
		face = false;
		turnangle += 180;
	}
	else {
		face = true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;
	RegulateAngle(vangle);
	if (vangle < -90 || vangle > 90)
		v = -v;
	v1 = v;

	if (distance > Limiteddis) {//it is too early to count the steps
		if (turnangle > Limitedangle || turnangle < -Limitedangle) {//adjust angle
			/////////////////测试这一段
			if (turnangle > 20 || turnangle < -20)
				anglespeedmax = 0;
			else if (turnangle > 10 || turnangle < -10)
				anglespeedmax = 125;
			else if (turnangle > 5 || turnangle < -5)
				anglespeedmax = 180;
			else
				anglespeedmax = 200;
			///////////////测试这一段
			PAngle(field, robot, posangle, anglespeedmax);
		}
		else {
			if (face)
				Velocity(field, robot, f, f);
			else
				Velocity(field, robot, -f, -f);
		}//it is time to rush
	}
	else {
		if (distance > 1) {		//调整角度	return!!!!!!
			if (turnangle > Limitedangle || turnangle < -Limitedangle) {
				Angle(field, robot, posangle);
				return;
			}
		}
		if (distance < 1) {	//停止并转向		return!!!!!!
			if (v<0.5 && v>-0.5) {
				Velocity(field, robot, -Xangle, Xangle);
				return;
			}
		}
		if (true) {
			vmax = 125;
			if (face) {
				f = -vmax;		//减速度  为  0000000
				v1 = VelocityOne(v1, -f, -f);		//加速一步
				s = v1;
				do {//whether to reduce
					if (v1 > SPEED_ZERO)	//as i said,this is limited
						v1 = VelocityOne(v1, 0, 0);
					else
						v1 = VelocityOne(v1, f, f);
					s += v1;
				} while (v1 > 0);

				s -= v1;

				if (s < distance) {//不满足减速条件加速
					Velocity(field, robot, -f, -f);
				}
				else {
					if (v > SPEED_ZERO)
						Velocity(field, robot, 0, 0);
					else {
						v1 = VelocityOne(v, f, f);		//减速一步
						if (v1 < 0) {
							do {//该降低功率了
								f++;		//f=-vmax;
								v1 = VelocityOne(v, f, f);
							} while (v1 < distance && f < vmax);
						}
						Velocity(field, robot, f, f);
					}
				}
			}
			else {
				f = vmax;		//减速度!!!!!
				v1 = VelocityOne(v1, -f, -f);
				s = v1;
				do {//whether to reduce
					if (v1 < -SPEED_ZERO)	//as i said,this is limited
						v1 = VelocityOne(v1, 0, 0);
					else
						v1 = VelocityOne(v1, f, f);
					s += v1;
				} while (v1 < -0.1);

				s -= v1;

				if (s > -distance) {//不满足减速条件加速
					Velocity(field, robot, -f, -f);
				}
				else {
					if (v < -SPEED_ZERO)
						Velocity(field, robot, 0, 0);
					else {
						v1 = VelocityOne(v, f, f);		//减速一步
						if (v1 > 0) {
							do {//该降低功率了
								f--;		//f=-vmax;
								v1 = VelocityOne(v, f, f);
							} while (v1 > -distance && f > -vmax);
						}
						Velocity(field, robot, f, f);
					}
				}
			}
		}
	}
}

void PositionBallX(Field* field, int  robot, Vector3D pos, double Xangle, double limit) {
	Mydata* p;
	p = (Mydata*)field->userData;

	double anglespeedmax = 0;		//控制转交速度的变量
	double vmax = 125;
	double Limitedangle = 2;

	if (limit < 2.8)
		limit = 2.8;
	double Limiteddis = limit;

	double  distance;
	double turnangle, posangle, vangle;
	double dx, dy;
	double a = SPEED_A;
	double b = SPEED_B;
	double v;
	double f = vmax;
	bool face = true;			//判断小车是否是正面前进
	bool turnornot = false;	//是否旋转,临时变量

	v = sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y * p->my_speed[robot].y);

	dx = pos.x - p->robot[robot].position.x;
	dy = pos.y - p->robot[robot].position.y;

	distance = Distance(p->robot[robot].position, pos);
	posangle = Atan(dy, dx);

	turnangle = p->robot[robot].rotation - posangle;		//think more!!
	RegulateAngle(turnangle);

	if (turnangle > 90) {
		face = false;
		turnangle -= 180;
	}
	else if (turnangle < -90) {
		face = false;
		turnangle += 180;
	}
	else {
		face = true;
	}

	vangle = p->my_speed[robot].z - p->robot[robot].rotation;
	RegulateAngle(vangle);


	if (distance < 3.2)
		turnornot = true;
	else if (distance < 3.5 && v > 0.5)
		turnornot = true;
	else if (distance < 4.5 && v > 0.8)
		turnornot = true;

	if (distance > Limiteddis)	//不在旋转范围内  则不转
		turnornot = false;

	if (turnornot) {//满足条件 转!!!
		Velocity(field, robot, -Xangle, Xangle);
	}//否则跑位
	else if (turnangle > Limitedangle || turnangle < -Limitedangle) {//adjust angle
		/////////////////测试这一段
		if (turnangle > 60 || turnangle < -60)
			anglespeedmax = 0;
		else if (turnangle > 30 || turnangle < -30)
			anglespeedmax = 100;
		else if (turnangle > 10 || turnangle < -10)
			anglespeedmax = 150;
		else
			anglespeedmax = 200;
		///////////////测试这一段
		PAngle(field, robot, posangle, anglespeedmax);

	}
	else {
		if (face)
			Velocity(field, robot, f, f);
		else
			Velocity(field, robot, -f, -f);
	}//it is time to rush
}

void PositionAndThrough(Field* field, int robot, Vector3D pos, double MAX)
{
	Mydata* p;
	p = (Mydata*)field->userData;

	double anglespeedmax = 0;		//控制转交速度的变量
	double max = MAX;
	double Limitedangle = 2;
	double Limiteddis = 0;
	double  distance;
	double turnangle, posangle, vangle;
	double dx, dy;
	double a = SPEED_A;
	double b = SPEED_B;
	double v, v1;
	double f;
	double s = 0;
	int n = 0;
	bool face = true;			//判断小车是否是正面前进

	v = sqrt(p->my_speed[robot].x * p->my_speed[robot].x + p->my_speed[robot].y * p->my_speed[robot].y);

	dx = pos.x - p->robot[robot].position.x;
	dy = pos.y - p->robot[robot].position.y;

	distance = Distance(p->robot[robot].position, pos);
	posangle = Atan(dy, dx);

	turnangle = posangle - p->robot[robot].rotation;		//think more!!
	RegulateAngle(turnangle);

	if (turnangle > 90) {
		face = false;
		turnangle -= 180;
	}
	else if (turnangle < -90) {
		face = false;
		turnangle += 180;
	}
	else {
		face = true;
	}

	vangle = p->my_speed[robot].z - posangle;
	RegulateAngle(vangle);
	if (vangle < -90 || vangle > 90)
		v = -v;
	v1 = v;

	if (distance > Limiteddis) {//it is too early to count the steps
		if (turnangle > Limitedangle || turnangle < -Limitedangle) {//adjust angle
			/////////////////测试这一段
			if (turnangle > 20 || turnangle < -20)
				anglespeedmax = 0;
			else if (turnangle > 10 || turnangle < -10)
				anglespeedmax = 125;
			else if (turnangle > 5 || turnangle < -5)
				anglespeedmax = 180;
			else
				anglespeedmax = 200;
			///////////////测试这一段
			PAngle(field, robot, posangle, anglespeedmax);
		}
		else {
			f = max;
			if (face)
				Velocity(field, robot, f, f);
			else
				Velocity(field, robot, -f, -f);

		}//it is time to rush
	}
	else
	{


	}//abserlutely count
}

/************************踢球动作*********************************/
//1.Kick 让robot把球踢到ToPos的位置
//2.Kick 让robot 以与robot1的角度跑向它 
//3.Kick 向着steps个周期后球的位置踢
//4.shoot 射门
/*****************************************************************/


void Kick(Field* field, int  robot, Vector2 ToPos)
{
	Mydata* p;
	p = (Mydata*)field->userData;

	double LimitedCircle = 3;
	Vector3D ball = Meetball_p(field, robot); //use the predictball position

	Vector3D RobotToBall; //人和球的相对位置
	RobotToBall.x = ball.x - p->robot[robot].position.x;
	RobotToBall.y = ball.y - p->robot[robot].position.y;
	RobotToBall.z = Atan(p->robot[robot].position, ball);

	Vector3D BallToGate; //球和球门的相对位置
	BallToGate.x = ToPos.x - ball.x;
	BallToGate.y = ToPos.y - ball.y;
	BallToGate.z = Atan(ball, ToPos);

	double gateangle = BallToGate.z;

	double RunAngle;
	RunAngle = RobotToBall.z - BallToGate.z;
	RegulateAngle(RunAngle);

	double dis = Distance(ball, p->robot[robot].position);

	if (dis > 3 * LimitedCircle) {
		Vector3D Center;
		if (RunAngle > 0) {
			BallToGate.z -= 90;
		}
		else {
			BallToGate.z += 90;
		}

		RegulateAngle(BallToGate.z);
		Center.x = ball.x + LimitedCircle * cos(BallToGate.z / 180.0);
		Center.y = ball.y + LimitedCircle * sin(BallToGate.z / 180.0);
		Center.z = 0;
		double distance = Distance(Center, p->robot[robot].position);

		if (distance < 2 * LimitedCircle)
		{
			RunAngle = RobotToBall.z + RunAngle / 2; // 可以调整  2 
		}
		else {
			double CenAngle = Atan(p->robot[robot].position, Center);
			if (RunAngle < 0) {
				RunAngle = CenAngle - 180 * LimitedCircle * asin(LimitedCircle / distance) / 3.142;
				RegulateAngle(RunAngle);
			}
			else {
				RunAngle = CenAngle + 180 * LimitedCircle * asin(LimitedCircle / distance) / 3.142;
				RegulateAngle(RunAngle);
			}
		}

	}
	else {


		RunAngle = RobotToBall.z + RunAngle / 2; // 可以调整  2 
		RegulateAngle(RunAngle);
	}
	double paraA = gateangle - p->robot[robot].rotation;
	if (paraA < 0) {
		paraA = -paraA;
	}
	if (paraA > 90) {
		paraA = 180 - paraA;
	}
	if (0.1 > paraA)
	{
		paraA = 0.1;
	}
	double paraB = 125 * dis / 3 * LimitedCircle * 10 / paraA;
	if (paraB > 125) {
		paraB = 125;
	}
	PAngle(field, robot, RunAngle, paraB);
}
void Kick(Field* field, int  robot, Vector3D ToPos)
{
	Mydata* p;
	p = (Mydata*)field->userData;

	double LimitedCircle = 3;
	Vector3D ball = Meetball_p(field, robot); //use the predictball position

	Vector3D RobotToBall; //人和球的相对位置
	RobotToBall.x = ball.x - p->robot[robot].position.x;
	RobotToBall.y = ball.y - p->robot[robot].position.y;
	RobotToBall.z = Atan(p->robot[robot].position, ball);

	Vector3D BallToGate; //球和球门的相对位置
	BallToGate.x = ToPos.x - ball.x;
	BallToGate.y = ToPos.y - ball.y;
	BallToGate.z = Atan(ball, ToPos);

	double gateangle = BallToGate.z;

	double RunAngle;
	RunAngle = RobotToBall.z - BallToGate.z;
	RegulateAngle(RunAngle);

	double dis = Distance(ball, p->robot[robot].position);

	if (dis > 3 * LimitedCircle) {
		Vector3D Center;
		if (RunAngle > 0) {
			BallToGate.z -= 90;
		}
		else {
			BallToGate.z += 90;
		}

		RegulateAngle(BallToGate.z);
		Center.x = ball.x + LimitedCircle * cos(BallToGate.z / 180.0);
		Center.y = ball.y + LimitedCircle * sin(BallToGate.z / 180.0);
		Center.z = 0;
		double distance = Distance(Center, p->robot[robot].position);

		if (distance < 2 * LimitedCircle)
		{
			RunAngle = RobotToBall.z + RunAngle / 2; // 可以调整  2 
		}
		else {
			double CenAngle = Atan(p->robot[robot].position, Center);
			if (RunAngle < 0) {
				RunAngle = CenAngle - 180 * LimitedCircle * asin(LimitedCircle / distance) / 3.142;
				RegulateAngle(RunAngle);
			}
			else {
				RunAngle = CenAngle + 180 * LimitedCircle * asin(LimitedCircle / distance) / 3.142;
				RegulateAngle(RunAngle);
			}
		}

	}
	else {


		RunAngle = RobotToBall.z + RunAngle / 2; // 可以调整  2 
		RegulateAngle(RunAngle);
	}
	double paraA = gateangle - p->robot[robot].rotation;
	if (paraA < 0) {
		paraA = -paraA;
	}
	if (paraA > 90) {
		paraA = 180 - paraA;
	}
	if (0.1 > paraA)
	{
		paraA = 0.1;
	}
	double paraB = 125 * dis / 3 * LimitedCircle * 10 / paraA;
	if (paraB > 125) {
		paraB = 125;
	}
	PAngle(field, robot, RunAngle, paraB);
}

void Kick(Field* field, int  robot, int robot1) {//踢人
	Mydata* p;
	p = (Mydata*)field->userData;
	Vector3D RobotToBall;		//人和球的相对位置
	RobotToBall.x = p->robot[robot1].position.x - p->robot[robot].position.x;
	RobotToBall.y = p->robot[robot1].position.y - p->robot[robot].position.y;
	RobotToBall.z = Atan(p->robot[robot].position, p->robot[robot1].position);

	Vector3D BallToGate;		//球和球门的相对位置
	BallToGate.x = CONSTGATE.x - p->robot[robot1].position.x;
	BallToGate.y = CONSTGATE.y - p->robot[robot1].position.y;
	BallToGate.z = Atan(p->robot[robot1].position, CONSTGATE);

	double RunAngle;
	RunAngle = RobotToBall.z - BallToGate.z;
	RegulateAngle(RunAngle);

	RunAngle = RobotToBall.z + RunAngle / 2;	// 可以调整  2 
	RegulateAngle(RunAngle);

	PAngle(field, robot, RunAngle, 125);
}

void Kick(Field* field, int robot, int steps, double limits) {
	Mydata* p = (Mydata*)field->userData;
	double dx, dy, angle;

	dx = p->ball_cur.x - p->robot[robot].position.x;
	dy = p->ball_cur.y - p->robot[robot].position.y;
	angle = Atan(dy, dx);
	PredictBall(field, steps);

	if (angle<90 && angle>-90) {
		if (p->ball_cur.y > 41.8)
			PositionBallX(field, robot, p->ball_pre, -125, 3);
		else
			PositionBallX(field, robot, p->ball_pre, 125, 3);
	}
	else
		shoot(field, robot);
}

void shoot(Field* field, int robot) {
	Mydata* p = (Mydata*)field->userData;
	double w1, w2, alfa;
	double dx, dy;
	/*改过*/
	if (p->ball_cur.y > GBOT && p->ball_cur.y <= (GTOP + GBOT) / 2) {
		if (p->ball_speed.z > 85 && p->ball_speed.z < 95)
			PositionBallX(field, robot, p->ball_cur, -90, 4);
		else if (p->ball_speed.z<-85 && p->ball_speed.z>-95)
			PositionBallX(field, robot, p->ball_cur, 90, 4);
	}
	else if (p->ball_cur.y >= (GTOP + GBOT) / 2 && p->ball_cur.y <= GTOP) {
		if (p->ball_speed.z > 85 && p->ball_speed.z < 95)
			PositionBallX(field, robot, p->ball_cur, -90, 4);
		else if (p->ball_speed.z<-85 && p->ball_speed.z>-95)
			PositionBallX(field, robot, p->ball_cur, 90, 4);
	}

	if (p->robot[robot].position.x <= p->ball_cur.x) {
		PredictBall(field, 2);
		dx = GRIGHT - p->robot[robot].position.x;
		dy = GTOP - p->robot[robot].position.y;
		w1 = Atan(dy, dx);

		dx = GRIGHT - p->robot[robot].position.x;
		dy = GBOT - p->robot[robot].position.y;
		w2 = Atan(dy, dx);

		dx = p->ball_pre.x - p->robot[robot].position.x;
		dy = p->ball_pre.y - p->robot[robot].position.y;
		alfa = Atan(dy, dx);

		if ((w1 - alfa) * (w2 - alfa) <= 0)
			PAngle(field, robot, alfa, 125);
		else if (p->ball_cur.y < GBOT + 4 && p->robot[robot].position.y < GBOT + 4)
			Kick(field, robot, BOTGATE);
		else if (p->ball_cur.y > GTOP - 4 && p->robot[robot].position.y > GBOT - 4)
			Kick(field, robot, TOPGATE);
		else
			Kick(field, robot, CONSTGATE);
	}
	else
		Kick(field, robot, CONSTGATE);
}

/************************防止犯规*********************************/
//1.判断robot队员和球的距离是否再LENGTH规定的范围内返回true  or false
/*****************************************************************/

bool Within(Field* field, int robot, double LENGTH) {
	Mydata* p;
	p = (Mydata*)field->userData;

	const double steps = 50;
	int who = robot;
	double dis;

	Vector3D ballgo = { 0,0,0 };
	Vector3D robotgo = { 0,0,0 };
	Vector3D ball = p->ball_cur;

	ballgo.x = ball.x + steps * p->my_speed[who].x;
	ballgo.y = ball.y + steps * p->my_speed[who].y;

	dis = Distance(ballgo, p->robot[robot].position);

	if (dis < LENGTH) {
		return true;
	}
	return false;
}


//////////////////////比赛状态/////////////////////
void NormalGame(Field* field) {
	Mydata* p;
	p = (Mydata*)field->userData;
	PredictBall(field, 2);
	if (p->ball_speed.x < 0.5 && p->ball_speed.y < 0, 5) Kick(field, 3, p->ball_pre);
	Vector3D pos, begin, end;
	static int flag = 1;
	int i, count, x;
	double alfa;

	if (flag == 1) {
		PredictBall(field, 2);
		Kick(field, 3, p->ball_pre);
		Kick(field, 4, p->ball_pre);
		flag++;
	}
	Keeper(field, 0);

	if (p->ball_cur.x >= 50 && p->ball_cur.x < 61)
		Kick(field, 1, CONSTGATE);
	else if (p->ball_cur.x >= 61 && p->ball_cur.x < 78.6) {
		if (p->ball_cur.y < 27.8) {
			pos.x = 37.2;
			pos.y = 20;
		}
		else if (p->ball_cur.y <= 58.5) {
			PredictBall(field, 2);
			pos.x = 37.2;
			pos.y = p->ball_pre.y;
		}
		else {
			pos.x = 37.2;
			pos.y = 66.2;
		}
		PositionAndStop(field, 1, pos);
	}
	else  if (p->ball_cur.x > 78.6 && ((p->ball_cur.y < 34 && p->ball_cur.y>6.3) || (p->ball_cur.y > 48 && p->ball_cur.y < 77.2))) {
		Order(field);
		count = 120;
		while (count > 0) {
			Kick(field, 1, p->Attacker);
			count--;
		}
	}
	switch (p->WIB) {
	case 1:
		Order(field);
		//ActiveAttacker,Attacker
		pos.x = 50;
		pos.y = 9;
		Kick(field, p->ActiveAttacker, pos);
		Kick(field, p->Attacker, pos);
		//Defender
		pos.x = 19;
		pos.y = 58;
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
				break;
		}
		if (i < 5)
			PositionAndThrough(field, p->Defender, p->opp[i].position, 125);
		else {
			pos.x = 20;
			pos.y = 58;
			PositionAndStop(field, p->Defender, pos, -145);
		}
		//NegativeAttacker
		pos.x = 19;
		pos.y = 27;
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
				break;
		}
		if (i < 5)
			PositionAndThrough(field, p->NegativeAttacker, p->opp[i].position, 125);
		else {
			pos.x = 20;
			pos.y = 27;
			PositionAndStop(field, p->NegativeAttacker, pos, -145);
		}
		break;
	case 2:
		Order(field);
		//Defender
		pos.x = 19;
		pos.y = 58;
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
				break;
		}
		if (i < 5)
			PositionAndThrough(field, p->Defender, p->opp[i].position, 125);
		else {
			pos.x = 20;
			pos.y = 58;
			PositionAndStop(field, p->Defender, pos, -145);
		}

		//小禁区加上下两小块
		if (p->ball_cur.x < SRG_LEFT + 0.8 && p->ball_cur.y<BRG_TOP + 0.8 && p->ball_cur.y>BRG_BOT - 0.8) {
			//ActiveAttacker
			pos.x = 8.9;
			pos.y = 58;
			for (i = 1; i < 5; i++) {
				if (fabs(p->opp[i].position.x - pos.x) < 2 && fabs(p->opp[i].position.y - pos.y) < 8) break;
			}
			if (i < 5)
				PositionAndThrough(field, p->ActiveAttacker, p->opp[i].position, 125);
			else {
				pos.x = BRG_LEFT;
				pos.y = SRG_TOP;
				PositionAndStop(field, p->ActiveAttacker, pos, -145);
			}
			//Attacker
			pos.x = 8.9;
			pos.y = 27;
			for (i = 1; i < 5; i++) {
				if (fabs(p->opp[i].position.x - pos.x) < 2 && fabs(p->opp[i].position.y - pos.y) < 8) break;
			}
			if (i < 5)
				PositionAndThrough(field, p->Attacker, p->opp[i].position, 125);
			else {
				pos.x = BRG_LEFT;
				pos.y = BD_BOT;
				PositionAndStop(field, p->Attacker, pos, 145);
			}
			//NegativeAttacker
			pos.x = 19;
			pos.y = 27;
			for (i = 1; i < 5; i++) {
				if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
					break;
			}
			if (i < 5)
				PositionAndThrough(field, p->NegativeAttacker, p->opp[i].position, 125);
			else {
				pos.x = 20;
				pos.y = 27;
				PositionAndStop(field, p->NegativeAttacker, pos, 145);
			}
		}
		//其他
		else {
			//ActiveAttacker,Attacker
			pos.x = 50;
			pos.y = 9;
			Kick(field, p->ActiveAttacker, pos);
			Kick(field, p->Attacker, pos);

			//NegativeAttacker
			pos.x = SRG_LEFT;
			pos.y = p->ball_cur.y;
			PositionAndStop(field, p->NegativeAttacker, pos);
		}
		break;

	case 3:
		Order(field);
		//Defender
		pos.x = 19;
		pos.y = 27;
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
				break;
		}
		if (i < 5)	PositionAndThrough(field, p->Defender, p->opp[i].position, 125);
		else {
			pos.x = 20;
			pos.y = 27;
			PositionAndStop(field, p->Defender, pos, 145);
		}
		//小禁区
		if (p->ball_cur.x < SRG_LEFT + 0.8 && p->ball_cur.y<BRG_TOP + 0.8 && p->ball_cur.y>BRG_BOT - 0.8) {
			//ActiveAttacker
			pos.x = 8.9;
			pos.y = 27;
			for (i = 1; i < 5; i++) {
				if (fabs(p->opp[i].position.x - pos.x) < 2 && fabs(p->opp[i].position.y - pos.y) < 8) break;
			}
			if (i < 5)
				PositionAndThrough(field, p->ActiveAttacker, p->opp[i].position, 125);
			else {
				pos.x = BRG_LEFT;
				pos.y = BD_BOT;
				PositionAndStop(field, p->ActiveAttacker, pos, 145);
			}
			//Attacker
			pos.x = 8.9;
			pos.y = 58;
			for (i = 1; i < 5; i++) {
				if (fabs(p->opp[i].position.x - pos.x) < 2 && fabs(p->opp[i].position.y - pos.y) < 8) break;
			}
			if (i < 5)
				PositionAndThrough(field, p->Attacker, p->opp[i].position, 125);
			else {
				pos.x = BRG_LEFT;
				pos.y = BD_TOP;
				PositionAndStop(field, p->Attacker, pos, -145);
			}
			pos.x = 19;
			pos.y = 58;
			for (i = 1; i < 5; i++) {
				if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
					break;
			}
			if (i < 5)
				PositionAndThrough(field, p->NegativeAttacker, p->opp[i].position, 125);
			else {
				pos.x = 20;
				pos.y = 27;
				PositionAndStop(field, p->NegativeAttacker, pos, -145);
			}

		}
		//其他
		else {
			//ActiveAttacker,Attacker
			pos.x = 50;
			pos.y = 74;
			Kick(field, p->ActiveAttacker, pos);
			Kick(field, p->Attacker, pos);
			//NegativeAttacker
			pos.x = SRG_LEFT;
			pos.y = p->ball_cur.y;
			PositionAndStop(field, p->NegativeAttacker, pos);

			if (p->ball_cur.x < 12 && p->ball_cur.y < 59)
				PositionAndThrough(field, p->NegativeAttacker, p->robot[0].position);
			else if (p->ball_cur.x > GLEFT + 4 && p->ball_cur.x < 17) {
				pos.x = 9;
				pos.y = 77;
				Kick(field, p->NegativeAttacker, pos);
			}
			else {
				pos.x = 20;
				pos.y = p->robot[0].position.y;
				PositionAndStop(field, p->NegativeAttacker, pos);
			}
		}
		break;
	case 4:
		Order(field);
		//ActiveAttacker,Attacker
		pos.x = 50;
		pos.y = 74;
		Kick(field, p->ActiveAttacker, pos);
		Kick(field, p->Attacker, pos);
		//Defender		
		pos.x = 19;
		pos.y = 27;
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
				break;
		}
		if (i < 5)
			PositionAndThrough(field, p->Defender, p->opp[i].position, 125);
		else {
			pos.x = 20;
			pos.y = 27;
			PositionAndStop(field, p->Defender, pos, 145);
		}
		//NegativeAttacker
		pos.x = 19;
		pos.y = 58;
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - pos.x) < 15 && fabs(p->opp[i].position.y - pos.y) < 15)
				break;
		}
		if (i < 5)
			PositionAndThrough(field, p->NegativeAttacker, p->opp[i].position, 125);
		else {
			pos.x = 20;
			pos.y = 27;
			PositionAndStop(field, p->NegativeAttacker, pos, -145);
		}
		break;
	case 5:
	case 6:
		Order(field);
		/*改过7.18
			x=WhoseBall(field);
			if(p->whoseBall==2){
				 Sweep(field,p->Attacker);
				 for(i=0;i<5;i++) if(Distance(p->robot[p->Attacker].position,p->opp[i].position)<5) break;
				 if(i<5) PositionAndThrough(field,p->ActiveAttacker,p->opp[i].position);
				 else Kick(field,p->ActiveAttacker,p->Attacker);
			}
			else{
				Kick(field,p->ActiveAttacker,1,1.5);
				Kick(field,p->Attacker,3,1.5);
			}
		改过7.18*/
		Kick(field, p->ActiveAttacker, 1, 1.5);
		Kick(field, p->Attacker, 3, 1.5);
		pos.x = 19;
		pos.y = 58;
		PositionAndStop(field, p->Defender, pos, -135);
		/*改过2011.7.17*/
	//NegativeAttacker
		PredictBall(field, 10);
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - p->ball_pre.x) < 15 && fabs(p->opp[i].position.y - p->ball_pre.y) < 15)
				break;
		}
		PositionAndThrough(field, p->NegativeAttacker, pos, 135);
		break;
	case 7:
	case 8:
		Order(field);
		/*改过7.18
			x=WhoseBall(field);
			if(p->whoseBall==2){
				 Sweep(field,p->Attacker);
				 for(i=0;i<5;i++) if(Distance(p->robot[p->Attacker].position,p->opp[i].position)<5) break;
				 if(i<5) PositionAndThrough(field,p->ActiveAttacker,p->opp[i].position);
				 else Kick(field,p->ActiveAttacker,p->Attacker);
			}
			else{
				Kick(field,p->ActiveAttacker,1,1.5);
				Kick(field,p->Attacker,3,1.5);
			}
		改过7.18*/
		Kick(field, p->ActiveAttacker, 1, 1.5);
		Kick(field, p->Attacker, 3, 1.5);
		pos.x = 20;
		pos.y = 27;
		PositionAndStop(field, p->Defender, pos, 135);
		/*改过2011.7.17*/
	//NegativeAttacker
		PredictBall(field, 10);
		for (i = 1; i < 5; i++) {
			if (fabs(p->opp[i].position.x - p->ball_pre.x) < 15 && fabs(p->opp[i].position.y - p->ball_pre.y) < 15)
				break;
		}
		PositionAndThrough(field, p->Defender, pos, 135);
		break;
	case 9:
		Order(field);
		//ActiveAttacker
		shoot(field, p->ActiveAttacker);
		//Attacker
		if (p->ball_cur.x > p->robot[p->ActiveAttacker].position.x && p->ball_cur.y < 10) {
			count = 120;
			while (count > 0) {
				Kick(field, p->Attacker, p->ActiveAttacker);
				count--;
			}
		}
		else {
			PredictBall(field, 2);
			PositionAndStop(field, p->Attacker, p->ball_pre);
			pos.x = 77;
			pos.y = 57.5;
			PositionAndStop(field, p->NegativeAttacker, pos, -45);
		}
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 10:
		Order(field);
		shoot(field, p->ActiveAttacker);
		PredictBall(field, 2);
		PositionAndStop(field, p->Attacker, p->ball_pre);
		pos.x = 77;
		pos.y = 57.5;
		PositionAndStop(field, p->NegativeAttacker, pos, -45);
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 11:
		Order(field);
		shoot(field, p->ActiveAttacker);
		PredictBall(field, 2);
		PositionAndStop(field, p->Attacker, p->ball_pre);
		pos.x = 77;
		pos.y = 26;
		PositionAndStop(field, p->NegativeAttacker, pos, -45);
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 12:
		Order(field);
		shoot(field, p->ActiveAttacker);
		if (p->ball_cur.x > p->robot[p->ActiveAttacker].position.x && p->ball_cur.y > 74.2) {
			count = 120;
			while (count > 0) {
				Kick(field, p->Attacker, p->ActiveAttacker);
				count--;
			}
		}
		else {
			PredictBall(field, 2);
			PositionAndStop(field, p->Attacker, p->ball_pre);
			pos.x = 77;
			pos.y = 26;
			PositionAndStop(field, p->NegativeAttacker, pos, -45);
		}
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 13:
		Order(field);
		//ActiveAttacke
		shoot(field, p->ActiveAttacker);
		//Attacker
		if (p->ball_cur.x < 92) {
			count = 120;
			while (count > 0) {
				Kick(field, p->Attacker, p->ActiveAttacker);
				count--;
			}
		}
		else
			shoot(field, p->Attacker);
		//NegativeAttacker
		if (p->ball_cur.x < 87 || p->ball_cur.y < 20) {
			count = 120;
			while (count > 0) {
				Kick(field, p->NegativeAttacker, p->Attacker);
				count--;
			}
		}
		else {
			pos.x = 77;
			pos.y = 57.5;
			PositionAndStop(field, p->NegativeAttacker, pos, -45);
		}
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 14:
		Order(field);
		//NegativeAttacker
		if (p->ball_cur.y > GBOT) {
			PredictBall(field, 2);
			begin = p->robot[p->NegativeAttacker].position;
			end = p->ball_pre;
			alfa = Atan(begin, end);
			PAngle(field, p->NegativeAttacker, alfa, 125);
		}
		else {
			pos.x = 77;
			pos.y = 57.5;
			PositionAndStop(field, p->NegativeAttacker, pos, -45);
		}
		//ActiveAttacker
		shoot(field, p->ActiveAttacker);
		//Attacker
		if (p->ball_cur.y < 34) {
			count = 120;
			while (count > 0) {
				Kick(field, p->Attacker, p->ActiveAttacker);
				count--;
			}
		}
		else {
			if (p->ball_speed.y > 1) {
				//PositionBallX(field,p->ActiveAttacker,p->robot[p->ActiveAttacker].position,-100,1);
				PositionBallX(field, p->Attacker, p->robot[p->ActiveAttacker].position, -100, 1);
			}
			else {
				//PositionBallX(field,p->ActiveAttacker,p->robot[p->ActiveAttacker].position,-70,1);
				PositionBallX(field, p->Attacker, p->robot[p->ActiveAttacker].position, -70, 1);

			}
			Kick(field, p->Attacker, CONSTGATE);
		}
		/*改过*/
	 //"Defender"
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 15:
		Order(field);
		if (p->ball_cur.y < GTOP) {
			PredictBall(field, 2);
			begin = p->robot[p->NegativeAttacker].position, end = p->ball_pre;
			alfa = Atan(begin, end);
			PAngle(field, p->NegativeAttacker, alfa, 125);
		}
		else {
			pos.x = 77; pos.y = 26;
			PositionAndStop(field, p->NegativeAttacker, pos, -45);
		}
		shoot(field, p->ActiveAttacker);
		if (p->ball_cur.y > 48) {
			count = 120;
			while (count > 0) {
				Kick(field, p->Attacker, p->ActiveAttacker);
				count--;
			}
		}
		else {
			if (p->ball_speed.y > 1) {
				//PositionBallX(field,p->ActiveAttacker,p->robot[p->ActiveAttacker].position,100,1);
				PositionBallX(field, p->Attacker, p->robot[p->ActiveAttacker].position, 100, 1);
			}
			else {
				//PositionBallX(field,p->ActiveAttacker,p->robot[p->ActiveAttacker].position,70,1);
				PositionBallX(field, p->Attacker, p->robot[p->ActiveAttacker].position, 70, 1);
			}
			Kick(field, p->Attacker, CONSTGATE);
		}
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	case 16:
		Order(field);
		shoot(field, p->ActiveAttacker);
		if (p->ball_cur.x < 92) {
			count = 120;
			while (count > 0) {
				Kick(field, p->Attacker, p->ActiveAttacker);
				count--;
			}
		}
		else
			shoot(field, p->Attacker);

		if (p->ball_cur.x < 87 || p->ball_cur.y>66) {
			count = 100;
			while (count > 0) {
				Kick(field, p->NegativeAttacker, p->Attacker);
				count--;
			}
		}
		else {
			pos.x = 77; pos.y = 26;
			PositionAndStop(field, p->NegativeAttacker, pos, 45);
		}
		/*改过*/
		for (i = 1; i < 5; i++) {
			if (i != p->ActiveAttacker && i != p->Attacker && i != p->NegativeAttacker) break;
		}
		PredictBall(field, 4);
		pos.x = 50;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, i, pos);
		break;
	}
}

/**************************策略*********************************/
//keeper 守门员守门
//
//点球
//球门球
//争球
//任意球
/***************************************************************/

void Keeper(Field* field, int robot)
{//先校正姿态，再去拦球
	Mydata* p;
	p = (Mydata*)field->userData;
	Vector3D go;
	double OX = GLEFT - (GTOP - GBOT);	// 该点为球门中心 向后移动半个球门
	double OY = (GTOP + GBOT) / 2;			//球门中心	
	double ballx = p->ball_cur.x;
	double bally = p->ball_cur.y;
	double gx = p->robot[robot].position.x;
	double gx_outline = GLEFT + 2.2;		//对x坐标的限定，防止离球门线太远了
	double gx_inline = GLEFT - 1;
	double gy = p->robot[robot].position.y;		//跑位点,初值为当前位置
	double goalline = GLEFT + 3;
	bool notout = true;
	bool   standby = true;	//限制x 坐标
	bool   XX = false;	//是否旋转

	if (ballx < GLEFT + 0.7 && bally<BD_BOT + 0.75 && bally>BD_BOT + 0.7 && p->robot[robot].position.y<BD_BOT + 2 && p->robot[robot].position.y>BD_BOT + 1.4 && Distance(p->ball_cur, p->robot[robot].position) < 2.4) {
		Velocity(field, robot, -125, 125);
	}
	else if (ballx < GLEFT + 0.7 && bally<BD_TOP + 0.75 && bally<BD_TOP - 0.7 && p->robot[robot].position.y>BD_TOP - 2 && p->robot[robot].position.y>BD_TOP - 1.4 && Distance(p->ball_cur, p->robot[robot].position) < 2.4) {
		Velocity(field, robot, 125, -125);
	}
	gy = OY + (goalline - OX) * (bally - OY) / (ballx - OX);
	if (notout) {
		if (gy > GTOP + 3)
			gy = GTOP + 3;
		else if (gy < GBOT - 3)
			gy = GBOT - 3;
	}
	if (standby) {
		if (gx > gx_outline)
			gx = gx_outline;
		else if (gx < gx_inline)
			gx = gx_inline;
	}
	if (fabs(p->ball_speed.y) < 10 && p->ball_cur.x < 15 && p) {
		if (p->ball_cur.y > BD_TOP - 3) {
			gx = GLEFT - 0.5;
			gy = BD_TOP - 0.7;
		}
		else if (p->ball_cur.y < BD_BOT + 3) {
			gx = GLEFT - 0.5;
			gy = BD_BOT + 0.7;
		}
		else  gx = GLEFT + 2.5;
	}
	else gx = GLEFT + 2.5;
	go.x = gx;
	go.y = gy;
	GoaliePosition(field, robot, go, 90, 1.5);
}

void Order(Field* field)
{
	Mydata* p;
	p = (Mydata*)field->userData;
	int i, j, k, a[4];
	double dis[4], b[4], temple;
	double dy = 0;
	static int record = 2;

	b[0] = dis[0] = Distance(p->robot[1].position, p->ball_cur);
	b[1] = dis[1] = Distance(p->robot[2].position, p->ball_cur);
	b[2] = dis[2] = Distance(p->robot[3].position, p->ball_cur);
	b[3] = dis[3] = Distance(p->robot[4].position, p->ball_cur);

	switch (p->WIB) {
	case 1:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
		for (i = 0; i < 3; i++)
			for (j = i + 1; j < 4; j++) {
				if (b[i] > b[j]) {
					temple = b[i];
					b[i] = b[j];
					b[j] = temple;
				}
			}
		for (i = 0; i < 4; i++)
			for (j = 0; j < 4; j++)
				if (dis[j] == b[i])
					a[i] = j + 1;

		p->ActiveAttacker = a[0];
		p->Attacker = a[1];
		p->NegativeAttacker = a[2];
		p->Defender = a[3];
		break;
	case 2:
	case 3:
		if (p->ball_cur.y > p->robot[0].position.y) {
			i = p->ActiveAttacker;
			p->ActiveAttacker = p->NegativeAttacker;
			p->NegativeAttacker = i;
		}
		break;
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:

		if (dis[1] < dis[2]) {

			if (dis[2] <= dis[3]) {
				i = 2;
				j = 3;
				k = 4;
			}
			else if (dis[1] <= dis[3]) {
				i = 2;
				j = 4;
				k = 3;
			}
			else {
				i = 4;
				j = 2;
				k = 3;
			}
		}
		else {

			if (dis[1] <= dis[3]) {
				i = 3;
				j = 2;
				k = 4;
			}
			else if (dis[2] <= dis[3]) {
				i = 3;
				j = 4;
				k = 2;
			}
			else {
				i = 4;
				j = 3;
				k = 2;
			}
		}

		p->ActiveAttacker = i;
		p->Attacker = j;
		p->NegativeAttacker = k;
		record = p->NegativeAttacker;
		break;
	case 14:
		if (!Within(field, record, 1))
			p->NegativeAttacker = record;

		else {
			if (p->ball_speed.z < 0) {
				dy = 100;
				for (i = 2; i < 5; i++)
					if (p->robot[i].position.y < dy) {
						dy = p->robot[i].position.y;
						record = i;
					}
				p->ActiveAttacker = p->NegativeAttacker;
				p->NegativeAttacker = record;
			}
			else {
				dy = 0;
				for (i = 2; i < 5; i++)
					if (p->robot[i].position.y > dy) {
						dy = p->robot[i].position.y;
						record = i;
					}
				p->ActiveAttacker = p->NegativeAttacker;
				p->NegativeAttacker = record;
			}
		}

		for (i = 2; i < 5; i++)
			if (i != p->NegativeAttacker && i != p->ActiveAttacker)
				p->Attacker = i;

		break;
	case 15:

		if (!Within(field, record, 1))
			p->NegativeAttacker = record;

		else {
			if (p->ball_speed.z < 0) {
				dy = 100;
				for (i = 2; i < 5; i++)
					if (p->robot[i].position.y < dy) {
						dy = p->robot[i].position.y;
						record = i;
					}
				p->ActiveAttacker = p->NegativeAttacker;
				p->NegativeAttacker = record;
			}
			else {
				dy = 0;
				for (i = 2; i < 5; i++)
					if (p->robot[i].position.y > dy) {
						dy = p->robot[i].position.y;
						record = i;
					}
				p->ActiveAttacker = p->NegativeAttacker;
				p->NegativeAttacker = record;
			}
		}
		for (i = 2; i < 5; i++)
			if (i != p->NegativeAttacker && i != p->ActiveAttacker)
				p->Attacker = i;

		break;
	case 16:

		if (dis[1] < dis[2]) {

			if (dis[2] <= dis[3]) {
				i = 2;
				j = 3;
				k = 4;
			}
			else if (dis[1] <= dis[3]) {
				i = 2;
				j = 4;
				k = 3;
			}
			else {
				i = 4;
				j = 2;
				k = 3;
			}
		}
		else {

			if (dis[1] <= dis[3]) {
				i = 3;
				j = 2;
				k = 4;
			}
			else if (dis[2] <= dis[3]) {
				i = 3;
				j = 4;
				k = 2;
			}
			else {
				i = 4;
				j = 3;
				k = 2;
			}
		}

		p->ActiveAttacker = i;
		p->Attacker = j;
		p->NegativeAttacker = k;
		record = p->NegativeAttacker;
		break;
	}
}



void FreeBallGame(Field* field) {//争球
	Mydata* p;
	p = (Mydata*)field->userData;

}
void PlaceBallGame(Field* field) {//开中场球
	Mydata* p;
	p = (Mydata*)field->userData;
	Vector3D pos;
	pos.x = (GLEFT + GRIGHT) / 2 - 10;
	pos.y = 41.8060;
	PredictBall(field, 10);
	PositionAndStop(field, 3, pos);
	Kick(field, 4, CONSTGATE);
	PositionAndThrough(field, 1, p->ball_pre);
	Keeper(field, 0);
}

void PenaltyBallGame(Field* field) {//点球
	Mydata* p;
	p = (Mydata*)field->userData;
	shoot(field, 4);
}
void FreeKickGame(Field* field) {//任意球
	Mydata* p;
	p = (Mydata*)field->userData;
	shoot(field, 4);

}
void GoalKickGame(Field* field) {//门球
	Mydata* p;
	p = (Mydata*)field->userData;
	shoot(field, 0);
}

void Sweep(Field* field, int robot) {
	Mydata* p;
	p = (Mydata*)field->userData;
	Vector3D pos;
	int add_time = 0;
	pos.x = p->ball_cur.x;
	pos.y = p->ball_cur.y;
	add_time = (int)(fabs(Distance(p->robot[robot].position, pos) - 3.5) / 1.2);
	if (add_time > 18) {
		PredictBall(field, 20);
		pos.x = p->ball_pre.x;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, robot, pos);
	}
	else if (add_time == 0) {
		pos.x = p->ball_cur.x;
		pos.y = p->ball_cur.y;
		PositionAndStop(field, robot, pos);
	}
	else
	{
		PredictBall(field, add_time - 1);
		pos.x = p->ball_pre.x;
		pos.y = p->ball_pre.y;
		PositionAndStop(field, robot, pos);
	}
}

int WhoseBall(Field* field) {
	Mydata* p;
	p = (Mydata*)field->userData;
	double temp1, temp2, dis;
	int k1, k2;

	temp1 = Distance(p->ball_cur, p->robot[1].position);
	k1 = 1;
	for (int i = 2; i < 5; i++) {
		dis = Distance(p->ball_cur, p->robot[i].position);
		if (dis < temp1) {
			temp1 = dis;
			k1 = i;
		}
	}

	temp2 = Distance(p->ball_cur, p->opp[1].position);
	k1 = 1;
	for (int i = 2; i < 5; i++) {
		dis = Distance(p->ball_cur, p->opp[i].position);
		if (dis < temp2) {
			temp2 = dis;
			k2 = i;
		}
	}
	if (temp1 <= temp2) { p->whoseBall = 1; return k1; }
	else { p->whoseBall = 2; return k2; }
}




void Action(Field* field) {

	Mydata* p;
	p = (Mydata*)field->userData;
	switch (whichType) {
	case 0:
		NormalGame(field);
		break;
	case 1:
		GoalKickGame(field);

		break;
	case 2:
		PenaltyBallGame(field);
		//PlaceBallGame(field);
		break;
	case 3:
		//FreeBallGame(field);
		//break;
	case 4:
		//	FreeKickGame(field);
			//break;
	case 5:

	case 6:
		FreeBallGame(field);
		break;
	}
}
