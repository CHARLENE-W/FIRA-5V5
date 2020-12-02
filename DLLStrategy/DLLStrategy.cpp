// DLLStrategy.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "platform.h"
#include "adapter.h"

using namespace Simuro;
using namespace Adapter;
JudgeType whichType;


typedef struct
{
	Vector2 myoldpos[5];	
	Vector2 MyOldPos[30][5];
	Vector2 opoldpos[5];	
	int TEAM;
}Mydata;
double MID = 0;
int TIMECOUNTER = 0;
double Gate = 90;
const double FTOP = 90;
const double FBOT = -90;
const double GTOPY = 20;
const double GBOTY = -20;
const double GRIGHT = 125;
const double GLEFT = -125;
const double FRIGHTX = 110;
const double FLEFTX = -110;
int IsBallBeforeDoor = 0, FuZhuShoot = 0;

/*ADD*/
const double PI = 3.141592;
double DISPLACEMENT[6] = { 0 };
double COUNT1 = 0, COUNT2 = 0;//保存调用次数
const double MAXL = 112;
int NEEDROTATE[5] = { 1,1,1,1,1 }; //1 need，else not need
int EV[6] = { 0 }; //estimate V估计的机器人的速度and the ball
double TRACE[6][2][2] = { -1,-1,-1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1 };
double PBP[2] = { 0,0 };

//射门锁
int dirShootLock[5] = { 0 };
Vector2 dirShootPos[5] = { 0 };

void avoidance(Field* field, int id);
//计算用
double Atan(double y, double x);
double Atan(Vector2 begin, Vector2 end);
double Distance(Vector2 p1, Vector2 p2);
double Distance(double x1, double x2, double y1, double y2);
void PredictBall2(double s, Field* field);
double F(double k, double x0, double y0, double x);

//更新全局变量
void estimateV(Field* field);

//基础动作
void Velocity(Robot* robot, int vl, int vr);
double RotateTo(Robot* robot, int rID, const double desX, const double desY);
void to(Robot* robot, int RID, double x, double y);
void go(Robot* robot, int rID, const double x, const double y);
void Position(Robot* robot, double x, double y);
void RightWing(Field* pEnv, int id);
/*ADD*/

//进攻
double KTOP, KBTO;
void Attack(Field* field);
void LineAttack(Field* field);


void attack(int robot1, int robot2, int robot3, Field* field);
int pos(Vector2 pos);

void dirShoot(Field* field, int id);

/*main*/
void OnEvent(EventType type, void* argument) {
	SendLog(L"V/DLLStrategy:OnEvent()");

	switch (type) {
	case EventType::FirstHalfStart: {
		SendLog(L"First Half Start");
		break;
	}
	case EventType::SecondHalfStart: {
		SendLog(L"Second Half Start");
		break;
	}
	case EventType::OvertimeStart: {
		SendLog(L"Overtime Start");
		break;
	}
	case EventType::PenaltyShootoutStart: {
		SendLog(L"Penalty Shootout Start");
		break;
	}
	case EventType::JudgeResult: {
		JudgeResultEvent* judgeResult = static_cast<JudgeResultEvent*>(argument);
		whichType = judgeResult->type;
		switch (judgeResult->type) {
		case JudgeType::PlaceKick:
			SendLog(L"Place Kick");
			break;
		case JudgeType::PenaltyKick:
			SendLog(L"Penalty Kick");
			break;
		case JudgeType::GoalKick:
			SendLog(L"Goal Kick");
			break;
		case JudgeType::FreeKickLeftBot:
		case JudgeType::FreeKickLeftTop:
		case JudgeType::FreeKickRightBot:
		case JudgeType::FreeKickRightTop:
			SendLog(L"Free Kick");
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
	static const wchar_t teamName[] = L"TEST 3";
	static constexpr size_t len = sizeof(teamName);
	memcpy(teamInfo->teamName, teamName, len);
}

void GetInstruction(Field* field) {
	SendLog(L"V/DLLStrategy:GetInstruction()");
	//attack(1, 2, 3, field);
	dirShoot(field, 4);
	//go(&(field->selfRobots[2]), 2, 0, 0);
}

void GetPlacement(Field* field) {
	SendLog(L"V/DLLStrategy:GetPlacement()");
	switch (whichType)
	{
	case JudgeType::PlaceKick:
		field->selfRobots[0].position = { 100,0 };
		field->selfRobots[1].position = { 80,60 };
		field->selfRobots[2].position = { 80,-60 };
		field->selfRobots[3].position = { 20,60 };
		field->selfRobots[4].position = { 30,-60 };
		break;
	case JudgeType::PenaltyKick:
		SendLog(L"Penalty Kick");
		break;
	case JudgeType::GoalKick:
		SendLog(L"Goal Kick");
		break;
	case JudgeType::FreeKickLeftBot:
		field->selfRobots[0].position = { 110,0 };
		field->selfRobots[1].position = { 80,60 };
		field->selfRobots[2].position = { 80,-60 };
		field->selfRobots[3].position = { 30,60 };
		field->selfRobots[4].position = { 30,-60 };
	case JudgeType::FreeKickLeftTop:
	case JudgeType::FreeKickRightBot:
	case JudgeType::FreeKickRightTop:
		SendLog(L"Free Kick");
		break;
	default:
		break;
	}
}

/*main*/

//计算用
double Atan(double y, double x) {

	if (x != 0.0 || y != 0.0)
		return 180 * atan2(y, x) / PI;
	else return 0.0;
}
double Atan(Vector2 begin, Vector2 end)
{
	double y, x;
	y = end.y - begin.y;
	x = end.x - begin.x;
	return Atan(y, x);
}
double Distance(Vector2 p1, Vector2 p2) {
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
double Distance(double x1, double x2, double y1, double y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}
void PredictBall2(double s, Field* field)
{
	double dx, dy;
	double xs;
	xs = s;
	dx = TRACE[5][1][0] - TRACE[5][0][0];
	dy = TRACE[5][1][1] - TRACE[5][0][1];
	PBP[0] = field->ball.position.x + dx * xs * 6;
	PBP[1] = field->ball.position.y + dy * xs * 6;

}
double F(double k, double x0, double y0, double x)
{
	return k * (x - x0) + y0;
}


//更新全局变量
void estimateV(Field* field) {
	double dx, dy;
	double k;
	int ym = 0;

	k = 6.8;
	if (TRACE[0][0][0] == -1)
	{
		for (ym = 0; ym < 5; ym++) {
			TRACE[ym][0][0] = field->selfRobots[ym].position.x;
			TRACE[ym][0][1] = field->selfRobots[ym].position.y;
			TRACE[ym][1][0] = field->selfRobots[ym].position.x;
			TRACE[ym][1][1] = field->selfRobots[ym].position.y;
		}
		TRACE[5][0][0] = field->ball.position.x;
		TRACE[5][0][1] = field->ball.position.y;
		TRACE[5][1][0] = field->ball.position.x;
		TRACE[5][1][1] = field->ball.position.y;
	}
	if (COUNT2 - COUNT1 >= 10) {
		for (ym = 0; ym < 6; ym++) {
			dx = TRACE[ym][1][0] - TRACE[ym][0][0];
			dy = TRACE[ym][1][1] - TRACE[ym][0][1];
			DISPLACEMENT[ym] = sqrt(dx * dx + dy * dy);
			TRACE[ym][0][0] = TRACE[ym][1][0];
			TRACE[ym][0][1] = TRACE[ym][1][1];
			EV[ym] = int(DISPLACEMENT[ym] * k);
		}
		COUNT1 = COUNT2;
	}
	else {
		for (ym = 0; ym < 5; ym++) {
			TRACE[ym][1][0] = field->selfRobots[ym].position.x;
			TRACE[ym][1][1] = field->selfRobots[ym].position.y;
		}
		TRACE[5][1][0] = field->ball.position.x;
		TRACE[5][1][1] = field->ball.position.y;
	}
}

//基础动作
void Velocity(Robot* robot, int vl, int vr)
{
	if (vl > 125)
		vl = 125;
	if (vr > 125)
		vr = 125;
	if (vl < -125)
		vl = -125;
	if (vr < -125)
		vr = -125;
	robot->wheel.leftSpeed = vl;
	robot->wheel.rightSpeed = vr;
}
double RotateTo(Robot* robot, int rID, const double desX, const double desY)
{
	double alpha, length;
	double sinValue;
	int direction;
	double aRRobot, aR;
	double dy;
	double beta;
	int v;
	double vk, k2 = 87;
	int error[6] = { 30,20,15,13,11,9 };
	int curError;
	int ll;

	vk = 1;
	length = sqrt((robot->position.x - desX) * (robot->position.x - desX) + (robot->position.y - desY) *
		(robot->position.y - desY));
	dy = desY - robot->position.y;
	if (dy < 0)
		dy = dy * -1;
	sinValue = dy / length;

	if (sinValue > 1)
		sinValue = 1;
	if (sinValue < -1)
		sinValue = -1;
	alpha = asinf(sinValue) / PI * 180;
	if ((desY < robot->position.y) && (desX < robot->position.x))//左下
		alpha = -180 + alpha;
	else if ((desY < robot->position.y) && (desX > robot->position.x))//右下
		alpha = -alpha;
	else if ((desY > robot->position.y) && (desX < robot->position.x))//左上
		alpha = 180 - alpha;
	else if ((desY > robot->position.y) && (desX > robot->position.x))//右上
		alpha = alpha;
	else if ((desY == robot->position.y) && (desX < robot->position.x))//左
		alpha = 180;
	else if ((desY == robot->position.y) && (desX > robot->position.x))//右
		alpha = 0;//把角转化为系统角度

	if (robot->rotation < 0)
		aRRobot = 360 + robot->rotation;
	else
		aRRobot = robot->rotation;

	if (alpha < 0)
		aR = 360 + alpha;
	else
		aR = alpha;

	beta = fabs(aRRobot - aR);
	if (aRRobot - aR > 0 && beta < 180)
		direction = 1;
	if (aRRobot - aR > 0 && beta > 180)
		direction = -1;
	if (aRRobot - aR < 0 && beta < 180)
		direction = -1;
	if (aRRobot - aR < 0 && beta > 180)
		direction = 1;

	//计算旋转方向
	if (beta > 180)
		beta = 360 - beta;
	else
		beta = beta;

	v = int(beta / 180 * 60 - vk);
	if (v < 0)//速度控制
		v = 0;
	if (v > 60)
		v = 60;
	if (v > 20)
		v = 60;
	if (direction == 1) {
		Velocity(robot, int(0.3 * EV[rID]) + v, int(0.3 * EV[rID]) - v);
	}
	else if (direction == -1) {
		Velocity(robot, int(0.3 * EV[rID]) - v, int(0.3 * EV[rID]) + v);
	}
	//选取误差限
	for (ll = 0; ll < 6; ll++) {
		if (length >= MAXL / 6 * (ll) && length < MAXL / 6 * (ll + 1))
			curError = error[ll];
	}
	if (beta > curError)
		NEEDROTATE[rID] = 1;
	else
		NEEDROTATE[rID] = 0;
	return beta;
}
void to(Robot* robot, int RID, double x, double y)
{
	int vBest[6] = { 30,50,60,70,100,125 };
	double length;
	double dx, dy;
	int lt;
	double l = 3;
	dx = robot->position.x - x;
	dy = robot->position.y - y;
	length = sqrt(dx * dx + dy * dy);
	lt = int(length / (112 * 2.54) * 6);
	double beta = RotateTo(robot, RID, x, y);
	RotateTo(robot, RID, x, y);
	if (NEEDROTATE[RID] != 1)
	{
		Velocity(robot, vBest[lt - 1], vBest[lt - 1]);
		if (length < l + 10)
		{
			Velocity(robot, 22, 22);
		}
		if (length < l + 5)
		{
			Velocity(robot, 16, 16);
		}
		if (length < l)
		{
			Velocity(robot, 11, 11);
		}
		if (length < 5)
		{
			Velocity(robot, 7, 7);
		}
		if (length < 1)
		{
			Velocity(robot, 4, 4);
		}
		if (length < 0.7)
		{
			Velocity(robot, 2, 2);
		}if (length < 0.4)
		{
			Velocity(robot, 1, 1);
		}
		if (length < 0.2)
		{
			Velocity(robot, 0, 0);
		}
	}
}
void go(Robot* robot, int rID, const double x, const double y) {
	double toX, toY;
	int vl, vr;

	toX = x;
	toY = y;
	vl = 125;
	vr = 125;
	RotateTo(robot, rID, toX, toY);
	if (NEEDROTATE[rID] != 1) {
		robot->wheel.leftSpeed = vl;
		robot->wheel.rightSpeed = vr;
	}
}
void Position(Robot* robot, double x, double y)
{
	int desired_angle = 0, theta_e = 0, d_angle = 0, vl, vr, vc = 120;

	double dx, dy, d_e, Ka = 10.0 / 90.0;
	dx = x - robot->position.x;					//计算出当前位置与目标位置的相对位移
	dy = y - robot->position.y;

	d_e = sqrt(dx * dx + dy * dy);			//计算出直线距离

	//计算出角度
	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int)(180. / PI * atan2((double)(dy), (double)(dx)));
	theta_e = desired_angle - (int)robot->rotation;

	//当前机器人角度与机器人到目标点的夹角
	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.)
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else
		Ka = 25. / 90.;

	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;

		if (theta_e > 180)
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)(vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)(vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (int)(+.17 * theta_e);
		vl = (int)(-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}
/*
进攻框架：
按照底角和中心分区，底角采取传球模式，中心进行射门姿势
*/
void Attack(Field* field)
{
	//左上进攻
	if (field->ball.position.x < -75 && field->ball.position.y>40)
	{

	}
	//左下进攻
	else if (field->ball.position.x < -75 && field->ball.position.y < -40)
	{

	}
	//中路射门
	else
	{
		LineAttack(field);
		//选出最适合
		int WhoAttack;
		double Min = 1000;
		for (int i = 2; i < 5; i++)
		{
			double K = (PBP[1] - field->selfRobots[i].position.y) / (PBP[0] - field->selfRobots[i].position.x);
			if (min(abs(K - KBTO), abs(K - KTOP)) < Min)
			{
				Min = min(abs(K - KBTO), abs(K - KTOP));
				WhoAttack = i;
			}
		}
		//进攻者调整角度进行进攻
	}
}
void LineAttack(Field* field)
{
	PredictBall2(1, field);
	KTOP = (PBP[1] - 20) / (PBP[0] + 110);
	KBTO = (PBP[1] + 20) / (PBP[0] + 110);
}

//琦玲的
void attack(int robot1, int robot2, int robot3,Field* field) {
	//球在对方半场
	int low, middle, most;
	
	if (field->ball.position.x < 0) {
		switch (pos(field->ball.position))
		{
		case 1: {
			
		}
		case 2:
		case 3:
		default:
			break;
		}
	}

}

int pos(Vector2 pos) {
	if (pos.x < -60) {
		if (pos.y > 50) return 1;
		if (pos.y < -50) return 2;
	}
	return 3;
}
//double dis(Vector2 p1, Vector2 p2) {
//	return (p1.x-p2.x)*(p2.x-p2.x)+
// }



void dirShoot(Field* field, int id) {
	//还锁
	if (field->selfRobots[id].position.x < field->ball.position.x) {
		dirShootLock[id] = 0;
		double dy = field->selfRobots[id].position.y > field->ball.position.y ? 3.0*2.54 : -3.0*2.54;
		;
		if (dy + field->ball.position.y > FTOP - 1.0*2.54) {
			dy *= -1;
		}
		if (dy + field->ball.position.y < FBOT + 1.0*2.54) {
			dy *= -1;
		}
		go(&(field->selfRobots[id]), id, field->ball.position.x + 3.0*2.54, field->ball.position.x + dy);
		return;
	}
	if (Distance(field->selfRobots[id].position, field->ball.position) > 15.0*2.54) {
		//注意dist的值不能小于自己设的那个延长距离
		dirShootLock[id] = 0;
	}

	//射门
	if (dirShootLock[id]) {
		//go(&(field->selfRobots[id]), id, dirShootPos[id].x, dirShootPos[id].y);
		double x = (dirShootPos[id].x - field->ball.position.x) * 0.3 + field->ball.position.x;
		double y = (dirShootPos[id].y - field->ball.position.y) * 0.3 + field->ball.position.y;
		//Position(&(field->selfRobots[id]), x, y);
		go(&(field->selfRobots[id]), id, x, y);
		//还锁

		return;
	}

	dirShootLock[id] = 0;
	//简单判断
	double tx = field->ball.position.x + 10.0*2.54, ty;
	if (field->ball.position.y < GBOTY + 5.0) {
		ty = F((GTOPY - field->ball.position.y) / (GLEFT - field->ball.position.x),
			field->ball.position.x, field->ball.position.y, tx);
		dirShootPos[id] = { GLEFT * 1.0, GBOTY * 0.3 + GTOPY * 0.7 };
	}
	else if (field->ball.position.y > GTOPY - 5.0) {
		ty = F((GBOTY - field->ball.position.y) / (GLEFT - field->ball.position.x),
			field->ball.position.x, field->ball.position.y, tx);
		dirShootPos[id] = { GLEFT * 1.0, GBOTY * 0.7 + GTOPY * 0.3 };
	}
	else {
		//需要细化
		ty = F((GBOTY - field->ball.position.y) / (GLEFT - field->ball.position.x),
			field->ball.position.x, field->ball.position.y, tx);
		dirShootPos[id] = { GLEFT * 1.0, GBOTY * 0.7 + GTOPY * 0.3 };
	}

	
	//if (env->opponent[0].pos.y > MID) {
	//	//守门员在上面
	//	ty = F((env->goalBounds.bottom - field->ball.position.y) / (env->goalBounds.left - field->ball.position.x),
	//		field->ball.position.x, field->ball.position.y, tx);
	//}
	//else {
	//	//守门员在下面
	//	ty = F((env->goalBounds.top - field->ball.position.y) / (env->goalBounds.left - field->ball.position.x),
	//		field->ball.position.x, field->ball.position.y, tx);
	//}

	//防止溢出
	if (ty > FTOP - 5.0*2.54 || ty <FBOT + 5.0*2.54) {
		Position(&(field->selfRobots[id]), field->ball.position.x, field->ball.position.x);
		return;
	}

	//do shooting

	double dist = Distance(tx, field->selfRobots[id].position.x, ty, field->selfRobots[id].position.y);
	if (dist < 1.5*2.54) {
		dirShootLock[id] = 1;
		//Velocity(&(field->selfRobots[id]), 0, 0);
		//RotateTo(&(field->selfRobots[id]), id, dirShootPos[id].x, dirShootPos[id].y);
		//Position(&(field->selfRobots[id]), dirShootPos[id].x, dirShootPos[id].y);
		double x = (dirShootPos[id].x - field->ball.position.x) * 0.3 + field->ball.position.x;
		double y = (dirShootPos[id].y - field->ball.position.y) * 0.3 + field->ball.position.y;
		go(&(field->selfRobots[id]), id, x, y);
	}
	else if (dist < 10.0*2.54) {
		Velocity(&(field->selfRobots[id]), 10, 10);
	}
	else {
		Position(&(field->selfRobots[id]), tx, ty);
	}

}
