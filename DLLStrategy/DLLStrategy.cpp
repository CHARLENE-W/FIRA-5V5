// DLLStrategy.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "platform.h"
#include "adapter.h"

using namespace Simuro;
using namespace Adapter;
JudgeType whichType;
const double PI = 3.1415926;


/*ADD*/
const double PI = 3.141592;
double DISPLACEMENT[6] = { 0 };
double COUNT1 = 0, COUNT2 = 0;//保存调用次数
const double MAXL = 112;
int NEEDROTATE[5] = { 1,1,1,1,1 }; //1 need，else not need
int EV[6] = { 0 }; //estimate V估计的机器人的速度and the ball
double TRACE[6][2][2] = { -1,-1,-1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1 };
double PBP[2] = { 0,0 };


void Position(Robot* robot, double x, double y);
void attack(int robot1, int robot2, int robot3, Field* field);
void Velocity(Robot* robot, int vl, int vr);
int pos(Vector2 pos);
double dis(Vector2 p1, Vector2 p2);
double RotateTo(Robot* robot, int rID, const double desX, const double desY);
void go(Robot* robot, int rID, const double x, const double y);
void to(Robot* robot, int RID, double x, double y);


double Atan(Vector2 begin, Vector2 end);
void RegulateAng180(double& ang);
void RegulateAng180(double& ang);
void RegulateAng360(double& ang);
double Distance(Vector2 p1, Vector2 p2);
double Distance(double x1, double x2, double y1, double y2);
void PredictBall2(double s, Field* field);
void estimateV(Field* field);


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
	static const wchar_t teamName[] = L"Waaaa";
	static constexpr size_t len = sizeof(teamName);
	memcpy(teamInfo->teamName, teamName, len);
}

void GetInstruction(Field* field) {
	SendLog(L"V/DLLStrategy:GetInstruction()");
	attack(4, 2, 3, field);
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


double Atan(double y, double x);
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
void RegulateAng180(double& ang)
{
	//规范角到(-180， 180]
	while (ang <= -180) {
		ang += 360;
	}
	while (ang > 180) {
		ang -= 360;
	}
}
void RegulateAng360(double& ang)
{
	//规范角到[0， 360)
	while (ang < 0) {
		ang += 360;
	}
	while (ang >= 360) {
		ang -= 360;
	}
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






void attack(int robot1, int robot2, int robot3,Field* field) {
	//球在对方半场
	int l, m, r;//最近 中间 最远
	double dis1 = dis(field->selfRobots[robot1].position, field->ball.position);
	double dis2 = dis(field->selfRobots[robot2].position, field->ball.position);
	double dis3 = dis(field->selfRobots[robot3].position, field->ball.position);
	if (dis1 < dis2) {
		l = dis1 < dis3 ? robot1 : robot3;
		r = dis2 > dis3 ? robot2 : robot3;
	}
	else {
		l = dis2 < dis3 ? robot2 : robot3;
		r = dis1 > dis3 ? robot1 : robot3;
	}
	m = robot1 + robot2 + robot3 - l - r;
	int L, M, R;//最近 中间 最远
	 dis1 = dis(field->selfRobots[2].position, field->ball.position);
	 dis2 = dis(field->selfRobots[3].position, field->ball.position);
	 dis3 = dis(field->selfRobots[4].position, field->ball.position);
	if (dis1 < dis2) {
		L = dis1 < dis3 ? 12 : 4;
		R = dis2 > dis3 ? 3 : 4;
	}
	else {
		L = dis2 < dis3 ? 3 : 4;
		R = dis1 > dis3 ? 2 : 4;
	}
	M = robot1 + robot2 + robot3 - L - R;

	if (field->ball.position.x < 30) {
		switch (pos(field->ball.position))
		{
		case 1: {
			Position(&field->selfRobots[l], field->ball.position.x, field->ball.position.y);
			Position(&field->selfRobots[m], field->ball.position.x+10, field->selfRobots[l].position.y*0.1);
			Position(&field->selfRobots[r], field->selfRobots[m].position.x, field->selfRobots[m].position.y);
		}
		case 2: {
			Position(&field->selfRobots[l], field->ball.position.x, field->ball.position.y);
			Position(&field->selfRobots[M], field->opponentRobots[L].position.x, field->opponentRobots[L].position.y);
			Position(&field->selfRobots[R], field->opponentRobots[M].position.x, field->opponentRobots[M].position.y);
		}
		case 3: {
			Position(&field->selfRobots[l], field->ball.position.x, field->ball.position.y);
			Position(&field->selfRobots[m], field->selfRobots[l].position.x, field->selfRobots[l].position.y);
			Position(&field->selfRobots[r], field->selfRobots[m].position.x, -field->selfRobots[m].position.y);
		}
		default:
			break;
		}
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
void Velocity(Robot* robot, int vl, int vr) {
	robot->wheel.leftSpeed = vl;
	robot->wheel.rightSpeed = vr;
}
int pos(Vector2 pos) {
	if (pos.x < -60) {
		if (pos.y > 50) return 1;
		if (pos.y < -50) return 2;
	}
	return 3;
}
double dis(Vector2 p1, Vector2 p2) {
	return (p1.x - p2.x) * (p2.x - p2.x) + (p1.y - p2.y) * (p2.y - p2.y);
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