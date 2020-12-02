// DLLStrategy.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "platform.h"
#include "adapter.h"

using namespace Simuro;
using namespace Adapter;
JudgeType whichType;
const double PI = 3.1415926333;


void Position(Robot* robot, double x, double y);
void attack(int robot1, int robot2, int robot3, Field* field);
void Velocity(Robot* robot, int vl, int vr);
int pos(Vector2 pos);


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
	attack(1, 2, 3, field);
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
	return (p1.x-p2.x)*(p2.x-p2.x)+
 }