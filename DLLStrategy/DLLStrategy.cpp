#include "stdafx.h"
#include "platform.h"
#include "adapter.h"
#include<math.h>
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
int Eventstate;
int Judgestate;
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
const double MAXL = 284.48;
int NEEDROTATE[5] = { 1,1,1,1,1 }; //1 need，else not need
int EV[6] = { 0 }; //estimate V估计的机器人的速度and the ball
double TRACE[6][2][2] = { -1,-1,-1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1, -1,-1 };
double PBP[2] = { 0,0 };

//粒子群
double particle_w = 0.5;
Vector2  Robot_old_pos[5];//!!!!记得初始化
Vector2 particle_X[5] = { 0 };
Vector2 particle_V[5] = { 0 };
double  particle_F[5] = { 0 };
Vector2 Pbest[5] = { 0 };
Vector2 Gbest = { 0, 0 };

int the_kick_one = 5;

//射门锁
int dirShootLock[5] = { 0 };
Vector2 dirShootPos[5] = { 0 };

void avoidance(Field* field, int id);
//计算用
double Atan(double y, double x);
double Atan(Vector2 begin, Vector2 end);
double Atan(Vector2 O, Vector2 A, Vector2 B);
void RegulateAng180(double& ang);
void RegulateAng360(double& ang);
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
void PositionPro(Robot* robot, int id,  double x, double y);
void avoidance(Field* field, int id);
void RightWing(Field* pEnv, int id);

void kickBall_mid(Field* field, int id, Vector2 pos);
void kickBall_circle(Field* field, int id, Vector2 pos);
void waiter(Field* field, int id, Vector2 pos);
/*ADD*/

/* 粒子群 */
unsigned int PSO_init_count = 0;
void PSO_init(Field* field);
void PSO_fresh(Field* field);
void PSO_do(Field* field);

void END(Field* field);
/* ------ */

//进攻
double KTOP, KBTO;
void Attack(Field* field);
void LineAttack(Field* field);
bool BlueShoot(Field* field);

void attack(int robot1, int robot2, int robot3, Field* field);
int pos(Vector2 pos);

bool canKshoot(Field* field, int id);
void dirShoot(Field* field, int id);

//防守
void Goliar(Field* field);
void activeDefender(Field* field, Robot* robot, int riD);
void Defend(Field* field, int rID);
void MidDefend(Field* field, int id1, int id2);
void NegDefend(Field* field, int id);
void keeper(Robot* robot, int rID, Field* field);




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
	
	COUNT2++;
	estimateV(field);
	//BlueShoot(field);
	if (PSO_init_count == 0) {
		PSO_init(field);
	}
	PSO_init_count++;
	if (PSO_init_count >= 60) {
		PSO_init_count = 0;
	}
	
	PSO_fresh(field);
	PSO_do(field);

	//dirShoot(field, 4);

	/*Vector2 kick2pos = { GLEFT, GBOTY };
	kickBall_mid(field, 4, kick2pos);*/

	/*if (the_kick_one == 5) {
		the_kick_one = 4;
		Vector2 kick2pos = { GLEFT, GBOTY };
		kickBall_mid(field, 4, kick2pos);
	}
	else {
		go(&(field->selfRobots[4]), 4, field->selfRobots[the_kick_one].position.x + 5.0, field->selfRobots[the_kick_one].position.y + 5.0);
		Velocity(&(field->selfRobots[4]), 10, 10);
	}
	
	Vector2 wait2pos = { -70, 0 };
	if (field->opponentRobots[0].position.y > GTOPY * 0.7 + GBOTY * 0.3) {
		wait2pos = { -70, GTOPY * 0.2 + GBOTY * 0.8 };
	}
	if (field->opponentRobots[0].position.y < GTOPY * 0.3 + GBOTY * 0.7) {
		wait2pos = { -70, GTOPY * 0.8 + GBOTY * 0.2 };
	}
	waiter(field, 3, wait2pos);*/
	//Position(&(field->selfRobots[4]), field->ball.position.x, field->ball.position.y);

	/*Vector2 kick2pos = { GLEFT, GBOTY };
	kickBall_mid(field, 4, kick2pos);*/
	END(field);
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

	
	for (int i = 0; i < 5; i++) {
		Robot_old_pos[i] = field->selfRobots[i].position;
	}
	COUNT2++;
	estimateV(field);
	PSO_init(field);
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
double Atan(Vector2 O, Vector2 A, Vector2 B)
{
	//注意返回的是正切
	double dA = Distance(O, A);
	double dB = Distance(O, B);

	double cosAng = ((A.x - O.x) * (B.x - O.x) + (A.y - O.y) * (B.y - O.y)) / (dA * dB);
	double tanAng = sqrt(1 - cosAng * cosAng) / cosAng;
	return tanAng;
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
double Distance(double x1, double y1, double x2, double y2)
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
	PBP[0] = field->ball.position.x + dx * xs;
	PBP[1] = field->ball.position.y + dy * xs;

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
	double l = 3 * 2.54;
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
		if (length < 2.54)
		{
			Velocity(robot, 4, 4);
		}
		if (length < 1.8)
		{
			Velocity(robot, 2, 2);
		}if (length < 1)
		{
			Velocity(robot, 1, 1);
		}
		if (length < 0.5)
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
void PositionPro(Robot* robot, int id, double x, double y) {
	double tAng = Atan(y - robot->position.y, x - robot->position.x);
	double da = tAng - robot->rotation;
	RegulateAng180(da);
	if (da > 80 || da < -80) {
		go(robot, id, x, y);
	}
	else {
		//go(robot, id, x, y);
		Position(robot, x, y);
	}
}
void avoidance(Field* field, int id)
{
	bool duiren = false;
	for (int i = 0; i <= 4; i++) {
		if (i != id) {

			//先查找距离是否相近，在判断是否在前进方向上
			if (Distance(field->selfRobots[i].position, field->selfRobots[id].position) < 6) {
				double deltaY = field->selfRobots[i].position.y - field->selfRobots[id].position.y;
				double deltaX = field->selfRobots[i].position.x - field->selfRobots[id].position.x;
				double angle_home_another = Atan(fabs(deltaY), fabs(deltaX));
				if (deltaX < 0 && deltaY>0) { angle_home_another = 180 - angle_home_another; }
				else
				{
					if (deltaX > 0 && deltaY < 0) { angle_home_another = -angle_home_another; }
					else
					{
						if (deltaX < 0 && deltaY < 0) { angle_home_another = -180 + angle_home_another; }

					}
				}

				if (fabs(angle_home_another - field->selfRobots[id].rotation) < 20)
				{

					duiren = true;

					break;
				}
			}

		}

		if (Distance(field->opponentRobots[i].position, field->selfRobots[id].position) < 6) {
			double deltaY = field->opponentRobots[i].position.y - field->selfRobots[id].position.y;
			double deltaX = field->opponentRobots[i].position.x - field->selfRobots[id].position.x;
			double angle_home_another = Atan(fabs(deltaY), fabs(deltaX));
			if (deltaX < 0 && deltaY>0) { angle_home_another = 180 - angle_home_another; }
			else {
				if (deltaX > 0 && deltaY < 0) { angle_home_another = -angle_home_another; }
				else
				{
					if (deltaX < 0 && deltaY < 0) { angle_home_another = -180 + angle_home_another; }

				}
			}


			if (fabs(angle_home_another - field->selfRobots[id].rotation) < 20)
			{

				duiren = true;

				break;
			}
		};

	}

	if (duiren)
		Velocity(&field->selfRobots[id], -120, 120);

}







void kickBall_mid(Field* field, int id, Vector2 pos)
{
	//目标线
	double k2 = (field->ball.position.y - pos.y) / (field->ball.position.x - pos.x);
	double x2 = field->ball.position.x;
	double y2 = field->ball.position.y;
	double yy = F(k2, x2, y2, field->selfRobots[id].position.x);
	if (field->selfRobots[id].position.x > field->ball.position.x &&
		Distance(field->selfRobots[id].position, field->ball.position) < 15.0 ) {

		go(&(field->selfRobots[id]), id, pos.x, pos.y);
		return;
	}

	if (field->selfRobots[id].position.x > field->ball.position.x && 
		fabs(yy - field->selfRobots[id].position.y) < 10.0) {

		go(&(field->selfRobots[id]), id, field->ball.position.x, field->ball.position.y);
		return;
	}
	

	//中垂线
	double kk = 0.5;
	double k1 = -(field->ball.position.x - field->selfRobots[id].position.x) / (field->ball.position.y - field->selfRobots[id].position.y);
	double x1 = (field->ball.position.x * kk + field->selfRobots[id].position.x * kk);
	double y1 = (field->ball.position.y * kk + field->selfRobots[id].position.y * kk);
	


	Vector2 tpos = { 0, 0 };
	tpos.x = (k1 * x1 - k2 * x2 + y2 - y1) / (k1 - k2);
	tpos.y = k2 * (tpos.x - x2) + y2;
	/*if (fabs(field->selfRobots[id].position.y - field->ball.position.y) < 5.0) {
		go(&(field->selfRobots[id]), id, field->ball.position.x, field->ball.position.y);
		return;
	}*/

	Position(&(field->selfRobots[id]), tpos.x, tpos.y);
	//go(&(field->selfRobots[id]), id, tpos.x, tpos.y);
	//PositionPro(&(field->selfRobots[id]), id, tpos.x, tpos.y);
}

void kickBall_circle(Field* field, int id, Vector2 pos)
{
	//有条件限制 外面得有判断
	double tanRBP = Atan(field->ball.position, field->selfRobots[id].position, pos);
	if (tanRBP < 0 && tanRBP > -1) {
		//RBP > 3/4 PI
		double tan_half_RBP = (-1 + sqrt(1 + tanRBP * tanRBP)) / tanRBP;
		double R = 10.0 * (1 + tan_half_RBP); //5.0是可调常数

		//
		
		double kk = R / Distance(field->ball.position, pos);
		Vector2 O = { field->ball.position.y + kk * (field->ball.position.x - pos.x), field->ball.position.x + kk * (field->ball.position.y - pos.y) };
	}
	return;
}

void waiter(Field* field, int id, Vector2 pos)
{
	if (Distance(field->selfRobots[id].position, field->ball.position) < 20.0 || 
		(Distance(field->selfRobots[id].position, pos) < 5.0 && field->ball.position.y < GTOPY && field->ball.position.y > GBOTY)) {

		the_kick_one = id;
		PredictBall2(1, field);
		Vector2 tpos = { FLEFTX, (float)PBP[1] };

		//kickBall_mid(field, id, tpos);
		go(&(field->selfRobots[id]), id, tpos.x, tpos.y);
		return;
	}

	the_kick_one = 5;

	go(&(field->selfRobots[id]), id, pos.x, pos.y);
	if (Distance(field->selfRobots[id].position, pos) < 5.0) {
		Velocity(&(field->selfRobots[id]), 10, 10);

	}
	//if(Distance(field->selfRobots[id].position, pos) < 5.0)
}

void PSO_init(Field* field)
{
	for (int i = 0; i < 5; i++) {
		particle_X[i] = field->selfRobots[i].position;
		particle_V[i] = { field->selfRobots[i].position.x - Robot_old_pos[i].x,  field->selfRobots[i].position.y - Robot_old_pos[i].y };
		Pbest[i] = particle_X[i];
	}

	PredictBall2(1, field);
	Gbest = { (float)PBP[0], (float)PBP[1] };

	for (int i = 0; i < 5; i++) {
		double theta = atan2(field->selfRobots[i].position.y - field->ball.position.y, field->selfRobots[i].position.x - field->ball.position.x); //blue
		particle_F[i] = 1 / (1 * Distance(field->selfRobots[i].position, field->ball.position) + 10 * fabs(theta));
	}
}

void PSO_fresh(Field* field)
{
	
	for (int i = 0; i < 5; i++) {
		particle_V[i].x = 0.5 * particle_V[i].x + 0.5 * (Pbest[i].x - particle_X[i].x)
			                                    + 2.0 * (Gbest.x    - particle_X[i].x);
		particle_V[i].y = 0.5 * particle_V[i].y + 0.5 * (Pbest[i].y - particle_X[i].y)
			                                    + 2.0 * (Gbest.y    - particle_X[i].y);


		particle_X[i].x += particle_V[i].x;
		particle_X[i].y += particle_V[i].y;

		if (particle_X[i].x < -105) {
			particle_X[i].x = -105;
		}
		if (particle_X[i].x > 105) {
			particle_X[i].x = 105;
		}
		
		if (particle_X[i].y < -85) {
			particle_X[i].y = -85;
		}
		if (particle_X[i].y > 85) {
			particle_X[i].y = 85;
		}
	}

	double fMin = 6666.0;
	Vector2 PbestMin;
	for (int i = 0; i < 5; i++) {
		double theta = atan2(particle_X[i].y - field->ball.position.y, particle_X[i].x - field->ball.position.x); //blue
		double f = 1 / (1.0 * Distance(particle_X[i], field->ball.position) + 10 * fabs(theta));
		
		if (f > particle_F[i]) {
			Pbest[i] = particle_X[i];
		}
		if (particle_F[i] < fMin && i != 0) {
			PbestMin.x = Pbest[i].x;
			PbestMin.y = Pbest[i].y;
		}
	}
	/*PredictBall2(1, field);
	Gbest = { (float)PBP[0], (float)PBP[1] };*/
	Gbest = field->ball.position;
	//Gbest = PbestMin;
}

void PSO_do(Field* field)
{
	int kickID = 5;
	double ff = -1.0;
	for (int i = 1; i < 5; i++) {
		if (particle_F[i] > ff) {
			kickID = i;
			ff = particle_F[i];
		}
	}

	Vector2 kick2pos = { GLEFT, GBOTY };
	kickBall_mid(field, kickID, kick2pos);

	for (int i = 1; i < 5; i++) {
		if (i == kickID) {
			continue;
		}
		PositionPro(&(field->selfRobots[i]), i, Pbest[i].x, Pbest[i].y);
		//PositionPro(&(field->selfRobots[i]), i, particle_X[i].x, particle_X[i].y);
	}
}

void END(Field* field)
{
	for (int i = 0; i < 5; i++) {
		Robot_old_pos[i] = field->selfRobots[i].position;
	}
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
void attack(int robot1, int robot2, int robot3, Field* field) {//传入参数就是进攻的三个机器人ID
	//球在对方半场
	int l, m, r;//最近 中间 最远
	double dis1 = Distance(field->selfRobots[robot1].position, field->ball.position);
	double dis2 = Distance(field->selfRobots[robot2].position, field->ball.position);
	double dis3 = Distance(field->selfRobots[robot3].position, field->ball.position);
	if (dis1 < dis2) {
		l = dis1 < dis3 ? robot1 : robot3;
		r = dis2 > dis3 ? robot2 : robot3;
	}
	else {
		l = dis2 < dis3 ? robot2 : robot3;
		r = dis1 > dis3 ? robot1 : robot3;
	}
	m = robot1 + robot2 + robot3 - l - r;

	//int L, M, R;//最近 中间 最远
	// dis1 = dis(field->opponentRobots[2].position, field->ball.position);
	// dis2 = dis(field->opponentRobots[3].position, field->ball.position);
	// dis3 = dis(field->opponentRobots[4].position, field->ball.position);
	//if (dis1 < dis2) {
	//	L = dis1 < dis3 ? 2 : 4;
	//	R = dis2 > dis3 ? 3 : 4;
	//}
	//else {
	//	L = dis2 < dis3 ? 3 : 4;
	//	R = dis1 > dis3 ? 2 : 4;
	//}
	//M = 2 + 3 + 4 - L - R;
	//30-》0
	if (field->ball.position.x < 30) {
		switch (pos(field->ball.position))
		{
		case 1: {
			//左上
			go(&field->selfRobots[l], l, field->ball.position.x, field->ball.position.y);
			go(&field->selfRobots[r], r, (double)field->ball.position.x + 10, (double)field->ball.position.y * 0.1);
			go(&field->selfRobots[m], m, (double)field->selfRobots[l].position.x + 10, (double)field->selfRobots[l].position.y - 10);
			break;
		}
		case 2: {
			//右下
			go(&field->selfRobots[l], l, field->ball.position.x, field->ball.position.y);
			go(&field->selfRobots[r], r, (double)field->ball.position.x + 10, (double)-field->selfRobots[l].position.y * 0.1);
			go(&field->selfRobots[m], m, (double)field->selfRobots[l].position.x + 10, (double)field->selfRobots[l].position.y + 10);
			break;
		}
		case 3: {
			//中间区域
			/*
				!!!!!!!
				亚欣在后面找个地方插夹球
				!!!!!!!
			*/
			//if(atan2)
			//判断夹球射击，BlueShoot返回true则夹球成功

			if (BlueShoot(field))
				;
			else
			{
				if (dirShootLock[l] == 1 || Distance(field->selfRobots[l].position, field->ball.position) > 8.0 * 2.54) {
					//最近球员都比较远
					dirShoot(field, l);

					//m 
					PositionPro(&(field->selfRobots[r]), r,  field->ball.position.x, field->ball.position.y);
					activeDefender(field, &(field->selfRobots[r]), r);
					//r
				}
				else if (dirShootLock[m] == 1) {
					//最近的控球 m冲
					PredictBall2(1 / 30, field);
					go(&(field->selfRobots[l]), l, PBP[0], PBP[1]);
					dirShoot(field, m);
					activeDefender(field, &(field->selfRobots[r]), r);
					//l
					//r
				}
				else if (dirShootLock[r] == 1) {
					//最近的控球 m冲
					PredictBall2(1 / 30, field);
					go(&(field->selfRobots[l]), l, PBP[0], PBP[1]);
					dirShoot(field, r);
					activeDefender(field, &(field->selfRobots[m]), m);
					//l
					//r
				}
			}
			if (canKshoot(field, l)) {
				//piao的老代码的 感觉不大好 亚欣改了predictBall看能不能好一点
				PredictBall2(1 / 30, field);
				go(&(field->selfRobots[l]), l, PBP[0], PBP[1]);
				//m
				//r
			}
			else if (canKshoot(field, m)) {
				PredictBall2(1 / 30, field);
				go(&(field->selfRobots[m]), m, PBP[0], PBP[1]);
				//l 控球？
				//r
			}
			else if (canKshoot(field, r)) {
				PredictBall2(1 / 30, field);
				go(&(field->selfRobots[r]), r, PBP[0], PBP[1]);

				//l 控球？
				//m 跟人？
			}


			else {
				//两个追 一个在中间等？
				PredictBall2(1 / 30, field);
				go(&(field->selfRobots[l]), l, PBP[0], PBP[1]);
				go(&(field->selfRobots[m]), m, PBP[0], PBP[1]);
				go(&(field->selfRobots[r]), r, -60, 0);
				//r wait
			}

			//依次判断是否能直射

		}
		default:
			break;
		}
	}
	else {
		//转防守？
		//go(&field->selfRobots[0], 0, field->ball.position.x, field->ball.position.y);
		//Velocity(&field->selfRobots[0], -100, 100);//!!!!测试 记得删掉
	}

}


int pos(Vector2 pos) {
	if (pos.x < -60) {
		if (pos.y > 50) return 1;
		if (pos.y < -50) return 2;
	}
	return 3;
}


bool canKshoot(Field* field, int id)
{

	if (field->selfRobots[id].position.x > field->ball.position.x) {
		double k = (field->selfRobots[id].position.y - field->ball.position.y) / (field->selfRobots[id].position.x - field->ball.position.x);
		double bx = field->ball.position.x;
		double by = field->ball.position.y;
		double y = F(k, bx, by, FLEFTX);
		if (y < GTOPY - 2.5 && y > GBOTY + 2.5) {
			return true;
		}
	}
	return false;
}
void dirShoot(Field* field, int id) {
	double Bdist = Distance(field->selfRobots[id].position, field->ball.position);
	//还锁
	if (field->selfRobots[id].position.x < field->ball.position.x) {
		//最好加上速度判断
		dirShootLock[id] = 0;
		double dy = field->selfRobots[id].position.y > field->ball.position.y ? 3.0 * 2.54 : -3.0 * 2.54;
		;
		if (dy + field->ball.position.y > FTOP - 1.0 * 2.54) {
			dy *= -1;
		}
		if (dy + field->ball.position.y < FBOT + 1.0 * 2.54) {
			dy *= -1;
		}
		go(&(field->selfRobots[id]), id, field->ball.position.x + 3.0 * 2.54, field->ball.position.y + dy);
		return;
	}
	if (Bdist > 15.0 * 2.54) {
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
	double tx = field->ball.position.x + 10.0 * 2.54, ty;

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

		/*ty = F((GBOTY - field->ball.position.y) / (GLEFT - field->ball.position.x),
			field->ball.position.x, field->ball.position.y, tx);
		dirShootPos[id] = { GLEFT * 1.0, GBOTY * 0.7 + GTOPY * 0.3 };*/
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

	//防止溢出 !!!!!!!!!!!!参数应该要改？
	if (ty > FTOP - 10.0 || ty < FBOT + 10.0 || tx < FLEFTX + 10.0 || tx > FRIGHTX - 10.0) {
		PositionPro(&(field->selfRobots[id]), id, field->ball.position.x, field->ball.position.y);
		return;
	}

	//do shooting

	double dist = Distance(tx, ty, field->selfRobots[id].position.x, field->selfRobots[id].position.y);
	if (dist < 2.5 * 2.54) {
		dirShootLock[id] = 1;
		//Velocity(&(field->selfRobots[id]), 0, 0);
		//RotateTo(&(field->selfRobots[id]), id, dirShootPos[id].x, dirShootPos[id].y);
		//Position(&(field->selfRobots[id]), dirShootPos[id].x, dirShootPos[id].y);
		double x = (dirShootPos[id].x - field->ball.position.x) * 0.3 + field->ball.position.x;
		double y = (dirShootPos[id].y - field->ball.position.y) * 0.3 + field->ball.position.y;
		go(&(field->selfRobots[id]), id, x, y);
	}
	/*else if (dist < 8.0) {
		Velocity(&(field->selfRobots[id]), 10, 10);
	}*/
	else {
		PositionPro(&(field->selfRobots[id]), id, tx, ty);
	}

}

bool BlueShoot(Field* field)
{
	//夹球射门
	double x1 = field->selfRobots[3].position.x;
	double y1 = field->selfRobots[3].position.y;
	double x2 = field->selfRobots[4].position.x;
	double y2 = field->selfRobots[4].position.y;
	double k = (x1 - x2) / (y1 - y2);
	PredictBall2(1, field);
	double xc = PBP[0];
	double yc = PBP[1];
	double s1 = atan2(yc - y1, xc - x1);
	double s2 = atan2(yc - y2, xc - x2);
	if (x1 < xc || x2 < xc)
		return FALSE;
	if (s1 + s2 < 20 / 180 * PI)
		return FALSE;
	double b = yc - k * xc;
	double ShootY = k * FLEFTX + b;
	if (ShootY > GTOPY || ShootY < GBOTY)
		return FALSE;
	double d1 = Distance(x1, y1, xc, yc);
	double d2 = Distance(x2, y2, xc, yc);
	double v1, v2;
	if (d1 > d2)
	{
		v1 = 125;
		v2 = v1 * d2 / d1;
	}
	else
	{
		v2 = 125;
		v1 = v2 * d1 / d2;
	}

	if (ShootY <= GTOPY && ShootY >= GBOTY)
	{
		RotateTo(&(field->selfRobots[3]), 3, xc, yc);
		RotateTo(&(field->selfRobots[4]), 4, xc, yc);
		if (NEEDROTATE[3] != 1 && NEEDROTATE[4] != 1)
		{
			Velocity(&(field->selfRobots[3]), v1, v1);
			Velocity(&(field->selfRobots[4]), v2, v2);
		}

	}
	return TRUE;
}


//defand
void activeDefender(Field* field, Robot* robot, int riD)
{
	PredictBall2(1, field);
	//绕到球的右边kick ball
	double bx, by;//ball 的坐标
	double rx, ry;//机器人坐标
	double xl, yl;//
	double dx;//rx-bx
	double desx, desy;//
	xl = 20;
	//yl = 15.24;
	yl = 5;
	bx = field->ball.position.x;
	by = field->ball.position.y;
	rx = robot->position.x;
	ry = robot->position.y;
	dx = rx - bx;
	//double length = distance()
	if (dx < 2.54)
	{
		if (ry < by)
		{

			desy = by - yl;
			if (desy < -90)
				desy = -90;

		}
		if (ry >= by)
		{
			desy = by + yl;
			if (desy > 90)
				desy = 90;

		}
		desx = bx + xl;
		if (desx > 110)
		{
			desx = 110;
		}
		go(robot, riD, desx, desy);

	}
	else
		go(robot, riD, bx + 4, by);
	//if (bx > 88.28 && by<33.93 && ry>by)
		//go(robot, riD, bx, by + 1);
	//if (bx > 88.28 && by > 49.68 && ry < by)
		//go(robot, riD, bx, by - 1);
}
void NegDefend(Field* field, int id)
{
	double bx = field->ball.position.x;
	double by = field->ball.position.y;
	double rx = field->selfRobots[id].position.x;
	double ry = field->selfRobots[id].position.y;
	if (bx >= 0)
	{
		if (bx <= rx)
		{
			Position(&field->selfRobots[id], 68, by);
		}
		else
		{
			if (by <= 0)
				Position(&field->selfRobots[id], rx, by + 10);
			else
				Position(&field->selfRobots[id], rx, by - 10);
		}
	}
}
void MidDefend(Field* field, int id1, int id2)
{
	PredictBall2(2, field);
	double bx = field->ball.position.x;
	double by = field->ball.position.y;
	if (bx > 0)
	{
		if (bx >= 0 && bx <= 30)
		{
			go(&field->selfRobots[id1], id1, bx, by);
			go(&field->selfRobots[id2], id2, bx, by);
			return;
		}
		if (by >= 40)
			Position(&field->selfRobots[id1], 2.5, by - 8);
		else
			Position(&field->selfRobots[id1], 2.5, 70);
		if (by <= -40)
			Position(&field->selfRobots[id2], 2.5, by + 8);
		else
			Position(&field->selfRobots[id2], 2.5, -70);
	}
}
void Defend(Field* field, int rID)
{
	if (field->ball.position.x <= 0)
	{
		go(&field->selfRobots[rID], rID, 6, field->ball.position.y);
	}
	else
	{
		double by = field->ball.position.y;
		double bx = field->ball.position.x;
		double rx = field->selfRobots[rID].position.x;
		double ry = field->selfRobots[rID].position.y;
		if (bx >= 75 && bx <= 93 && by >= -40 && by <= 40 && rx <= bx)
		{
			go(&field->selfRobots[rID], rID, 72, ry);
			return;
		}
		if (bx >= 75 && bx <= 93 && by >= -40 && by <= 40 && rx >= bx)
		{
			go(&field->selfRobots[rID], rID, bx + 4, by);
			return;
		}
		if (field->ball.position.x <= 93)
		{
			activeDefender(field, &field->selfRobots[rID], rID);
		}
		else
		{
			if (by >= 42)
			{
				if (ry >= 42)
					activeDefender(field, &field->selfRobots[rID], rID);
				else
					go(&field->selfRobots[rID], rID, rx, 42);

			}
			else if (by <= -42)
			{
				if (ry <= -42)
					activeDefender(field, &field->selfRobots[rID], rID);
				else
					go(&field->selfRobots[rID], rID, rx, 42);
			}
			else
			{
				go(&field->selfRobots[rID], rID, 67.5, by + 5);
			}
		}
	}
}
void Goliar(Field* field)
{
	double bx = field->ball.position.x;
	double by = field->ball.position.y;
	if (bx <= 0)
	{
		if (by >= 25)
			go(&field->selfRobots[0], 0, 110 - 4, 25);
		else if (by <= -25)
			go(&field->selfRobots[0], 0, 110 - 4, -25);
		else
			go(&field->selfRobots[0], 0, 110 - 4, by);
	}
	else
	{
		if (bx >= 71.5 && bx <= 115 && by >= -40 && by <= 40)
		{
			go(&field->selfRobots[0], 0, bx + 3, by);
		}
		else
		{
			if (by >= 25)
				go(&field->selfRobots[0], 0, 110 - 4, 25);
			else if (by <= -25)
				go(&field->selfRobots[0], 0, 110 - 4, -25);
			else
				go(&field->selfRobots[0], 0, 110 - 4, by);
		}
	}
}
void keeper(Robot* robot, int rID, Field* field) {
	double bx, by, rx, ry;
	double le;
	int bv[6] = { 3,10,20,30,40,50 };

	PredictBall2(1, field);

	le = 2.0;
	double X = 110;
	double Y = 0;
	rx = robot->position.x;
	ry = robot->position.y;

	bx = field->ball.position.x;
	by = field->ball.position.y;

	if (PBP[1] > 40)
		PBP[1] = 40;
	if (PBP[1] < -40)
		PBP[1] = -40;
	if (by > 40)
		by = 40;
	if (by < -40)
		by = -40;
	if (bx > 75)
		to(robot, rID, X, PBP[1]);
	else
		go(robot, rID, X, by);

}
