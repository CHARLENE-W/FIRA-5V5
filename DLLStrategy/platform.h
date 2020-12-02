#pragma once
#include <cstdint>

namespace Simuro {

	static const int PLAYERS_PER_SIDE = 5;
	static const int MAX_STRING_LEN = 128;

	enum EventType {
		JudgeResult = 0,
		MatchStart = 1,
		MatchStop = 2,
		FirstHalfStart = 3,
		SecondHalfStart = 4,
		OvertimeStart = 5,
		PenaltyShootoutStart = 6,
	};

	enum JudgeType
	{
		PlaceKick = 0,
		GoalKick = 1,
		PenaltyKick = 2,
		FreeKickRightTop = 3,
		FreeKickRightBot = 4,
		FreeKickLeftTop = 5,
		FreeKickLeftBot = 6,
	};

	enum Team
	{
		Self,
		Opponent,
		None,
	};

	struct Vector2 {
		float x;
		float y;
	};

	struct TeamInfo {
		wchar_t teamName[MAX_STRING_LEN];
	};

	struct Ball {
		Vector2 position;
	};

	struct Wheel {
		float leftSpeed;
		float rightSpeed;
	};

	struct Robot {
		Vector2 position;
		float rotation;
		Wheel wheel;
	};

	struct Field {
		Robot selfRobots[PLAYERS_PER_SIDE];
		Robot opponentRobots[PLAYERS_PER_SIDE];
		Ball ball;
		int32_t tick;
	};

	struct JudgeResultEvent {
		JudgeType type;
		Team actor;
		wchar_t reason[MAX_STRING_LEN];
	};
}
