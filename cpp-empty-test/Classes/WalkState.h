#pragma once
#include "State.h"

class Character;

class WalkState : public State<Character> {
public:
	WalkState(){}

public:
	void Enter(Character* character) {}

	void Execute(Character* character);

	void Exit(Character* character) {}

	bool OnMessage(Character*, const Telegram&);
};





