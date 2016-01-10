#pragma once

#include "Motion.h"
#include "StateMachine.h"
#include "MovingEntity.h"
#include <memory>
#include <string>

//------------------------------------------------------------------------
//
//  Name:   Character.h
//
//  Desc:   This class makes data structure for character from json file and
//			shares it for every class which needs them.
//
//  Author: Insub Im (insooneelife@naver.com)
//
//------------------------------------------------------------------------

class Character : public MovingEntity {
public:
	Character(const std::string& cname,
		GameWorld*		world,
		const cocos2d::Vec2&	position,
		const double&			rotation,
		const cocos2d::Vec2&	velocity,
		const double&			mass,
		const double&			max_force,
		const double&			max_speed,
		const double&			max_turn_rate,
		const double&			scale);

	virtual ~Character();

	virtual void update(double time_elapsed);

	virtual bool handleMessage(const Telegram& msg);

private:
	Character(const Character&) = delete; // no copies
	Character& operator=(const Character&) = delete; // no self-assignments
	Character() = delete;

	std::unique_ptr<StateMachine<Character> > _state_machine;
};




