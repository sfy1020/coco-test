#pragma once
#include "cocos2d.h"
#include <string>
#include <map>
#include <tuple>
#include <vector>
#include "GlobalValues.h"
#include"BaseGameEntity.h"

//------------------------------------------------------------------------
//
//  Name:   Motion.h
//
//  Desc:   This class selects animation frame, 
//			which is suitable for current state
//
//  Author: Insub Im (insooneelife@naver.com)
//
//------------------------------------------------------------------------

class Motion {
private:	//values
	const std::vector<cocos2d::SpriteFrame*>& _c_anim_frames;
	const std::map<std::string, std::tuple<int, int> >& _c_motions;
	cocos2d::Sprite* const _now_sprite;
	cocos2d::Sprite* const _shadow_sprite;
	int _frame;
	int _motion_frame;
	int _divide_frame;

private:	//functions
	Motion() = delete;
	Motion(const Motion& copy) = delete;
	Motion& operator=(const Motion& copy) = delete;

public:
	~Motion();
	Motion(
		const std::string& name,
		const std::string& foldername,
		cocos2d::Sprite* const now_sprite,
		cocos2d::Sprite* const shadow_sprite,
		const cocos2d::Vec2& pos,
		int direction = DOWN);

	void update(const std::string& new_motion, const cocos2d::Vec2& new_pos, const int& new_dir);
	void setMotion(const std::string& new_motion, const int& new_dir);

public:
	//getters & setters
	const int& get_frame() const;
	const int& get_motion_frame() const;
	const int& get_divide_frame() const;
};