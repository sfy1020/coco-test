#pragma once
#include "cocos2d.h"
#include <algorithm>
#include <map>

const int DIRECTIONS = 8;
const enum {
	DOWN, LEFT_DOWN, LEFT, LEFT_UP, UP, RIGHT_UP, RIGHT, RIGHT_DOWN
};

//방향판단을 위한 축
const cocos2d::Vec2 axis1 = cocos2d::Vec2(cos(M_PI / 8), sin(M_PI / 8));
const cocos2d::Vec2 axis2 = cocos2d::Vec2(cos(3 * M_PI / 8), sin(3 * M_PI / 8));
const cocos2d::Vec2 axis3 = cocos2d::Vec2(cos(5 * M_PI / 8), sin(5 * M_PI / 8));
const cocos2d::Vec2 axis4 = cocos2d::Vec2(cos(7 * M_PI / 8), sin(7 * M_PI / 8));
const cocos2d::Vec2 axis5 = cocos2d::Vec2(cos(9 * M_PI / 8), sin(9 * M_PI / 8));
const cocos2d::Vec2 axis6 = cocos2d::Vec2(cos(11 * M_PI / 8), sin(11 * M_PI / 8));
const cocos2d::Vec2 axis7 = cocos2d::Vec2(cos(13 * M_PI / 8), sin(13 * M_PI / 8));
const cocos2d::Vec2 axis8 = cocos2d::Vec2(cos(15 * M_PI / 8), sin(15 * M_PI / 8));

const int MAP_ZORDER = 0;
const int SHADOW_ZORDER = 1; 
const int BRADIOUS_ZORDER = 2000000000;