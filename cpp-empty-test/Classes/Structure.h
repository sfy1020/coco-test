#ifndef STRUCTURE_H
#define STRUCTURE_H
//------------------------------------------------------------------------
//
//  Name:   Util.h
//
//  Desc:   Simple structures.
//
//  Author: Insub Im (insooneelife@naver.com)
//
//------------------------------------------------------------------------

#include "BaseGameEntity.h"
#include "Util.h"
#include "RenderingEntity.h"
#include <string>

class Structure : public RenderingEntity {
private:
public:
	Structure(std::string name, cocos2d::Vec2 pos, cocos2d::Vec2 scale, double radius)
		: RenderingEntity(name, "structures", radius, pos, scale)
	{}

	virtual ~Structure() {}

	//this is defined as a pure virtual function in BasegameEntity so it must be implemented
	virtual void update(double time_elapsed) {}

private:
	Structure(const BaseGameEntity&) = delete; // no copies
	Structure& operator=(const Structure&) = delete; // no self-assignments
	Structure() = delete;
};



#endif

