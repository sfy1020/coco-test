#ifndef GameWorld_H
#define GameWorld_H
#pragma warning (disable:4786)
//------------------------------------------------------------------------
//
//  Name:   GameWorld.h
//
//  Desc:   All the environment data and methods for the Steering
//          Behavior projects. This class is the root of the project's
//          update and render calls (excluding main of course)
//
//  Author: Mat Buckland 2002 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------
#include <vector>
#include <memory>
#include "cocos2d.h"
#include "BaseGameEntity.h"
#include "EntityFunctionTemplates.h"

class Path;
class MovingEntity;
class Structure;

class GameWorld {
public:
	GameWorld();
	~GameWorld();

	void	update(double time_elapsed);

	void	nonPenetrationContraint(MovingEntity* const v);

	void	tagVehiclesWithinViewRange(MovingEntity* const pVehicle, double range);

	void	tagObstaclesWithinViewRange(BaseGameEntity* const pVehicle, double range);

	void	createCharacters();

	void	createStructures();

	const cocos2d::Vec2&	getCrossHair()const;
	
	void					setCrossHair(const cocos2d::Vec2& v);

	const std::vector< std::shared_ptr<MovingEntity> >&		getCharacters();

	const std::vector< std::shared_ptr<Structure> >&		getObstacles()const;

private:
	//a container of all the moving entities
	std::vector< std::shared_ptr<MovingEntity> >  _characters;

	//any structures
	std::vector<std::shared_ptr<Structure> >  _structures;

	//the position of the crosshair
	cocos2d::Vec2	_cross_hair;
};



#endif