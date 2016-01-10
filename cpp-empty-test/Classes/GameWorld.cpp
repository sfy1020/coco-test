#include "GameWorld.h"
#include "Util.h"
#include "GlobalValues.h"
#include "Transformations.h"
#include "SteeringBehaviors.h"
#include "cocos2d.h"
#include "EntityManager.h"
#include "Character.h"
#include "ParameterLoader.h"
#include "AppMacros.h"
#include "Structure.h"
#include <list>

using std::list;
USING_NS_CC;

//------------------------------- ctor -----------------------------------
//------------------------------------------------------------------------
GameWorld::GameWorld() : _cross_hair(Vec2(0, 0)){

	createCharacters();

	createStructures();

	ParameterLoader::getInstance();
}


//-------------------------------- dtor ----------------------------------
//------------------------------------------------------------------------
GameWorld::~GameWorld() { CCLOG("~GameWorld()"); }


//----------------------------- update -----------------------------------
//------------------------------------------------------------------------
void GameWorld::update(double time_elapsed) {
	//update the vehicles
	for (unsigned int a = 0; a < _characters.size(); ++a) {
		_characters[a]->update(time_elapsed);
	}
}

void GameWorld::createCharacters() {
	for (int a = 0; a < Param.NumAgents; ++a) {
		//determine a random starting position
		float window_width = designResolutionSize.width;
		float window_height = designResolutionSize.height;
		Vec2 spawn_pos = Vec2(window_width / 2.0 + Util::randomClamped()*window_height / 2.0,
			window_width / 2.0 + Util::randomClamped()*window_height / 2.0);


		_characters.emplace_back(
			new Character(
				"Andariel",
				this,
				spawn_pos,											//initial position
				Util::genRand<0, 1>() * M_PI * 2,		//start rotation
				Vec2(0, 0),											//velocity
				Param.VehicleMass,									//mass
				Param.SteeringForce * Param.SteeringForceTweaker,	//max force
				Param.MaxSpeed,										//max velocity
				M_PI / 2,											//max turn rate
				Param.VehicleScale)
			);

	}

	Vec2 pos[5] = { Vec2(50, 50), Vec2(700, 300), Vec2(850, 100), Vec2(100, 900), Vec2(900, 900) };
	std::list<Vec2> tempPath;
	for (int j = 0; j < 5; j++) {
		tempPath.push_back(pos[j]);
	}

	_characters[0]->get_steering()->followPathOn();
	_characters[0]->get_steering()->set_path(tempPath);
	_characters[0]->get_steering()->obstacleAvoidanceOn();

	for (int i = 1; i < Param.NumAgents - 4; ++i) {
		_characters[i]->get_steering()->offsetPursuitOn(_characters[0].get(), Vec2(Util::genRand<0, 200>() - 100, Util::genRand<0, 200>() - 100));
		_characters[i]->get_steering()->obstacleAvoidanceOn();
	}

	_characters[Param.NumAgents - 4]->get_steering()->seekOn();
	_characters[Param.NumAgents - 4]->get_steering()->set_target(Vec2(4480 / 2, 0));
	_characters[Param.NumAgents - 3]->get_steering()->seekOn();
	_characters[Param.NumAgents - 3]->get_steering()->set_target(Vec2(0, 2440 / 2));
	_characters[Param.NumAgents - 2]->get_steering()->seekOn();
	_characters[Param.NumAgents - 2]->get_steering()->set_target(Vec2(4480 / 2, 2440 / 2));
	_characters[Param.NumAgents - 1]->get_steering()->seekOn();

	for (auto it = begin(_characters); it != end(_characters); ++it) {
		EntityMgr.registerEntity(*it);
	}
}

//--------------------------- createStructures -----------------------------
//
//  Sets up the vector of obstacles with random positions and sizes. Makes
//  sure the obstacles do not overlap
//------------------------------------------------------------------------
void GameWorld::createStructures()
{
	//create a number of randomly sized tiddlywinks
	for (int o = 0; o<Param.NumObstacles; ++o) {
		bool bOverlapped = true;

		//keep creating tiddlywinks until we find one that doesn't overlap
		//any others.Sometimes this can get into an endless loop because the
		//obstacle has nowhere to fit. We test for this case and exit accordingly

		int NumTrys = 0; int NumAllowableTrys = 200;

		while (bOverlapped) {
			NumTrys++;

			if (NumTrys > NumAllowableTrys) return;

			double radius = Util::genRand((double)Param.MinObstacleRadius, (double)Param.MaxObstacleRadius);

			const int border = 10;
			const int MinGapBetweenObstacles = 20;

			Structure* ob = new Structure(
				"structure1",
				Vec2(Util::genRand(radius + border, 2000 - radius - border),
					Util::genRand(radius + border, 1000 - radius - 30 - border)),
				Vec2(0,0),
				radius);

			if (!overlapped(ob, _structures, MinGapBetweenObstacles)) {
				//its not overlapped so we can add it
				_structures.emplace_back(ob);

				bOverlapped = false;
			}
			else {
				delete ob;
			}
		}
	}

	for (auto it = begin(_structures); it != end(_structures); ++it) {
		EntityMgr.registerEntity(*it);
	}
}


void  GameWorld::tagVehiclesWithinViewRange(MovingEntity* const pVehicle, double range)
{
	tagNeighbors(pVehicle, _characters, range);
}

void  GameWorld::tagObstaclesWithinViewRange(BaseGameEntity* const pVehicle, double range)
{
	tagNeighbors(pVehicle, _structures, range);
}

void GameWorld::nonPenetrationContraint(MovingEntity* const v)
{
	enforceNonPenetrationConstraint(v, _characters);
}


const std::vector< std::shared_ptr<MovingEntity> >& GameWorld::getCharacters()
{
	return _characters; 
}

const std::vector< std::shared_ptr<Structure> >& GameWorld::getObstacles() const
{
	return _structures; 
}

const cocos2d::Vec2& GameWorld::getCrossHair()const
{
	return _cross_hair; 
}

void GameWorld::setCrossHair(const cocos2d::Vec2& v) {
	_cross_hair = v; 
}