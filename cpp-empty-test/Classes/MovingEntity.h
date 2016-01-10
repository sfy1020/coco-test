#ifndef MOVING_ENTITY
#define MOVING_ENTITY
//------------------------------------------------------------------------
//
//  Name:   MovingEntity.h
//
//  Desc:   A base class defining an entity that moves. The entity has 
//          a local coordinate system and members for defining its
//          mass and velocity.
//
//  Author: Im Insub
//
//------------------------------------------------------------------------

#include <cassert>
#include <memory>
#include "RenderingEntity.h"
#include "cocos2d.h"
#include "GameWorld.h"
#include "SteeringBehaviors.h"
#include "Smoother.h"
#include "Motion.h"

class MovingEntity : public RenderingEntity {
public:
	bool					isSpeedMaxedOut() const;
	double					getSpeed() const;
	double					getSpeedSq() const;
	bool					rotateHeadingToFacePosition(const cocos2d::Vec2& target);

	const std::unique_ptr<SteeringBehavior>&	get_steering() const;
	GameWorld* const		getWorld() const;

	const cocos2d::Vec2&	getVelocity() const;
	void					setVelocity(const cocos2d::Vec2& velocity);

	double					getMass() const;
	void					setMass(double mass);

	const cocos2d::Vec2&	getSide() const;
	void					setSide(const cocos2d::Vec2& side);

	double					getMaxSpeed() const;
	void					setMaxSpeed(double max_speed);

	double					getMaxForce() const;
	void					setMaxForce(double max_force);

	const cocos2d::Vec2&	getHeading() const;
	void					setHeading(const cocos2d::Vec2& new_heading);

	double					getMaxTurnRate() const;
	void					setMaxTurnRate(double max_turn_rate);

	int						getDirection() const;
	void					setDirection(int direction);

	const std::string&		getMotion() const;
	void					setMotion(const std::string& motion);

	double					getTimeElapsed();
	void					setTimeElapsed(double time_elapsed);

	virtual void			update(double time_elapsed);

protected:

	cocos2d::Vec2			_velocity;

	//a normalized vector pointing in the direction the entity is heading. 
	cocos2d::Vec2			_heading;

	//a vector perpendicular to the heading vector
	cocos2d::Vec2			_side;

	double					_mass;

	//the maximum speed this entity may travel at.
	double					_max_speed;

	//the maximum force this entity can produce to power itself 
	//(think rockets and thrust)
	double					_max_force;

	//the maximum rate (radians per second)this vehicle can rotate         
	double					_max_turn_rate;

	double					_time_elapsed;

	//The following members are used to smooth the vehicle's heading
	Smoother<cocos2d::Vec2>	_heading_smoother;

	//Motion class for animate image frame.
	std::unique_ptr<Motion>	_motion;
	int						_direction;
	std::string				_motion_name;

	//The steering behavior class.
	std::unique_ptr<SteeringBehavior>	_steering;

	//A pointer for reference the world data.
	//So a vehicle can access any obstacle, path, wall or agent data.
	//Weak reference for remove circular reference.
	GameWorld*				_game_world;

	MovingEntity(
		const std::string&		name,
		const std::string&		foldername,
		const cocos2d::Vec2&	position,
		double					radius,
		const cocos2d::Vec2&	velocity,
		double					max_speed,
		const cocos2d::Vec2&	heading,
		double					mass,
		const cocos2d::Vec2&	scale,
		double					turn_rate,
		double					max_force,
		GameWorld* const		world);

	virtual ~MovingEntity();

private:
	MovingEntity(const MovingEntity&) = delete; // no copies
	MovingEntity& operator=(const MovingEntity&) = delete; // no self-assignments
	MovingEntity() = delete;
};



#endif