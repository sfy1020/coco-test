#ifndef STEERINGBEHAVIORS_H
#define STEERINGBEHAVIORS_H
#pragma warning (disable:4786)
//------------------------------------------------------------------------
//
//  Name:   SteeringBehaviors.h
//
//  Desc:   class to encapsulate steering behaviors for a Vehicle
//
//  Author: Mat Buckland 2002 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------
#include <vector>
#include <string>
#include <list>

#include "cocos2d.h"
#include "Path.h"

class BaseGameEntity;
class MovingEntity;
class Structure;

class SteeringBehavior {
private:
	enum behavior_type
	{
		NONE = 0x00000,
		SEEK = 0x00002,
		FLEE = 0x00004,
		ARRIVE = 0x00008,
		WANDER = 0x00010,
		COHESION = 0x00020,
		SEPARATION = 0x00040,
		ALIGNMENT = 0x00080,
		OBSTACLE_AVOIDANCE = 0x00100,
		WALL_AVOIDANCE = 0x00200,
		FOLLOW_PATH = 0x00400,
		PURSUIT = 0x00800,
		EVADE = 0x01000,
		INTERPOSE = 0x02000,
		HIDE = 0x04000,
		FLOCK = 0x08000,
		OFFSET_PURSUIT = 0x10000,
	};

public:
	SteeringBehavior(MovingEntity* const agent);
	virtual ~SteeringBehavior();

	//calculates and sums the steering forces from any active behaviors
	const cocos2d::Vec2& calculate();

	//calculates the component of the steering force that is parallel
	//with the vehicle heading
	double    forwardComponent();

	//calculates the component of the steering force that is perpendicuar
	//with the vehicle heading
	double    sideComponent();

	void      createRandomPath(int num_waypoints, int mx, int my, int cx, int cy) const
	{
		_path->CreateRandomPath(num_waypoints, mx, my, cx, cy);
	}

	void fleeOn() { _behavior_set |= FLEE; }
	void seekOn() { _behavior_set |= SEEK; }
	void arriveOn() { _behavior_set |= ARRIVE; }
	void wanderOn() { _behavior_set |= WANDER; }
	void pursuitOn(MovingEntity* const v) { _behavior_set |= PURSUIT; _target_agent = v; }
	void evadeOn(MovingEntity* const v) { _behavior_set |= EVADE; _target_agent = v; }
	void cohesionOn() { _behavior_set |= COHESION; }
	void separationOn() { _behavior_set |= SEPARATION; }
	void alignmentOn() { _behavior_set |= ALIGNMENT; }
	void obstacleAvoidanceOn() { _behavior_set |= OBSTACLE_AVOIDANCE; }
	void wallAvoidanceOn() { _behavior_set |= WALL_AVOIDANCE; }
	void followPathOn() { _behavior_set |= FOLLOW_PATH; }
	void hideOn(MovingEntity* const v) { _behavior_set |= HIDE; _target_agent = v; }
	void offsetPursuitOn(MovingEntity* const v, const cocos2d::Vec2& offset) { _behavior_set |= OFFSET_PURSUIT; _offset = offset; _target_agent = v; }
	void flockingOn() { cohesionOn(); alignmentOn(); separationOn(); wanderOn(); }

	void fleeOff() { if (isOn(FLEE))   _behavior_set ^= FLEE; }
	void seekOff() { if (isOn(SEEK))   _behavior_set ^= SEEK; }
	void arriveOff() { if (isOn(ARRIVE)) _behavior_set ^= ARRIVE; }
	void wanderOff() { if (isOn(WANDER)) _behavior_set ^= WANDER; }
	void pursuitOff() { if (isOn(PURSUIT)) _behavior_set ^= PURSUIT; }
	void evadeOff() { if (isOn(EVADE)) _behavior_set ^= EVADE; }
	void cohesionOff() { if (isOn(COHESION)) _behavior_set ^= COHESION; }
	void separationOff() { if (isOn(SEPARATION)) _behavior_set ^= SEPARATION; }
	void alignmentOff() { if (isOn(ALIGNMENT)) _behavior_set ^= ALIGNMENT; }
	void obstacleAvoidanceOff() { if (isOn(OBSTACLE_AVOIDANCE)) _behavior_set ^= OBSTACLE_AVOIDANCE; }
	void wallAvoidanceOff() { if (isOn(WALL_AVOIDANCE)) _behavior_set ^= WALL_AVOIDANCE; }
	void followPathOff() { if (isOn(FOLLOW_PATH)) _behavior_set ^= FOLLOW_PATH; }
	void interposeOff() { if (isOn(INTERPOSE)) _behavior_set ^= INTERPOSE; }
	void hideOff() { if (isOn(HIDE)) _behavior_set ^= HIDE; }
	void offsetPursuitOff() { if (isOn(OFFSET_PURSUIT)) _behavior_set ^= OFFSET_PURSUIT; }
	void flockingOff() { cohesionOff(); alignmentOff(); separationOff(); wanderOff(); }

	bool isFleeOn() { return isOn(FLEE); }
	bool isSeekOn() { return isOn(SEEK); }
	bool isArriveOn() { return isOn(ARRIVE); }
	bool isWanderOn() { return isOn(WANDER); }
	bool isPursuitOn() { return isOn(PURSUIT); }
	bool isEvadeOn() { return isOn(EVADE); }
	bool isCohesionOn() { return isOn(COHESION); }
	bool isSeparationOn() { return isOn(SEPARATION); }
	bool isAlignmentOn() { return isOn(ALIGNMENT); }
	bool isObstacleAvoidanceOn() { return isOn(OBSTACLE_AVOIDANCE); }
	bool isWallAvoidanceOn() { return isOn(WALL_AVOIDANCE); }
	bool isFollowPathOn() { return isOn(FOLLOW_PATH); }
	bool isInterposeOn() { return isOn(INTERPOSE); }
	bool isHideOn() { return isOn(HIDE); }
	bool isOffsetPursuitOn() { return isOn(OFFSET_PURSUIT); }


	void      set_target(const cocos2d::Vec2& t) { _target = t; }

	void      set_target_agent(MovingEntity* const agent) { _target_agent = agent; }

	void      set_offset(const cocos2d::Vec2& offset) { _offset = offset; }
	const cocos2d::Vec2&  get_offset() const { return _offset; }

	void      set_path(const std::list<cocos2d::Vec2>& new_path) { _path->Set(new_path); }

	const cocos2d::Vec2& get_steering_force() const { return _steering_force; }

	double get_weight_separation()const { return _weight_separation; }
	double get_weight_alignment()const { return _weight_alignment; }
	double get_weight_cohesion()const { return _weight_cohesion; }


	double get_wander_jitter()const { return _wander_jitter; }
	double get_wander_distance()const { return _wander_distance; }
	double get_wander_radius()const { return _wander_radius; }

	const int get_behavior_set()const { return _behavior_set; }


	double get_box_length()const { return _box_length; }
	void set_box_length(const double& box_length) {
		_box_length = box_length;
	}

	double get_wall_detection_feeler_length()const { return _wall_detection_feeler_length; }
	void set_path(const double& feeler_length) { _wall_detection_feeler_length = feeler_length; }

	const std::vector<cocos2d::Vec2>& get_feelers()const { return _feelers; }
	void set_path(const std::vector<cocos2d::Vec2>& feelers) { _feelers = feelers; }


	private:
		SteeringBehavior() = delete;	//no default constructors
		SteeringBehavior(const SteeringBehavior&) = delete; // no copies
		SteeringBehavior& operator=(const SteeringBehavior&) = delete; // no self-assignments

	private:

		//a pointer to the owner of this instance
		MovingEntity*		_moving_entity;

		//these can be used to keep track of friends, pursuers, or prey
		MovingEntity*		_target_agent;

		//the steering force created by the combined effect of all
		//the selected behaviors
		cocos2d::Vec2		_steering_force;

		//the current target
		cocos2d::Vec2		_target;

		//the current position on the wander circle the agent is attempting to steer towards
		cocos2d::Vec2		_wander_target;

		//explained above
		double				_wander_jitter;
		double				_wander_radius;
		double				_wander_distance;

		//multipliers. These can be adjusted to effect strength of the  
		//appropriate behavior. Useful to get flocking the way you require
		//for example.
		double				_weight_separation;
		double				_weight_cohesion;
		double				_weight_alignment;
		double				_weight_wander;
		double				_weight_obstacle_avoidance;
		double				_weight_wall_avoidance;
		double				_weight_seek;
		double				_weight_flee;
		double				_weight_arrive;
		double				_weight_pursuit;
		double				_weight_offset_pursuit;
		double				_weight_interpose;
		double				_weight_hide;
		double				_weight_evade;
		double				_weight_follow_path;

		//how far the agent can 'see'
		double				_view_distance;

		//pointer to any current path
		Path*				_path;

		//the distance (squared) a vehicle has to be from a path waypoint before
		//it starts seeking to the next waypoint
		double				_waypoint_seek_distSq;

		//any offset used for formations or offset pursuit
		cocos2d::Vec2		_offset;

		//binary flags to indicate whether or not a behavior should be active
		int					_behavior_set;

		//length of the 'detection box' utilized in obstacle avoidance
		double                 _box_length;

		//a vertex buffer to contain the feelers rqd for wall avoidance  
		std::vector<cocos2d::Vec2> _feelers;

		//the length of the 'feeler/s' used in wall detection
		double                 _wall_detection_feeler_length;


		//Arrive makes use of these to determine how quickly a vehicle
		//should decelerate to its target
		enum Deceleration { SLOW = 3, NORMAL = 2, FAST = 1 };

		//default
		Deceleration		_deceleration;

		/* .......................................................

		BEGIN BEHAVIOR DECLARATIONS

		.......................................................*/

		//this behavior moves the agent towards a target position
		cocos2d::Vec2 Seek(const cocos2d::Vec2& TargetPos);

		//this behavior returns a vector that moves the agent away
		//from a target position
		cocos2d::Vec2 Flee(const cocos2d::Vec2& TargetPos);

		//this behavior is similar to seek but it attempts to arrive 
		//at the target position with a zero velocity
		cocos2d::Vec2 Arrive(const cocos2d::Vec2&     TargetPos,
			Deceleration deceleration);

		//this behavior predicts where an agent will be in time T and seeks
		//towards that point to intercept it.
		cocos2d::Vec2 Pursuit(const MovingEntity* const agent);

		//this behavior maintains a position, in the direction of offset
		//from the target vehicle
		cocos2d::Vec2 OffsetPursuit(const MovingEntity* const agent, const cocos2d::Vec2& offset);

		//this behavior attempts to evade a pursuer
		cocos2d::Vec2 Evade(const MovingEntity* const agent);

		//this behavior makes the agent wander about randomly
		cocos2d::Vec2 Wander();

		//given a series of Vector2Ds, this method produces a force that will
		//move the agent along the waypoints in order
		cocos2d::Vec2 FollowPath();

		cocos2d::Vec2 ObstacleAvoidance(const std::vector<std::shared_ptr<Structure> >& obstacles);

		// -- Group Behaviors -- //

		cocos2d::Vec2 Cohesion(const std::vector<std::shared_ptr<MovingEntity> >& agents);

		cocos2d::Vec2 Separation(const std::vector<std::shared_ptr<MovingEntity> >& agents);

		cocos2d::Vec2 Alignment(const std::vector<std::shared_ptr<MovingEntity> >& agents);

		/* .......................................................

		END BEHAVIOR DECLARATIONS

		.......................................................*/

		//this function tests if a specific bit of m_iFlags is set
		bool		isOn(behavior_type bt) { return (_behavior_set & bt) == bt; }

		bool		AccumulateForce(cocos2d::Vec2 &sf, const cocos2d::Vec2& ForceToAdd);

		//calculates and sums the steering forces from any active behaviors
		cocos2d::Vec2 CalculatePrioritized();
};




#endif