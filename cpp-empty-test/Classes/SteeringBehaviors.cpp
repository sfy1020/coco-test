#include "SteeringBehaviors.h"
#include "GlobalValues.h"
#include "Structure.h"
#include "Transformations.h"
#include "Character.h"
#include "ParameterLoader.h"


using std::string;
using std::vector;
USING_NS_CC;


//------------------------- ctor -----------------------------------------
//
//------------------------------------------------------------------------
SteeringBehavior::SteeringBehavior(MovingEntity* const agent)
	:
	_moving_entity(agent),
	_behavior_set(0),
	_weight_cohesion(Param.CohesionWeight),
	_weight_alignment(Param.AlignmentWeight),
	_weight_separation(Param.SeparationWeight),
	_weight_obstacle_avoidance(Param.ObstacleAvoidanceWeight),
	_weight_wander(Param.WanderWeight),
	_weight_wall_avoidance(Param.WallAvoidanceWeight),
	_view_distance(Param.ViewDistance),
	_deceleration(NORMAL),
	_target_agent(nullptr),
	_wander_distance(Param.WanderDist),
	_wander_jitter(Param.WanderJitterPerSec),
	_wander_radius(Param.WanderRad),
	_waypoint_seek_distSq(Param.WaypointSeekDist * Param.WaypointSeekDist),
	_weight_seek(Param.SeekWeight),
	_weight_flee(Param.FleeWeight),
	_weight_arrive(Param.ArriveWeight),
	_weight_pursuit(Param.PursuitWeight),
	_weight_offset_pursuit(Param.OffsetPursuitWeight),
	_weight_interpose(Param.InterposeWeight),
	_weight_hide(Param.HideWeight),
	_weight_evade(Param.EvadeWeight),
	_weight_follow_path(Param.FollowPathWeight)
{
	//stuff for the wander behavior
	double theta = Util::genRand<0, 1>() * 2 * M_PI;

	//create a vector to a target position on the wander circle
	_wander_target = Vec2(_wander_radius * cos(theta),
		_wander_radius * sin(theta));

	//create a Path
	_path = new Path();
	_path->LoopOn();
}

//---------------------------------dtor ----------------------------------
SteeringBehavior::~SteeringBehavior() { 
	delete _path; 
}


/////////////////////////////////////////////////////////////////////////////// CALCULATE METHODS 


//----------------------- Calculate --------------------------------------
//
//  calculates the accumulated steering force according to the method set
//  in m_SummingMethod
//------------------------------------------------------------------------
const Vec2& SteeringBehavior::calculate()
{
	//reset the steering force
	_steering_force = _steering_force.ZERO;
	if (isOn(SEPARATION) || isOn(ALIGNMENT) || isOn(COHESION))
	{
		_moving_entity->getWorld()->tagVehiclesWithinViewRange(_moving_entity, _view_distance);
	}
	
	_steering_force = CalculatePrioritized();

	return _steering_force;
}

//------------------------- ForwardComponent -----------------------------
//
//  returns the forward oomponent of the steering force
//------------------------------------------------------------------------
double SteeringBehavior::forwardComponent()
{
	return _moving_entity->getHeading().dot(_steering_force);
}

//--------------------------- SideComponent ------------------------------
//  returns the side component of the steering force
//------------------------------------------------------------------------
double SteeringBehavior::sideComponent()
{
	return _moving_entity->getSide().dot(_steering_force);
}


//--------------------- AccumulateForce ----------------------------------
//
//  This function calculates how much of its max steering force the 
//  vehicle has left to apply and then applies that amount of the
//  force to add.
//------------------------------------------------------------------------
bool SteeringBehavior::AccumulateForce(Vec2 &RunningTot, const Vec2& ForceToAdd) {

	//calculate how much steering force the vehicle has used so far
	double MagnitudeSoFar = RunningTot.getLength();

	//calculate how much steering force remains to be used by this vehicle
	double MagnitudeRemaining = _moving_entity->getMaxForce() - MagnitudeSoFar;

	//return false if there is no more force left to use
	if (MagnitudeRemaining <= 0.0) return false;

	//calculate the magnitude of the force we want to add
	double MagnitudeToAdd = ForceToAdd.getLength();

	//if the magnitude of the sum of ForceToAdd and the running total
	//does not exceed the maximum force available to this vehicle, just
	//add together. Otherwise add as much of the ForceToAdd vector is
	//possible without going over the max.
	if (MagnitudeToAdd < MagnitudeRemaining)
	{
		RunningTot += ForceToAdd;
	}

	else
	{
		//add it to the steering force
		RunningTot += (ForceToAdd.getNormalized() * MagnitudeRemaining);
	}

	return true;
}

//---------------------- CalculatePrioritized ----------------------------
//
//  this method calls each active steering behavior in order of priority
//  and acumulates their forces until the max steering force magnitude
//  is reached, at which time the function returns the steering force 
//  accumulated to that  point
//------------------------------------------------------------------------
Vec2 SteeringBehavior::CalculatePrioritized() {
	Vec2 force;

	if (isOn(OBSTACLE_AVOIDANCE))
	{
		force = ObstacleAvoidance(_moving_entity->getWorld()->getObstacles()) *
			_weight_obstacle_avoidance;
		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	if (isOn(EVADE))
	{
		CCASSERT(_target_agent, "Evade target not assigned");
		force = Evade(_target_agent) * _weight_evade;
		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}


	if (isOn(FLEE))
	{
		force = Flee(_moving_entity->getWorld()->getCrossHair()) * _weight_flee;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}



	//these next three can be combined for flocking behavior (wander is
	//also a good behavior to add into this mix)
	
	if (isOn(SEPARATION))
	{
		force = Separation(_moving_entity->getWorld()->getCharacters()) * _weight_separation;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	if (isOn(ALIGNMENT))
	{
		force = Alignment(_moving_entity->getWorld()->getCharacters()) * _weight_alignment;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	if (isOn(COHESION))
	{
		force = Cohesion(_moving_entity->getWorld()->getCharacters()) * _weight_cohesion;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}
	

	if (isOn(SEEK))
	{
		force = Seek(_target) * _weight_seek;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}


	if (isOn(ARRIVE))
	{
		force = Arrive(_moving_entity->getWorld()->getCrossHair(), _deceleration) * _weight_arrive;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	if (isOn(WANDER))
	{
		force = Wander() * _weight_wander;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	if (isOn(PURSUIT))
	{
		CCASSERT(_target_agent, "pursuit target not assigned");

		force = Pursuit(_target_agent) * _weight_pursuit;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	if (isOn(OFFSET_PURSUIT))
	{
		CCASSERT(_target_agent, "pursuit target not assigned");
		CCASSERT(!_offset.isZero(), "No offset assigned");

		force = OffsetPursuit(_target_agent, _offset);

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}



	if (isOn(FOLLOW_PATH))
	{
		force = FollowPath() * _weight_follow_path;

		if (!AccumulateForce(_steering_force, force)) return _steering_force;
	}

	return _steering_force;
}


/////////////////////////////////////////////////////////////////////////////// START OF BEHAVIORS

//------------------------------- Seek -----------------------------------
//
//  Given a target, this behavior returns a steering force which will
//  direct the agent towards the target
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Seek(const Vec2& target_pos)
{
	Vec2 temp = target_pos - _moving_entity->getPos();
	Vec2 desired_velocity = temp.getNormalized() * _moving_entity->getMaxSpeed();

	return (desired_velocity - _moving_entity->getVelocity());
}

//----------------------------- Flee -------------------------------------
//
//  Does the opposite of Seek
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Flee(const Vec2& target_pos)
{
	//only flee if the target is within 'panic distance'. Work in distance
	//squared space.
   /* const double PanicDistanceSq = 100.0f * 100.0;
	if (Vec2DDistanceSq(m_pVehicle->Pos(), target) > PanicDistanceSq)
	{
	  return Vector2D(0,0);
	}
	*/
	Vec2 temp = _moving_entity->getPos() - target_pos;
	Vec2 desired_velocity = temp.getNormalized() * _moving_entity->getMaxSpeed();

	return (desired_velocity - _moving_entity->getVelocity());
}

//--------------------------- Arrive -------------------------------------
//
//  This behavior is similar to seek but it attempts to arrive at the
//  target with a zero velocity
//------------------------------------------------------------------------
#include <algorithm>
Vec2 SteeringBehavior::Arrive(const Vec2& target_pos, Deceleration deceleration)
{
	Vec2 to_target = target_pos - _moving_entity->getPos();

	//calculate the distance to the target
	double dist = to_target.getLength();

	if (dist > 0)
	{
		//because Deceleration is enumerated as an int, this value is required
		//to provide fine tweaking of the deceleration..
		const double deceleration_tweaker = 0.3;

		//calculate the speed required to reach the target given the desired
		//deceleration
		double speed = dist / ((double)deceleration * deceleration_tweaker);

		//make sure the velocity does not exceed the max
		speed = std::min(speed, _moving_entity->getMaxSpeed());

		//from here proceed just like Seek except we don't need to normalize 
		//the ToTarget vector because we have already gone to the trouble
		//of calculating its length: dist. 
		Vec2 DesiredVelocity = to_target * speed / dist;

		return (DesiredVelocity - _moving_entity->getVelocity());
	}

	return Vec2(0, 0);
}

//------------------------------ Pursuit ---------------------------------
//
//  this behavior creates a force that steers the agent towards the 
//  evader
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Pursuit(const MovingEntity* const evader)
{
	//if the evader is ahead and facing the agent then we can just seek
	//for the evader's current position.
	Vec2 to_evader = evader->getPos() - _moving_entity->getPos();

	double relative_heading = _moving_entity->getHeading().dot(evader->getHeading());

	if ((to_evader.dot(_moving_entity->getHeading()) > 0) &&
		(relative_heading < -0.95))  //acos(0.95)=18 degs
	{
		return Seek(evader->getPos());
	}

	//Not considered ahead so we predict where the evader will be.

	//the lookahead time is propotional to the distance between the evader
	//and the pursuer; and is inversely proportional to the sum of the
	//agent's velocities
	double look_ahead_time = to_evader.getLength() /
		(_moving_entity->getMaxSpeed() + evader->getMaxSpeed());

	//now seek to the predicted future position of the evader
	return Seek(evader->getPos() + evader->getVelocity() * look_ahead_time);
}


//----------------------------- Evade ------------------------------------
//
//  similar to pursuit except the agent Flees from the estimated future
//  position of the pursuer
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Evade(const MovingEntity* const pursuer)
{
	/* Not necessary to include the check for facing direction this time */

	Vec2 to_pursuer = pursuer->getPos() - _moving_entity->getPos();

	//uncomment the following two lines to have Evade only consider pursuers 
	//within a 'threat range'
	const double threat_range = 100.0;
	if (to_pursuer.getLengthSq() > threat_range * threat_range) return Vec2();

	//the lookahead time is propotional to the distance between the pursuer
	//and the pursuer; and is inversely proportional to the sum of the
	//agents' velocities
	double look_ahead_time = to_pursuer.getLength() /
		(_moving_entity->getMaxSpeed() + pursuer->getSpeed());

	//now flee away from predicted future position of the pursuer
	return Flee(pursuer->getPos() + pursuer->getVelocity() * look_ahead_time);
}


//--------------------------- Wander -------------------------------------
//
//  This behavior makes the agent wander about randomly
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Wander()
{
	//this behavior is dependent on the update rate, so this line must
	//be included when using time independent framerate.
	double JitterThisTimeSlice = _wander_jitter * _moving_entity->getTimeElapsed();

	//first, add a small random vector to the target's position
	_wander_target += Vec2(Util::randomClamped() * JitterThisTimeSlice,
		Util::randomClamped() * JitterThisTimeSlice);

	//reproject this new vector back on to a unit circle
	_wander_target.getNormalized();

	//increase the length of the vector to the same as the radius
	//of the wander circle
	_wander_target *= _wander_radius;

	//move the target into a position WanderDist in front of the agent
	Vec2 target = _wander_target + Vec2(_wander_distance, 0);

	//project the target into world space
	Vec2 Target = PointToWorldSpace(target,
		_moving_entity->getHeading(),
		_moving_entity->getSide(),
		_moving_entity->getPos());

	//and steer towards it
	return Target - _moving_entity->getPos();
}


//---------------------------- Separation --------------------------------
//
// this calculates a force repelling from the other neighbors
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Separation(const vector<std::shared_ptr<MovingEntity> >& neighbors)
{
	Vec2 steering_force;

	for (unsigned int a = 0; a < neighbors.size(); ++a)
	{
		//make sure this agent isn't included in the calculations and that
		//the agent being examined is close enough. ***also make sure it doesn't
		//include the evade target ***
		if ((neighbors[a].get() != _moving_entity) && neighbors[a]->isTagged() &&
			(neighbors[a].get() != _target_agent))
		{
			Vec2 to_agent = _moving_entity->getPos() - neighbors[a]->getPos();

			//scale the force inversely proportional to the agents distance  
			//from its neighbor.
			Vec2 temp = to_agent.getNormalized();
			steering_force += temp / to_agent.getLength();
		}
	}

	return steering_force;
}


//---------------------------- Alignment ---------------------------------
//
//  returns a force that attempts to align this agents heading with that
//  of its neighbors
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Alignment(const vector<std::shared_ptr<MovingEntity> >& neighbors)
{
	//used to record the average heading of the neighbors
	Vec2 AverageHeading;

	//used to count the number of vehicles in the neighborhood
	int    NeighborCount = 0;

	//iterate through all the tagged vehicles and sum their heading vectors  
	for (unsigned int a = 0; a < neighbors.size(); ++a)
	{
		//make sure *this* agent isn't included in the calculations and that
		//the agent being examined  is close enough ***also make sure it doesn't
		//include any evade target ***
		if ((neighbors[a].get() != _moving_entity) && neighbors[a]->isTagged() &&
			(neighbors[a].get() != _target_agent))
		{
			AverageHeading += neighbors[a]->getHeading();

			++NeighborCount;
		}
	}

	//if the neighborhood contained one or more vehicles, average their
	//heading vectors.
	if (NeighborCount > 0)
	{
		AverageHeading = AverageHeading / (double)NeighborCount;

		AverageHeading -= _moving_entity->getHeading();
	}

	return AverageHeading;
}

//-------------------------------- Cohesion ------------------------------
//
//  returns a steering force that attempts to move the agent towards the
//  center of mass of the agents in its immediate area
//------------------------------------------------------------------------
Vec2 SteeringBehavior::Cohesion(const vector<std::shared_ptr<MovingEntity> > &neighbors)
{
	//first find the center of mass of all the agents
	Vec2 CenterOfMass, SteeringForce;

	int NeighborCount = 0;

	//iterate through the neighbors and sum up all the position vectors
	for (unsigned int a = 0; a < neighbors.size(); ++a)
	{
		//make sure *this* agent isn't included in the calculations and that
		//the agent being examined is close enough ***also make sure it doesn't
		//include the evade target ***
		if ((neighbors[a].get() != _moving_entity) && neighbors[a]->isTagged() &&
			(neighbors[a].get() != _target_agent))
		{
			CenterOfMass += neighbors[a]->getPos();

			++NeighborCount;
		}
	}

	if (NeighborCount > 0)
	{
		//the center of mass is the average of the sum of positions
		CenterOfMass = CenterOfMass / (double)NeighborCount;

		//now seek towards that position
		SteeringForce = Seek(CenterOfMass);
	}

	//the magnitude of cohesion is usually much larger than separation or
	//allignment so it usually helps to normalize it.
	return SteeringForce.getNormalized();
}



//------------------------------- FollowPath -----------------------------
//
//  Given a series of Vector2Ds, this method produces a force that will
//  move the agent along the waypoints in order. The agent uses the
// 'Seek' behavior to move to the next waypoint - unless it is the last
//  waypoint, in which case it 'Arrives'
//------------------------------------------------------------------------

Vec2 SteeringBehavior::FollowPath()
{
	//move to next target if close enough to current target (working in
	//distance squared space)
	
	if (_path->CurrentWaypoint().distance(_moving_entity->getPos()) < _waypoint_seek_distSq)
	{
		_path->SetNextWaypoint();
	}
	

	if (!_path->Finished())
	{
		return Seek(_path->CurrentWaypoint());
	}

	else
	{
		return Arrive(_path->CurrentWaypoint(), NORMAL);
	}
}

//------------------------- Offset Pursuit -------------------------------
//
//  Produces a steering force that keeps a vehicle at a specified offset
//  from a leader vehicle
//------------------------------------------------------------------------
Vec2 SteeringBehavior::OffsetPursuit(const MovingEntity* const leader,
	const Vec2& offset)
{
	//calculate the offset's position in world space
	Vec2 WorldOffsetPos = PointToWorldSpace(offset,
		leader->getHeading(),
		leader->getSide(),
		leader->getPos());

	Vec2 ToOffset = WorldOffsetPos - _moving_entity->getPos();

	//the lookahead time is propotional to the distance between the leader
	//and the pursuer; and is inversely proportional to the sum of both
	//agent's velocities
	double LookAheadTime = ToOffset.getLength() /
		(_moving_entity->getMaxSpeed() + leader->getMaxSpeed());

	//now Arrive at the predicted future position of the offset
	return Arrive(WorldOffsetPos + leader->getVelocity() * LookAheadTime, FAST);
}


//---------------------- ObstacleAvoidance -------------------------------
//
//  Given a vector of CObstacles, this method returns a steering force
//  that will prevent the agent colliding with the closest obstacle
//------------------------------------------------------------------------
Vec2 SteeringBehavior::ObstacleAvoidance(const std::vector<std::shared_ptr<Structure> >& obstacles)
{
	//the detection box length is proportional to the agent's velocity
	_box_length = Param.MinDetectionBoxLength +
		(_moving_entity->getSpeed() / _moving_entity->getMaxSpeed()) *
		Param.MinDetectionBoxLength;
	
	//tag all obstacles within range of the box for processing
	
	_moving_entity->getWorld()->tagObstaclesWithinViewRange(_moving_entity, _box_length);


	//this will keep track of the closest intersecting obstacle (CIB)
	Structure* ClosestIntersectingObstacle = nullptr;

	//this will be used to track the distance to the CIB
	double DistToClosestIP = std::numeric_limits<double>::max();

	//this will record the transformed local coordinates of the CIB
	Vec2 LocalPosOfClosestObstacle;

	std::vector<std::shared_ptr<Structure> >::const_iterator curOb = obstacles.begin();

	while (curOb != obstacles.end())
	{
		//if the obstacle has been tagged within range proceed
		if ((*curOb)->isTagged())
		{
			
			//calculate this obstacle's position in local space
			Vec2 LocalPos = PointToLocalSpace((*curOb)->getPos(),
				_moving_entity->getHeading(),
				_moving_entity->getSide(),
				_moving_entity->getPos());

			//if the local position has a negative x value then it must lay
			//behind the agent. (in which case it can be ignored)
			if (LocalPos.x >= 0)
			{
				//if the distance from the x axis to the object's position is less
				//than its radius + half the width of the detection box then there
				//is a potential intersection.
				double ExpandedRadius = (*curOb)->getBoundingRadius() + _moving_entity->getBoundingRadius();

				if (fabs(LocalPos.y) < ExpandedRadius)
				{
					//now to do a line/circle intersection test. The center of the 
					//circle is represented by (cX, cY). The intersection points are 
					//given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
					//We only need to look at the smallest positive value of x because
					//that will be the closest point of intersection.
					double cX = LocalPos.x;
					double cY = LocalPos.y;
					
					//we only need to calculate the sqrt part of the above equation once
					double SqrtPart = sqrt(ExpandedRadius*ExpandedRadius - cY*cY);

					double ip = cX - SqrtPart;

					if (ip <= 0.0)
					{
						ip = cX + SqrtPart;
					}

					//test to see if this is the closest so far. If it is keep a
					//record of the obstacle and its local coordinates
					if (ip < DistToClosestIP)
					{
						DistToClosestIP = ip;

						ClosestIntersectingObstacle = (*curOb).get();

						LocalPosOfClosestObstacle = LocalPos;
					}
				}
			}
		}

		curOb ++;
	}

	//if we have found an intersecting obstacle, calculate a steering 
	//force away from it
	Vec2 SteeringForce;

	if (ClosestIntersectingObstacle)
	{
		
		//the closer the agent is to an object, the stronger the 
		//steering force should be
		double multiplier = 1.0 + (_box_length - LocalPosOfClosestObstacle.x) /
			_box_length;

		//calculate the lateral force
		SteeringForce.y = (ClosestIntersectingObstacle->getBoundingRadius() -
			LocalPosOfClosestObstacle.y)  * multiplier;

		//apply a braking force proportional to the obstacles distance from
		//the vehicle. 
		const double BrakingWeight = 0.2;

		SteeringForce.x = (ClosestIntersectingObstacle->getBoundingRadius() -
			LocalPosOfClosestObstacle.x) *
			BrakingWeight;
	}

	//finally, convert the steering vector from local to world space
	return VectorToWorldSpace(SteeringForce,
		_moving_entity->getHeading(),
		_moving_entity->getSide());
}