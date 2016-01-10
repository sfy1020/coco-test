#include "MovingEntity.h"
#include "ParameterLoader.h"
#include "Util.h"
#include "C2DMatrix.h"

USING_NS_CC;

MovingEntity::MovingEntity(
		const std::string&		name,
		const std::string&		foldername,
		const Vec2&	position,
		double					radius,
		const Vec2&	velocity,
		double					max_speed,
		const Vec2&	heading,
		double					mass,
		const Vec2&	scale,
		double					turn_rate,
		double					max_force,
		GameWorld* const world)
		:
	RenderingEntity(name, foldername, radius, position, scale),
		_heading(heading),
		_velocity(velocity),
		_mass(mass),
		_side(_heading.getPerp()),
		_max_speed(max_speed),
		_max_turn_rate(turn_rate),
		_max_force(max_force),
		_game_world(world),
		_motion(new Motion(name, foldername, _now_sprite, _shadow_sprite, position)),
		_direction(Util::make_direction(_heading)),
		_motion_name("Walk"),
		_steering(new SteeringBehavior(this)),
		_heading_smoother(
			Smoother<Vec2>(Param.NumSamplesForSmoothing, Vec2(0.0, 0.0)))
{}

MovingEntity::~MovingEntity() 
{}

void MovingEntity::update(double time_elapsed) 
{
	//update the time elapsed
	_time_elapsed = time_elapsed;

	//keep a record of its old position so we can update its cell later in this method
	Vec2 old_pos = _pos;
	Vec2 steering_force;

	//calculate the combined force from each steering behavior in the  vehicle's list
	steering_force = _steering->calculate();

	Vec2 acceleration = steering_force / _mass;

	//update velocity
	_velocity += acceleration * time_elapsed;

	//make sure vehicle does not exceed maximum velocity
	if (_velocity.getLength() > _max_speed) {
		_velocity.normalize();
		_velocity *= _max_speed;
	}

	//update the position
	_pos += _velocity * time_elapsed;

	//update the heading if the vehicle has a non zero velocity
	if (_velocity.getLengthSq() > 0.00000001)
	{
		_heading = _velocity;
		_heading = _heading_smoother.Update(_heading);
		_heading.normalize();

		_side = _heading.getPerp();
	}

	int new_dir = Util::make_direction(_heading);

	if (new_dir != _direction) {
		_direction = new_dir;
		_motion->setMotion(_motion_name, new_dir);
	}
	_motion->update(_motion_name, _pos, new_dir);

	RenderingEntity::update(time_elapsed);
}


//--------------------------- rotateHeadingToFacePosition ---------------------
//
//  given a target position, this method rotates the entity's heading and
//  side vectors by an amount not greater than m_dMaxTurnRate until it
//  directly faces the target.
//
//  returns true when the heading is facing in the desired direction
//-----------------------------------------------------------------------------
inline bool MovingEntity::rotateHeadingToFacePosition(const Vec2& target)
{
	Vec2 toTarget = target - _pos;
	toTarget.normalize();

	double dot = _heading.dot(toTarget);

	//some compilers lose acurracy so the value is clamped to ensure it
	//remains valid for the acos
	Util::clamp(dot, -1, 1);

	//first determine the angle between the heading vector and the target
	double angle = acos(dot);

	//return true if the player is facing the target
	if (angle < 0.00001) return true;

	//clamp the amount to turn to the max turn rate
	if (angle > _max_turn_rate) angle = _max_turn_rate;

	//The next few lines use a rotation matrix to rotate the player's heading
	//vector accordingly

	C2DMatrix RotationMatrix;

	//notice how the direction of rotation has to be determined when creating
	//the rotation matrix
	RotationMatrix.Rotate(angle * Util::sign(_heading, toTarget));
	RotationMatrix.Rotate(angle * Util::sign(_heading, toTarget));
	RotationMatrix.TransformVector2Ds(_heading);
	RotationMatrix.TransformVector2Ds(_velocity);

	//finally recreate m_vSide
	_side = _heading.getPerp();

	return false;
}

//------------------------- SetHeading ----------------------------------------
//
//  first checks that the given heading is not a vector of zero length. If the
//  new heading is valid this fumction sets the entity's heading and side 
//  vectors accordingly
//-----------------------------------------------------------------------------
inline void MovingEntity::setHeading(const Vec2& new_heading)
{
	CC_ASSERT((new_heading.getLengthSq() - 1.0) < 0.00001);

	_heading = new_heading;

	//the side vector must always be perpendicular to the heading
	_side = _heading.getPerp();
}


bool MovingEntity::isSpeedMaxedOut() const
{
	return _max_speed*_max_speed >= _velocity.getLengthSq();
}

double MovingEntity::getSpeed() const 
{
	return _velocity.getLength();
}

double MovingEntity::getSpeedSq() const 
{
	return _velocity.getLengthSq(); 
}

const std::unique_ptr<SteeringBehavior>& MovingEntity::get_steering() const 
{
	return _steering;
}

GameWorld* const MovingEntity::getWorld() const 
{
	return _game_world;
}

const Vec2& MovingEntity::getVelocity() const 
{
	return _velocity; 
}

void MovingEntity::setVelocity(const Vec2& velocity) 
{
	_velocity = velocity; 
}

double MovingEntity::getMass() const 
{ 
	return _mass; 
}

void MovingEntity::setMass(double mass)
{
	_mass = mass; 
}

const Vec2& MovingEntity::getSide() const 
{
	return _side; 
}

void MovingEntity::setSide(const Vec2& side) 
{
	_side = side; 
}

double MovingEntity::getMaxSpeed() const
{
	return _max_speed; 
}

void MovingEntity::setMaxSpeed(double max_speed) 
{
	_max_speed = max_speed; 
}

double MovingEntity::getMaxForce() const 
{
	return _max_force; 
}

void MovingEntity::setMaxForce(double max_force) 
{
	_max_force = max_force; 
}

const Vec2& MovingEntity::getHeading() const 
{
	return _heading; 
}

double MovingEntity::getMaxTurnRate() const 
{
	return _max_turn_rate; 
}

void MovingEntity::setMaxTurnRate(double max_turn_rate) 
{
	_max_turn_rate = max_turn_rate; 
}

int MovingEntity::getDirection() const 
{
	return _direction;
}

void MovingEntity::setDirection(int direction)
{
	_direction = direction;
}

const std::string& MovingEntity::getMotion() const
{
	return _motion_name;
}

void MovingEntity::setMotion(const std::string& motion) 
{
	_motion_name = motion;
}

double MovingEntity::getTimeElapsed() {
	return _time_elapsed;
}

void MovingEntity::setTimeElapsed(double time_elapsed)
{
	this->_time_elapsed = time_elapsed;
}
