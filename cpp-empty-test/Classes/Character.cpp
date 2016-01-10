#include "Character.h"
#include "WalkState.h"
#include "GlobalValues.h"

USING_NS_CC;

Character::Character(
	const std::string& cname,
	GameWorld*				world,
	const cocos2d::Vec2&	position,
	const double&			rotation,
	const cocos2d::Vec2&	velocity,
	const double&			mass,
	const double&			max_force,
	const double&			max_speed,
	const double&			max_turn_rate,
	const double&			scale)
	: 
	MovingEntity(
		cname,
		"characters",
		position,
		scale,
		velocity,
		max_speed,
		Vec2(sin(rotation), -cos(rotation)),
		mass,
		Vec2(scale, scale),
		max_turn_rate,
		max_force,
		world),
	_state_machine(new StateMachine<Character>(this))
{
	_state_machine->SetCurrentState(new WalkState());
}

Character::~Character()
{}

void Character::update(double time_elapsed) 
{
	_state_machine->Update();
	MovingEntity::update(time_elapsed);
}

bool Character::handleMessage(const Telegram& msg)
{
	return false; 
}

//getters & setters


/*
void FieldPlayer::Update()
{
	//run the logic for the current state
	m_pStateMachine->Update();

	//calculate the combined steering force
	m_pSteering->Calculate();

	//if no steering force is produced decelerate the player by applying a
	//braking force
	if (m_pSteering->Force().isZero())
	{
		const double BrakingRate = 0.8;

		m_vVelocity = m_vVelocity * BrakingRate;
	}

	//the steering force's side component is a force that rotates the 
	//player about its axis. We must limit the rotation so that a player
	//can only turn by PlayerMaxTurnRate rads per update.
	double TurningForce = m_pSteering->SideComponent();

	Clamp(TurningForce, -Prm.PlayerMaxTurnRate, Prm.PlayerMaxTurnRate);

	//rotate the heading vector
	Vec2DRotateAroundOrigin(m_vHeading, TurningForce);

	//make sure the velocity vector points in the same direction as
	//the heading vector
	m_vVelocity = m_vHeading * m_vVelocity.Length();

	//and recreate m_vSide
	m_vSide = m_vHeading.Perp();


	//now to calculate the acceleration due to the force exerted by
	//the forward component of the steering force in the direction
	//of the player's heading
	Vector2D accel = m_vHeading * m_pSteering->ForwardComponent() / m_dMass;

	m_vVelocity += accel;

	//make sure player does not exceed maximum velocity
	m_vVelocity.Truncate(m_dMaxSpeed);

	//update the position
	m_vPosition += m_vVelocity;


	//enforce a non-penetration constraint if desired
	if (Prm.bNonPenetrationConstraint)
	{
		EnforceNonPenetrationContraint(this, AutoList<PlayerBase>::GetAllMembers());
	}
}*/