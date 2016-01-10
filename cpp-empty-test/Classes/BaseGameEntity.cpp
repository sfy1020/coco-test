#include "BaseGameEntity.h"
#include "IDGenerator.h"

USING_NS_CC;

BaseGameEntity::BaseGameEntity(
	double bradius,
	const Vec2& pos,
	const Vec2& scale)
	:
	_id(IDGen.genID()),
	_type(DEFAULT_ENTITY_TYPE),
	_tag(false),
	_bounding_radius(bradius),
	_pos(pos),
	_scale(scale)
{}

BaseGameEntity::~BaseGameEntity() 
{}

void BaseGameEntity::update(double time_elapsed) 
{}

bool BaseGameEntity::handleMessage(const Telegram& msg) 
{ 
	return false; 
}

//entities should be able to read/write their data to a stream
void BaseGameEntity::write(std::ostream&  os) const 
{}

void BaseGameEntity::read(std::ifstream& is) 
{}

//getters & setters
unsigned int	BaseGameEntity::getId() const
{
	return _id;
}

const Vec2& BaseGameEntity::getPos() const
{
	return _pos;
}

void BaseGameEntity::setPos(const Vec2& new_pos) 
{
	_pos = new_pos;
}

double BaseGameEntity::getBoundingRadius() const 
{
	return _bounding_radius;
}

void BaseGameEntity::setBoundingRadius(double r) 
{
	_bounding_radius = r;
}

const bool BaseGameEntity::isTagged() const 
{
	return _tag;
}

void BaseGameEntity::tag()
{
	_tag = true;
}

void BaseGameEntity::unTag() 
{
	_tag = false;
}

int BaseGameEntity::getType() const 
{
	return _type;
}

void BaseGameEntity::setType(int new_type) 
{
	_type = new_type;
}

const Vec2& BaseGameEntity::getScale() const
{
	return _scale;
}

void BaseGameEntity::setScale(const Vec2& val) 
{
	_bounding_radius *= std::max(val.x, val.y) / std::max(_scale.x, _scale.y);
	_scale = val;
}

void BaseGameEntity::setScale(double val) 
{
	_bounding_radius *= (val / std::max(_scale.x, _scale.y));
	_scale = Vec2(val, val);
}


