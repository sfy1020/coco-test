#ifndef BASE_GAME_ENTITY_H
#define BASE_GAME_ENTITY_H
#pragma warning (disable:4786)

//------------------------------------------------------------------------
//
//  Name: BaseGameEntity.h
//
//  Desc: Base class to define a common interface for all game
//        entities
//
//  Author: Mat Buckland (fup@ai-junkie.com)
//
//------------------------------------------------------------------------

#include <vector>
#include <string>
#include <iosfwd>
#include <cocos2d.h>

struct Telegram;

class BaseGameEntity {
public:
	enum { 
		DEFAULT_ENTITY_TYPE = -1,
		CHARACTER,
		STRUCTURE,
		MAP,
		ITEM,
		SKILL
	};

	unsigned int			getId() const;

	const cocos2d::Vec2&	getPos() const;
	void					setPos(const cocos2d::Vec2& new_pos);

	double					getBoundingRadius() const;
	void					setBoundingRadius(double r);

	const bool				isTagged() const;
	void					tag();
	void					unTag();

	int						getType() const;
	void					setType(int new_type);

	const cocos2d::Vec2&	getScale() const;
	void					setScale(const cocos2d::Vec2& val);
	void					setScale(double val);

	virtual void			update(double time_elapsed);

	virtual bool			handleMessage(const Telegram& msg);

protected:
	//its location in the environment
	cocos2d::Vec2	_pos;
	cocos2d::Vec2	_scale;

	//the magnitude of this object's bounding radius
	double			_bounding_radius;

	BaseGameEntity::BaseGameEntity(
		double bradius,
		const cocos2d::Vec2& pos,
		const cocos2d::Vec2& scale);

	virtual ~BaseGameEntity();

	//entities should be able to read/write their data to a stream
	virtual void write(std::ostream&  os) const;
	virtual void read(std::ifstream& is);

private:
	BaseGameEntity(const BaseGameEntity&) = delete; // no copies
	BaseGameEntity& operator=(const BaseGameEntity&) = delete; // no self-assignments
	BaseGameEntity() = delete;

	//each entity has a unique ID
	unsigned int	_id;

	//every entity has a type associated with it (health, troll, ammo etc)
	int				_type;

	//this is a generic flag. 
	bool			_tag;
};




#endif




