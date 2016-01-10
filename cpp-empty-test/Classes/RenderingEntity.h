#ifndef RENDERING_ENTITY
#define RENDERING_ENTITY
//-------------------------------------------------------------------------------
//
//  Name:   RenderingEntity.h
//
//  Desc:   A base class defining an entity that renders. The entity has 
//          a sprite image and shadow.
//          We can add more rendering things like bounding circle or bounding box.
//
//  Author: Im Insub 
//
//-------------------------------------------------------------------------------

#include <cassert>
#include <memory>
#include "BaseGameEntity.h"
#include "cocos2d.h"


class RenderingEntity : public BaseGameEntity {
public:
	cocos2d::Sprite* forwardSprite() const;

protected:
	cocos2d::Sprite*		_now_sprite;	
	cocos2d::Sprite*		_shadow_sprite;
	cocos2d::DrawNode*		_bounding_circle;
	cocos2d::DrawNode*		_bounding_box;

	RenderingEntity(
		const std::string&	name,
		const std::string&	foldername,
		double radius,
		const cocos2d::Vec2& position,
		const cocos2d::Vec2& scale);

	virtual ~RenderingEntity();

	virtual void update(double time_elapsed);

private:
	RenderingEntity(const RenderingEntity&) = delete; // no copies
	RenderingEntity& operator=(const RenderingEntity&) = delete; // no self-assignments
	RenderingEntity() = delete;
};



#endif