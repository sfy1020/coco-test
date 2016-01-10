#include "RenderingEntity.h"
#include "Util.h"

USING_NS_CC;

RenderingEntity::RenderingEntity(
	const std::string&	name,
	const std::string&	foldername,
	double radius,
	const  Vec2& position,
	const  Vec2& scale)
	:
	BaseGameEntity(radius, position, scale)
{
	//--------------------------Main Sprite Setting-------------------------------//

	_now_sprite =  Sprite::create(foldername + "/" + name + "/" + name + ".png");

	float textureX = _now_sprite->getTextureRect().getMaxX() - _now_sprite->getTextureRect().getMinX();
	float textureY = _now_sprite->getTextureRect().getMaxY() - _now_sprite->getTextureRect().getMinY();

	_now_sprite->setPosition(position);
	_now_sprite->setGlobalZOrder(Util::zOrdering(position.x, position.y));
	_now_sprite->setLocalZOrder(Util::zOrdering(position.x, position.y));

	float midx = _now_sprite->getContentSize().width / 2;
	float midy = _now_sprite->getContentSize().height / 2;


	//--------------------------Shadow Sprite Setting-------------------------------//

	_shadow_sprite =  Sprite::create(foldername + "/" + name + "/" + name + ".png");
	_shadow_sprite->setSkewX(-45);
	_shadow_sprite->setColor( Color3B(0, 0, 0));
	_shadow_sprite->setGlobalZOrder(SHADOW_ZORDER);
	_shadow_sprite->setPosition( Vec2(0, midy));
	_now_sprite->addChild(_shadow_sprite);


	//--------------------------Bounding Circle Setting-------------------------------//

	 Color4F color =  Color4F::WHITE;
	_bounding_circle =  DrawNode::create();
	_bounding_circle->drawCircle(
		 Vec2(textureX / 2, textureX / 4),
		_bounding_radius,
		(float)(2 * M_PI),
		40, false, color);
	_bounding_circle->setGlobalZOrder(BRADIOUS_ZORDER);
	_bounding_circle->setLineWidth(1);
	_now_sprite->addChild(_bounding_circle);


	//--------------------------Bounding Box Setting-------------------------------//

	color =  Color4F::RED;
	_bounding_box =  DrawNode::create();
	_bounding_box->drawRect(
		 Vec2(
			_now_sprite->getTextureRect().getMinX(),
			_now_sprite->getTextureRect().getMinY()
			),
		 Vec2(
			_now_sprite->getTextureRect().getMaxX(),
			_now_sprite->getTextureRect().getMaxY()
			),
		color
		);
	_bounding_box->setLineWidth(1);
	_bounding_box->setGlobalZOrder(BRADIOUS_ZORDER);
	_now_sprite->addChild(_bounding_box);
}

RenderingEntity::~RenderingEntity()
{}

 Sprite* RenderingEntity::forwardSprite() const 
{
	return _now_sprite;
}

void RenderingEntity::update(double time_elapsed) 
{}

