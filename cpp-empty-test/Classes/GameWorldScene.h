#ifndef __GAMEWORLD_SCENE_H__
#define __GAMEWORLD_SCENE_H__

//------------------------------------------------------------------------
//
//  Name:   GameWorldScene.h
//
//  Desc:   This class makes the game play scene. 
//			So it has a layer for show game entity,
//			and a camera for user's view,
//			and gameworld for data interact.
//
//  Author: Insub Im (insooneelife@naver.com)
//
//------------------------------------------------------------------------

#include "cocos2d.h"
#include "Character.h"
#include <memory>

class GameWorld;

class GameWorldScene : public cocos2d::Layer {
public:
	// implement the "static create()" method manually
	CREATE_FUNC(GameWorldScene);
	virtual bool init() override;
	static cocos2d::Scene* scene();

	~GameWorldScene();

	void setGameWorldShared(const std::shared_ptr<GameWorld>& ref);

	// a selector callback
	void menuCloseCallback(cocos2d::Ref* sender);
	void go(float fDelta);

private:
	// scene interacts with game world.
	std::shared_ptr<GameWorld>	_game_world;

	// all visible entity renders on this layer.
	cocos2d::Layer*				_veiw_layer;

	// camera which we look at.
	cocos2d::Camera*			_veiw_camera;
	cocos2d::Vec3				_look_at;

	// for mouse movement
	cocos2d::Vec2				_move_pos;
	float						_prev_mouse_x;
	float						_prev_mouse_y;
};

#endif // __HELLOWORLD_SCENE_H__
