#include "GameWorldScene.h"
#include "AppMacros.h"
#include "EntityManager.h"
#include "PrecisionTimer.h"
#include "Util.h"
#include "Structure.h"
#include <iostream>
#include <string>

USING_NS_CC;

PrecisionTimer timer;

Scene* GameWorldScene::scene() {
	// 'scene' is an autorelease object
	auto scene = Scene::create();

	// 'layer' is an autorelease object
	GameWorldScene *layer = GameWorldScene::create();

	// add layer as a child to scene
	scene->addChild(layer);

	// return the scene
	return scene;
}

// on "init" you need to initialize your instance
bool GameWorldScene::init()
{
	if (!Layer::init())
		return false;

	//------------------------User Interface Setting-----------------------------//

	auto visibleSize = Director::getInstance()->getVisibleSize();
	auto origin = Director::getInstance()->getVisibleOrigin();

	auto closeItem = MenuItemImage::create(
		"CloseNormal.png",
		"CloseSelected.png",
		CC_CALLBACK_1(GameWorldScene::menuCloseCallback, this));

	closeItem->setPosition(origin + Vec2(visibleSize) - Vec2(closeItem->getContentSize() / 2));

	// create menu, it's an autorelease object
	auto menu = Menu::create(closeItem, nullptr);
	menu->setPosition(Vec2::ZERO);
	this->addChild(menu, 1);

	auto label = Label::createWithTTF("Game World", "fonts/arial.ttf", TITLE_FONT_SIZE);

	// position the label on the center of the screen
	label->setPosition(origin.x + visibleSize.width / 2,
		origin.y + visibleSize.height - label->getContentSize().height);

	// add the label as a child to this layer
	this->addChild(label, 1);



	//--------------------------GameWorld Setting-------------------------------//
	_veiw_layer = Layer::create();
	this->addChild(_veiw_layer);

	// if you want to use shared_ptr without constructing, use like this.
	_game_world.reset(new GameWorld());

	for (auto e : _game_world->getCharacters()) {
		_veiw_layer->addChild(e->forwardSprite());
	}

	for (auto e : _game_world->getObstacles()) {
		_veiw_layer->addChild(e->forwardSprite());
	}

	timer.SmoothUpdatesOn();

	//start the timer
	timer.Start();
	this->schedule(CC_SCHEDULE_SELECTOR(GameWorldScene::go), Param.GameSpeedInterval);

	//--------------------------Tile Map Setting-------------------------------//
	int kTagTileMap = 1;
	auto map = TMXTiledMap::create("tilemap/diablo2_court2.tmx");
	_veiw_layer->addChild(map, 0, kTagTileMap);

	Size CC_UNUSED map_size = map->getContentSize();

	// move map to the center of the screen

	auto ms = map->getMapSize();
	auto ts = map->getTileSize();
	float lengthx = (ms.width + 0.5f) * ts.width;
	float lengthy = (ms.height + 1) * ts.height / 2;
	map->setGlobalZOrder(MAP_ZORDER);
	//map->setPosition(Vec2(-lengthx /4, -lengthy /4));

	//--------------------------Camera Setting-------------------------------//

	//this is the layer, when adding camera to it, all its children will be affect only when you set the second parameter to true
	_veiw_layer->setCameraMask((unsigned short)CameraFlag::USER2, true);

	_veiw_camera = Camera::createPerspective(60, (float)visibleSize.width / visibleSize.height, 1.0, 1000);
	_veiw_camera->setCameraFlag(CameraFlag::USER2);

	//the calling order matters, we should first call setPosition3D, then call lookAt.
	_look_at = Vec3(Param.StartViewX, Param.StartViewY, 0.0);
	_veiw_camera->setPosition3D(_look_at + Vec3(0.0, 0.0, Param.CameraDistance));
	_veiw_camera->lookAt(_look_at + Vec3(0.0, 1.0, 0.0));
	_veiw_layer->addChild(_veiw_camera);


	//--------------------------Event Setting-------------------------------//

	auto listener = cocos2d::EventListenerMouse::create();
	listener->onMouseDown = [this](cocos2d::EventMouse* e) {
		if (e->getMouseButton() == MOUSE_BUTTON_LEFT) {}
		else if (e->getMouseButton() == MOUSE_BUTTON_RIGHT) {}
		else if (e->getMouseButton() == MOUSE_BUTTON_MIDDLE) {}
		else {}
	};

	listener->onMouseMove = [this](cocos2d::EventMouse* e) {
		float mx = e->getCursorX();
		float my = e->getCursorY();
		float center_x = designResolutionSize.width / 2;
		float center_y = designResolutionSize.height / 2;
		_move_pos = Vec2(0, 0);

		if (Util::isBorder(mx, my)) {
			_move_pos = Vec2(mx - center_x, my - center_y);
			_move_pos.normalize();
			_move_pos *= Param.MouseMoveVelocity;
		}

		if (e->getMouseButton() == MOUSE_BUTTON_LEFT) {}
		else if (e->getMouseButton() == MOUSE_BUTTON_RIGHT) {}
		else if (e->getMouseButton() == MOUSE_BUTTON_MIDDLE) {}
		else {}
	};

	listener->onMouseUp = [this](cocos2d::EventMouse* e) {
		if (e->getMouseButton() == MOUSE_BUTTON_LEFT) {}
		else if (e->getMouseButton() == MOUSE_BUTTON_RIGHT) {}
		else if (e->getMouseButton() == MOUSE_BUTTON_MIDDLE) {}
		else {}
	};

	//마우스 이벤트 등록
	_eventDispatcher->addEventListenerWithSceneGraphPriority(listener, _veiw_layer);

	return true;
}

GameWorldScene::~GameWorldScene() 
{}

void GameWorldScene::setGameWorldShared(const std::shared_ptr<GameWorld>& ref)
{ 
	_game_world = ref;
}

void GameWorldScene::go(float fDelta) 
{
	_game_world->update(timer.TimeElapsed());
	_look_at.x += _move_pos.x;
	_look_at.y += _move_pos.y;
	_veiw_camera->setPosition3D(_look_at + Vec3(0.0, 0.0, Param.CameraDistance));
	_veiw_camera->lookAt(_look_at + Vec3(0.0, 1.0, 0.0));
}


void GameWorldScene::menuCloseCallback(Ref* sender)
{
	Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
	exit(0);
#endif
}
