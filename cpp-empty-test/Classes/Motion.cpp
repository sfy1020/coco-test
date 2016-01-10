#include "Motion.h"
#include "Util.h"
#include "CharacterDataPool.h"

USING_NS_CC;	

Motion::~Motion() {}

Motion::Motion(
	const std::string& name,
	const std::string& foldername,
	Sprite* const now_sprite,
	Sprite* const shadow_sprite,
	const Vec2& pos,
	int direction)
	:
	_c_anim_frames(std::get<0>(CharacterPool.insert(name, foldername))),
	_c_motions(std::get<1>(CharacterPool.insert(name, foldername))),
	_now_sprite(now_sprite),
	_shadow_sprite(shadow_sprite),
	_frame(0),
	_motion_frame(0),
	_divide_frame(0) 
{
	setMotion("Walk", direction);
}

void Motion::update(const std::string& new_motion, const Vec2& new_pos, const int& new_dir) {
	_now_sprite->getPositionX();
	_now_sprite->setGlobalZOrder(Util::zOrdering(new_pos.x, new_pos.y));
	_now_sprite->setPosition(new_pos);

	auto animation_frame = _c_anim_frames[_frame];
	_now_sprite->setSpriteFrame(animation_frame);
	_shadow_sprite->setSpriteFrame(animation_frame);
	_shadow_sprite->setPosition(Vec2(0, _now_sprite->getContentSize().height / 2));
	_frame = (_frame + 1) % (_divide_frame * (new_dir + 1) + _motion_frame);

	if (_frame == 0) {
		_frame = _motion_frame + _divide_frame * new_dir;
		setMotion(new_motion, new_dir);
	}
}

void Motion::setMotion(const std::string& new_motion, const int& new_dir) {
	std::map<std::string, std::tuple<int, int> >::const_iterator iter = _c_motions.find(new_motion);
	if (iter != _c_motions.end()) {
		const std::tuple<int, int>& data = iter->second;
		_divide_frame = std::get<0>(data);
		_motion_frame = std::get<1>(data);
	}
	_frame = _motion_frame + _divide_frame * new_dir;
}


//getters & setters
const int& Motion::get_frame() const {
	return _frame;
}

const int& Motion::get_motion_frame() const {
	return _motion_frame;
}

const int& Motion::get_divide_frame() const {
	return _divide_frame;
}