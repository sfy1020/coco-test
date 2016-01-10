#include "CharacterDataPool.h"
#include "GlobalValues.h"
#include "Util.h"

USING_NS_CC;

const std::tuple<
	std::vector<SpriteFrame*>,
	std::map<std::string, std::tuple<int, int> >
>&	
CharacterDataPool::insert(
	const std::string& name,
	const std::string& foldername,
	const std::string& format) 
{
	auto already = _pool.find(name);
	if (already != end(_pool)) {
		return already->second;
	}

	rapidjson::Document json;
	Util::readJSON(foldername + "/" + name + "/data.json", json);

	std::vector<SpriteFrame*> animFrames;
	std::map<std::string, std::tuple<int, int> > actions;

	for (rapidjson::Value::MemberIterator M = json["actions"].MemberBegin(); M != json["actions"].MemberEnd(); M++) {
		std::string key = M->name.GetString();
		actions.insert(std::make_pair(key, std::make_tuple(M->value["frameSize"].GetInt(), 0)));
	}
	
	auto frameCache = SpriteFrameCache::getInstance();
	char number[10] = { 0 };
	int accum = 0;

	for (std::map<std::string, std::tuple<int, int>>::iterator iter = begin(actions); iter != end(actions); iter++) {
		
		std::string filename = foldername + "/" + name + "/" + iter->first + ".plist";
		frameCache->addSpriteFramesWithFile(filename);
		iter->second = std::make_tuple(std::get<0>(iter->second), accum);

		for (int i = 1; i <= DIRECTIONS * std::get<0>(iter->second); i++) {
			sprintf(number, "_%02d.", i);
			std::string tempstr = iter->first + number + format;

			auto frame = frameCache->getSpriteFrameByName(tempstr);
			animFrames.push_back(frame);
		}
		accum += std::get<0>(iter->second) * DIRECTIONS;
	}

	auto result = _pool.insert(std::make_pair(name, std::make_tuple(animFrames, actions)));
	return result.first->second;
}
