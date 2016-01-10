#include "EntityManager.h"
#include "BaseGameEntity.h"

const std::shared_ptr<BaseGameEntity>& EntityManager::getEntityFromID(unsigned int id) const 
{
  //find the entity
  EntityMap::const_iterator ent = _entity_map.find(id);

  //assert that the entity is a member of the map
  assert ( (ent !=  _entity_map.end()) && "<EntityManager::GetEntityFromID>: invalid ID");

  return ent->second;
}

void EntityManager::removeEntity(const std::shared_ptr<BaseGameEntity>& p_entity) 
{
	_entity_map.erase(_entity_map.find(p_entity->getId()));
} 

void EntityManager::reset() {
	_entity_map.clear(); 
}

void EntityManager::registerEntity(const std::shared_ptr<BaseGameEntity>& new_entity) 
{
	_entity_map.insert(std::make_pair(new_entity->getId(), new_entity));
}

const EntityManager::EntityMap& EntityManager::getAllEntity() const
{
	return _entity_map;
}




