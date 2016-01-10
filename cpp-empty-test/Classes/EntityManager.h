#ifndef ENTITYMANAGER_H
#define ENTITYMANAGER_H
#pragma warning (disable:4786)

//------------------------------------------------------------------------
//
//  Name:   EntityManager.h
//
//  Desc:   Singleton class to handle the  management of Entities.          
//
//  Author: Mat Buckland (fup@ai-junkie.com)
//
//------------------------------------------------------------------------

#include <map>
#include <memory>
#include "Singleton.h"

class BaseGameEntity;

//provide easy access
#define EntityMgr EntityManager::getInstance()

class EntityManager : public Singleton<EntityManager> {
public:
	typedef std::map<unsigned int, std::shared_ptr<BaseGameEntity> > EntityMap;

	//returns a pointer to the entity with the ID given as a parameter
	const std::shared_ptr<BaseGameEntity>& getEntityFromID(unsigned int id) const;

	//this method stores a pointer to the entity in the std::vector
	//m_Entities at the index position indicated by the entity's ID
	//(makes for faster access)
	void				registerEntity(const std::shared_ptr<BaseGameEntity>& new_entity);

	//this method removes the entity from the list
	void				removeEntity(const std::shared_ptr<BaseGameEntity>& p_entity);

	//clears all entities from the entity map
	void				reset();

	const EntityMap&	getAllEntity() const;

private:
	EntityManager(const EntityManager&) = delete; // no copies
	EntityManager& operator=(const EntityManager&) = delete; // no self-assignments
	EntityManager() {}
	friend class Singleton<EntityManager>;

	EntityMap _entity_map;
};



#endif