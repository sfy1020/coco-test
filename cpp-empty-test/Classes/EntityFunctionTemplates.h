#ifndef GAME_ENTITY_FUNCTION_TEMPLATES
#define GAME_ENTITY_FUNCTION_TEMPLATES

#include "BaseGameEntity.h"
#include "geometry.h"
#include "cocos2d.h"

//////////////////////////////////////////////////////////////////////////
//
//  Some useful template functions
//
//////////////////////////////////////////////////////////////////////////

//------------------------- Overlapped -----------------------------------
//
//  tests to see if an entity is overlapping any of a number of entities
//  stored in a std container
//------------------------------------------------------------------------
template <class T, class ConT>
bool overlapped(const T* ob, const ConT& conOb, double min_dist_between_obstacles = 40.0)
{
	typename ConT::const_iterator it;

	for (it = conOb.begin(); it != conOb.end(); ++it)
	{
		if (TwoCirclesOverlapped(ob->getPos(),
			ob->getBoundingRadius() + min_dist_between_obstacles,
			(*it)->getPos(),
			(*it)->getBoundingRadius()))
		{
			return true;
		}
	}

	return false;
}

//----------------------- TagNeighbors ----------------------------------
//
//  tags any entities contained in a std container that are within the
//  radius of the single entity parameter
//------------------------------------------------------------------------
template <class T, class ConT>
void tagNeighbors(T entity, ConT& others, const double radius)
{
	typename ConT::iterator it;

	//iterate through all entities checking for range
	for (it = others.begin(); it != others.end(); ++it) {
		//first clear any current tag
		(*it)->unTag();

		//work in distance squared to avoid sqrts
		cocos2d::Vec2 to = (*it)->getPos() - entity->getPos();

		//the bounding radius of the other is taken into account by adding it 
		//to the range
		double range = radius + (*it)->getBoundingRadius();

		//if entity within range, tag for further consideration
		if (((*it)->getId() != entity->getId()) && (to.getLengthSq() < range*range))
		{
			(*it)->tag();
		}
	}//next entity
}


//------------------- EnforceNonPenetrationContraint ---------------------
//
//  Given a pointer to an entity and a std container of pointers to nearby
//  entities, this function checks to see if there is an overlap between
//  entities. If there is, then the entities are moved away from each
//  other
//------------------------------------------------------------------------
template <class T, class ConT>
void enforceNonPenetrationConstraint(T entity, const ConT& others)
{
	typename ConT::const_iterator it;

	//iterate through all entities checking for any overlap of bounding
	//radii
	for (it = others.begin(); it != others.end(); ++it)
	{
		//make sure we don't check against this entity
		if ((*it)->getId() == entity->getId()) continue;

		//calculate the distance between the positions of the entities
		cocos2d::Vec2 to_entity = entity->getPos() - (*it)->getPos();

		double dist_from_each_other = to_entity.getLength();

		//if this distance is smaller than the sum of their radii then this
		//entity must be moved away in the direction parallel to the
		//ToEntity vector   
		double amount_of_overlap = (*it)->getBoundingRadius() + entity->getBoundingRadius() -
			dist_from_each_other;

		if (amount_of_overlap >= 0) {
			//move the entity a distance away equivalent to the amount of overlap.
			entity->setPos(entity->getPos() + (to_entity / dist_from_each_other) *
				amount_of_overlap);
		}
	}//next entity
}










#endif