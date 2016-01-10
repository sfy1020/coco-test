#ifndef PATH_H
#define PATH_H
//------------------------------------------------------------------------
//
//  Name:   Path.h
//
//  Desc:   class to define, manage, and traverse a path (defined by a series of 2D vectors)
//          
//
//  Author: Mat Buckland 2003 (fup@ai-junkie.com)
//
//------------------------------------------------------------------------
#include <list>
#include <cassert>

#include "cocos2d.h"


class Path
{
private:

	std::list<cocos2d::Vec2>            m_WayPoints;

	//points to the current waypoint
	std::list<cocos2d::Vec2>::iterator  curWaypoint;

	//flag to indicate if the path should be looped
	//(The last waypoint connected to the first)
	bool                           m_bLooped;

public:

	Path() :m_bLooped(false), curWaypoint(std::list<cocos2d::Vec2>::iterator()) {}

	//constructor for creating a path with initial random waypoints. MinX/Y
	//& MaxX/Y define the bounding box of the path.
	Path(int    NumWaypoints,
		double MinX,
		double MinY,
		double MaxX,
		double MaxY,
		bool   looped) :m_bLooped(looped)
	{
		CreateRandomPath(NumWaypoints, MinX, MinY, MaxX, MaxY);

		curWaypoint = m_WayPoints.begin();
	}


	//returns the current waypoint
	cocos2d::Vec2    CurrentWaypoint()const {
		return *curWaypoint; 
	}

	//returns true if the end of the list has been reached
	bool        Finished() { return !(curWaypoint != m_WayPoints.end()); }

	//moves the iterator on to the next waypoint in the list
	inline void SetNextWaypoint();

	//creates a random path which is bound by rectangle described by
	//the min/max values
	std::list<cocos2d::Vec2> CreateRandomPath(int    NumWaypoints,
		double MinX,
		double MinY,
		double MaxX,
		double MaxY);


	void addPath(cocos2d::Vec2 pos) {
		m_WayPoints.push_back(pos);
	}

	void LoopOn() { m_bLooped = true; }
	void LoopOff() { m_bLooped = false; }

	//methods for setting the path with either another Path or a list of vectors
	void Set(std::list<cocos2d::Vec2> new_path) { m_WayPoints = new_path; curWaypoint = m_WayPoints.begin(); }
	void Set(const Path& path) { m_WayPoints = path.GetPath(); curWaypoint = m_WayPoints.begin(); }


	void Clear() { m_WayPoints.clear(); }

	std::list<cocos2d::Vec2> GetPath()const { return m_WayPoints; }

};




//-------------------- Methods -------------------------------------------

inline void Path::SetNextWaypoint()
{
	assert(m_WayPoints.size() > 0);

	if (++curWaypoint == m_WayPoints.end())
	{
		if (m_bLooped)
		{
			curWaypoint = m_WayPoints.begin();
		}
	}
}



#endif