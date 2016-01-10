#include "Path.h"
#include "transformations.h"
#include "Util.h"
#include <algorithm>

USING_NS_CC;

std::list<Vec2> Path::CreateRandomPath(int   NumWaypoints,
	double MinX,
	double MinY,
	double MaxX,
	double MaxY)
{
	m_WayPoints.clear();

	double midX = (MaxX + MinX) / 2.0;
	double midY = (MaxY + MinY) / 2.0;

	double smaller = std::min(midX, midY);

	double spacing = M_PI * 2 / (double)NumWaypoints;

	CCLOG("(");
	for (int i = 0; i < NumWaypoints; ++i)
	{
		double RadialDist = smaller*0.2f + Util::genRand<0, 1>() * (smaller - smaller * 0.2f);
		Vec2 temp(RadialDist, 0.0f);

		Vec2DRotateAroundOrigin(temp, i*spacing);

		temp.x += midX; temp.y += midY;

		m_WayPoints.push_back(temp);
		CCLOG("(%lf, %lf)", temp.x, temp.y);
	}
	CCLOG(")");

	curWaypoint = m_WayPoints.begin();
	return m_WayPoints;
}
