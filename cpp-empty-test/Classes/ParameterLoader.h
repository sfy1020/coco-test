#pragma once
#include "Singleton.h"
#include <map>
#include <string>

//provide easy access
#define Param ParameterLoader::getInstance()

class ParameterLoader : public Singleton<ParameterLoader>{
private:
	ParameterLoader();

public:
	double NumAgents;
	double SteeringForceTweaker;
	double SteeringForce;
	double MaxSpeed;
	double VehicleMass;
	double VehicleScale;
	double SeparationWeight;
	double AlignmentWeight;
	double CohesionWeight;
	double ObstacleAvoidanceWeight;
	double WallAvoidanceWeight;
	double WanderWeight;
	double SeekWeight;
	double FleeWeight;
	double ArriveWeight;
	double PursuitWeight;
	double OffsetPursuitWeight;
	double InterposeWeight;
	double HideWeight;
	double EvadeWeight;
	double FollowPathWeight;
	double ViewDistance;
	double MinDetectionBoxLength;
	double WallDetectionFeelerLength;
	double NumSamplesForSmoothing;
	double WanderRad;
	double WanderDist;
	double WanderJitterPerSec;
	double WaypointSeekDist;
	double StartViewX;
	double StartViewY;
	double CameraDistance;
	double GameSpeedInterval;
	double BorderSize;
	double MouseMoveVelocity;
	double NumObstacles;
	double MinObstacleRadius;
	double MaxObstacleRadius;

private:
	ParameterLoader(const ParameterLoader&) = delete; // no copies
	ParameterLoader& operator=(const ParameterLoader&) = delete; // no self-assignments
	friend class Singleton<ParameterLoader>;
};