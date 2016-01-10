#include "Util.h"
#include "ParameterLoader.h"

ParameterLoader::ParameterLoader() {
	rapidjson::Document json;
	Util::readJSON("params.json", json);
	for (rapidjson::Value::MemberIterator M = json.MemberBegin(); M != json.MemberEnd(); M++) {
		std::string name = M->name.GetString();
		double value = M->value.GetDouble();
		if ("NumAgents" == name) {
			NumAgents = value;
		}
		else if ("SteeringForceTweaker" == name) {
			SteeringForceTweaker = value;
		}
		else if ("SteeringForce" == name) {
			SteeringForce = value;
		}
		else if ("MaxSpeed" == name) {
			MaxSpeed = value;
		}
		else if ("VehicleMass" == name) {
			VehicleMass = value;
		}
		else if ("VehicleScale" == name) {
			VehicleScale = value;
		}
		else if ("SeparationWeight" == name) {
			SeparationWeight = value;
		}
		else if ("AlignmentWeight" == name) {
			AlignmentWeight = value;
		}
		else if ("CohesionWeight" == name) {
			CohesionWeight = value;
		}
		else if ("ObstacleAvoidanceWeight" == name) {
			ObstacleAvoidanceWeight = value;
		}
		else if ("WallAvoidanceWeight" == name) {
			WallAvoidanceWeight = value;
		}
		else if ("WanderWeight" == name) {
			WanderWeight = value;
		}
		else if ("SeekWeight" == name) {
			SeekWeight = value;
		}
		else if ("FleeWeight" == name) {
			FleeWeight = value;
		}
		else if ("ArriveWeight" == name) {
			ArriveWeight = value;
		}
		else if ("PursuitWeight" == name) {
			PursuitWeight = value;
		}
		else if ("OffsetPursuitWeight" == name) {
			OffsetPursuitWeight = value;
		}
		else if ("InterposeWeight" == name) {
			InterposeWeight = value;
		}
		else if ("HideWeight" == name) {
			HideWeight = value;
		}
		else if ("EvadeWeight" == name) {
			EvadeWeight = value;
		}
		else if ("FollowPathWeight" == name) {
			FollowPathWeight = value;
		}
		else if ("ViewDistance" == name) {
			ViewDistance = value;
		}
		else if ("MinDetectionBoxLength" == name) {
			MinDetectionBoxLength = value;
		}
		else if ("WallDetectionFeelerLength" == name) {
			WallDetectionFeelerLength = value;
		}
		else if ("NumSamplesForSmoothing" == name) {
			NumSamplesForSmoothing = value;
		}
		else if ("WanderRad" == name) {
			WanderRad = value;
		}
		else if ("WanderDist" == name) {
			WanderDist = value;
		}
		else if ("WanderJitterPerSec" == name) {
			WanderJitterPerSec = value;
		}
		else if ("WaypointSeekDist" == name) {
			WaypointSeekDist = value;
		}
		else if ("StartViewX" == name) {
			StartViewX = value;
		}
		else if ("StartViewY" == name) {
			StartViewY = value;
		}
		else if ("CameraDistance" == name) {
			CameraDistance = value;
		}
		else if ("GameSpeedInterval" == name) {
			GameSpeedInterval = value;
		}
		else if ("BorderSize" == name) {
			BorderSize = value;
		}
		else if ("MouseMoveVelocity" == name) {
			MouseMoveVelocity = value;
		}
		else if ("NumObstacles" == name) {
			NumObstacles = value;
		}
		else if ("MinObstacleRadius" == name) {
			MinObstacleRadius = value;
		}
		else if ("MaxObstacleRadius" == name) {
			MaxObstacleRadius = value;
		}
	}
}