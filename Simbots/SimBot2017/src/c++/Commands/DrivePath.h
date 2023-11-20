#ifndef DrivePath_H
#define DrivePath_H

#include "../CommandBase.h"
#include "pathfinder.h"

#define MAX_POINTS 8

class DrivePath : public CommandBase {
	int length;
	Segment *trajectory=NULL;
	Segment *leftTrajectory=NULL;
	Segment *rightTrajectory=NULL;
	DistanceFollower leftFollower;
	DistanceFollower rightFollower;
	FollowerConfig config;
	int num_points;
	int side;
	TrajectoryCandidate candidate;
	Waypoint points[MAX_POINTS];


	void ModifyTank();
	double FollowDistance(DistanceFollower *follower, Segment *trajectory, double distance);

public:
	enum mode { LEFT, RIGHT,CENTER};
	DrivePath(int side);
	~DrivePath();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	void cleanup();
};

#endif  // DrivePath_H
