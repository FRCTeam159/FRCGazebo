#include "DrivePath.h"

#define I2M(x) x*0.0254
#define F2M(x) x*0.0254*12


//#define PRINT_PATH
#define PLOT_PATH
#define PLOT_TRAJECTORY
#define DEBUG_COMMAND
//#define USE_GYRO  // in gazebo wheels can slip while turning (known bug) so may need to use gyro to correct
#define GFACT 5   // gyro correction factor

#define TIME_STEP 0.02
#define MAX_VEL 1.0 //
#define MAX_ACC 0.5
#define MAX_JRK 0.5
#define KP 1.2
#define KI 0.5
#define KD 0.0
#define KV 1.0/MAX_VEL
#define KA 0.0
#define WHEELBASE_WIDTH  0.66

static Timer mytimer;
static double runtime;

DrivePath::DrivePath(int s) : config{KP,KI,KD,KV,KA} {
	Requires(driveTrain.get());
	points[0] = { 0, 0, 0 };
	side=s;
	switch(side){
	case LEFT:
		num_points=2;
		points[1] = { 1.9, -0.7, d2r(-60) };
		break;
	case RIGHT:
		num_points=2;
		points[1] = { 1.9, 0.7, d2r(60) };
		break;
	case CENTER:
		num_points=2;
		points[1] = { 1.4, 0.0, 0 };
		break;
	}

    std::cout << "new DrivePath("<<side<<")"<< std::endl;


	// Prepare the Trajectory for Generation (example)
	//
	// Arguments:
	// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
	//                      PATHFINDER_SAMPLES_LOW  (10 000)
	//                      PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step:           0.001 Seconds
	// Max Velocity:        15 m/s
	// Max Acceleration:    10 m/s/s
	// Max Jerk:            60 m/s/s/s

	pathfinder_prepare(points, num_points, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, TIME_STEP, MAX_VEL, MAX_ACC, MAX_JRK, &candidate);

	length = candidate.length;

	// Array of Segments (the trajectory points) to store the trajectory in
	trajectory = (Segment*)malloc(length * sizeof(Segment));
	leftTrajectory = (Segment*)malloc(length * sizeof(Segment));
	rightTrajectory = (Segment*)malloc(length * sizeof(Segment));

	// Generate the trajectory
	int result = pathfinder_generate(&candidate, trajectory);
	if (result < 0) {
	    // An error occured
	    printf("Uh-Oh! Trajectory could not be generated!\n");
		cleanup();
	    return;
	}
	runtime=length*TIME_STEP;
	printf("Trajectory length:%d path_length:%d totalLength:%f travel_time:%f\n",
			 candidate.length,candidate.path_length,candidate.totalLength,runtime);
#ifdef PRINT_PATH
	for (int i = 0; i < length; i++) {
	    Segment s = trajectory[i];
	    printf("Time Step: %f\n", s.dt);
	    printf("Coords: (%f, %f)\n", s.x, s.y);
	    printf("Position (Distance): %f\n", s.position);
	    printf("Velocity: %f\n", s.velocity);
	    printf("Acceleration: %f\n", s.acceleration);
	    printf("Jerk (Acceleration per Second): %f\n", s.jerk);
	    printf("Heading (radians): %f\n", s.heading);
	}
#endif
#ifdef PLOT_PATH
	double t;
	for (int i = 0; i < length; i++) {
	    Segment s = trajectory[i];
	    double h=r2d(s.heading);
	    h=h>180?h-360:h;
	    printf("%f %f %f %f %f %f \n", t, s.x, s.y,s.velocity, s.acceleration,h);
	    t+=s.dt;
	}
#endif

    // Generate the Left and Right trajectories of the wheel-base using the
    // originally generated trajectory
	ModifyTank();
    std::cout << "Path generated length="<<length<< std::endl;
#ifdef PLOT_TRAJECTORY
    double tm=0;
	for (int i = 0; i < length; i++) {
		Segment l = leftTrajectory[i];
		Segment r = rightTrajectory[i];
	    //printf("%f %f %f %f %f\n", tm, l.position,r.position,l.velocity,r.velocity);
	    printf("%f %f %f %f %f\n", tm, l.x,l.y,r.x,r.y);

	    tm+=TIME_STEP;
	}
#endif
}

DrivePath::~DrivePath() {
	cleanup();
}

// Called just before this Command runs the first time
void DrivePath::Initialize() {
	leftFollower.finished=0;leftFollower.last_error=0;leftFollower.segment=0;
	rightFollower.finished=0;rightFollower.last_error=0;rightFollower.segment=0;
	std::cout << "DrivePath Started .."<< std::endl;
	driveTrain->Reset();
	mytimer.Start();
	mytimer.Reset();
}

// Called repeatedly when this Command is scheduled to run
void DrivePath::Execute() {
	double ld=I2M(driveTrain->GetLeftDistance());
	double rd=I2M(driveTrain->GetRightDistance());
	double l=FollowDistance(&leftFollower,leftTrajectory,ld);
	double r=FollowDistance(&rightFollower,rightTrajectory,rd);
	double lt=leftTrajectory[leftFollower.segment-1].position;
	double rt=rightTrajectory[rightFollower.segment-1].position;
#ifdef PLOT_VELOCITY
	double ltv=leftTrajectory[leftFollower.segment-1].velocity;
	double rtv=rightTrajectory[rightFollower.segment-1].velocity;
	double lv=I2M(driveTrain->GetLeftVelocity());
	double rv=I2M(driveTrain->GetRightVelocity());
#endif
	double rerr=rt-rd;
	double lerr=lt-ld;
	double gh = driveTrain->GetHeading() ;     // Assuming gyro angle is given in degrees
	double th = r2d(leftFollower.heading);
	th=th>180?360-th:th;
	double herr = th - gh;    // Make sure to bound this from -180 to 180, otherwise you will get super large values
#ifdef USE_GYRO
	double turn = GFACT * (-1.0/180.0) * herr;
#else
	double turn=0;
#endif

#ifdef DEBUG_COMMAND
#ifdef PLOT_VELOCITY
    printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", mytimer.Get(), lv, ltv, rv, rtv, gh,th,ltv-lv,rtv-rv,th-gh,l+turn,r-turn);
#else
    printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", mytimer.Get(), ld, lt, rd, rt, gh,th,lerr,rerr,herr,l+turn,r-turn);
#endif
#endif
	driveTrain->TankDrive(l+turn,r-turn);
}

#define MAX_ANGLE_ERROR 1   // degrees
#define MAX_POSITION_ERROR 0.5 // meters
// return true when this Command no longer needs to run execute()
bool DrivePath::IsFinished() {
	if(trajectory==NULL)
		return true;
	if(mytimer.Get()-runtime>0.5)
		return true;

	if(leftFollower.finished && rightFollower.finished){ // end of calculated path
		double ld=I2M(driveTrain->GetLeftDistance());
		double rd=I2M(driveTrain->GetRightDistance());
		double lerr=leftTrajectory[length-1].position-ld;
		double rerr=rightTrajectory[length-1].position-rd;
		if(fabs(lerr>MAX_POSITION_ERROR) ||  fabs(rerr)>MAX_POSITION_ERROR){
			return false;
		}
		return true;
	}
	return false;
}

// Called once after isFinished returns true
void DrivePath::End() {
    std::cout << "DrivePath End" << std::endl;
	mytimer.Stop();
	driveTrain->StopMotor();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DrivePath::Interrupted() {
    std::cout << "DrivePath Interrupted" << std::endl;
    End();
}

void DrivePath::cleanup() {
	if(trajectory)
		delete trajectory;
	if(leftTrajectory)
		delete leftTrajectory;
	if(rightTrajectory)
		delete rightTrajectory;
	trajectory=NULL;
	leftTrajectory=NULL;
	rightTrajectory=NULL;
}

// modified from tank.c/pathfinder_modify_tank
void DrivePath::ModifyTank() {
    double w = WHEELBASE_WIDTH / 2;
    int i;
    for (i = 0; i < length; i++) {
        Segment seg = trajectory[i];
        Segment left = seg;
        Segment right = seg;
        double cos_angle = cos(seg.heading);
        double sin_angle = sin(seg.heading);

        // note: this code can cause outside wheels to spin faster than measured max velocity
        //       (and to generate correction values outside +- 1  )
        //       work-around is to reduce max velocity input to trajectory calculation

        left.x = seg.x - (w * sin_angle);
        left.y = seg.y + (w * cos_angle);

        if (i > 0) {
            Segment last = leftTrajectory[i - 1];
            double distance = sqrt(
                (left.x - last.x) * (left.x - last.x)
                + (left.y - last.y) * (left.y - last.y)
            );
            left.position = last.position + distance;
            left.velocity = distance / seg.dt;
            left.acceleration = (left.velocity - last.velocity) / seg.dt;
            left.jerk = (left.acceleration - last.acceleration) / seg.dt;
        }
        right.x = seg.x + (w * sin_angle);
        right.y = seg.y - (w * cos_angle);
        if (i > 0) {
            Segment last = rightTrajectory[i - 1];
            double distance = sqrt(
                (right.x - last.x) * (right.x - last.x)
                + (right.y - last.y) * (right.y - last.y)
            );

            right.position = last.position + distance;
            right.velocity = distance / seg.dt;
            right.acceleration = (right.velocity - last.velocity) / seg.dt;
            right.jerk = (right.acceleration - last.acceleration) / seg.dt;
        }
        leftTrajectory[i] = left;
        rightTrajectory[i] = right;
    }
}
// modified from distance.c/pathfinder_follow_distance
double DrivePath::FollowDistance(DistanceFollower *follower, Segment *trajectory, double distance) {
    int segment = follower->segment;
    if (segment >= length)
        follower->finished = 1;
    else
		follower->finished = 0;

	Segment s = trajectory[segment];
	double error = s.position - distance;
	double calculated_value = config.kp * error +
			config.kd * ((error - follower->last_error) / s.dt) +
							  (config.kv * s.velocity + config.ka * s.acceleration);

	follower->last_error = error;
	follower->heading = s.heading;
	follower->output = calculated_value;
	follower->segment = follower->segment>=length-1?length-1:follower->segment + 1;
	return calculated_value;
}

