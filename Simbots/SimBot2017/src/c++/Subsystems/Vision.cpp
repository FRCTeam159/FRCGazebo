#include <Commands/VisionUpdate.h>
#include "Vision.h"
#include "RobotMap.h"
#include "Subsystems/GripPipeline.h"
#include <CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "llvm/ArrayRef.h"
#include "llvm/StringRef.h"

#define SIMULATION

#define RPD(x) (x)*2*M_PI/360
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define DFOV 60   // diagonal spec
#define HFOV 49.6 // measured 44.28
#define VFOV 30.2 //

#define ASPECT_RATIO (4.0/3.0)

#define HOFFSET 8.0 // camera offset from robot center

using namespace frc;

static clock_t start = clock();
static int frame_count=0;
static int last_frame=0;

static double delt=0;
double sumdt=0;

Vision::Vision() : Subsystem("VisionSubsystem") {
	SetCameraInfo(IMAGE_WIDTH,IMAGE_HEIGHT,HFOV,HOFFSET);
}

void Vision::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	SetDefaultCommand(new VisionUpdate());
}

void Vision::Init() {
	table=NetworkTable::GetTable("datatable");
#ifdef APP_TEST
	frc::SmartDashboard::PutBoolean("showColorThreshold", false);
	frc::SmartDashboard::PutNumber("Rectangles", 0);
	frc::SmartDashboard::PutBoolean("showGoodRects", true);
	//std::system("../ImageProc/Ubuntu/ImageProc &");
#else
	std::thread visionThread(VisionThread);
	visionThread.detach();
#endif
	frc::SmartDashboard::PutNumber("Distance", 0);
	frc::SmartDashboard::PutNumber("HOffset", HOFFSET);
	frc::SmartDashboard::PutNumber("HorizontalAngle", 0);
#ifndef SIMULATION
	cs::UsbCamera camera1 = CameraServer::GetInstance()->StartAutomaticCapture("Logitech",0);
	cs::UsbCamera camera2 = CameraServer::GetInstance()->StartAutomaticCapture("DriverCam",1);

	// Set the resolution
	camera1.SetResolution(320, 240);
	camera2.SetResolution(320, 240);
	//camera.SetFPS(1);
	camera1.SetFPS(10);
#endif
	//ssh pi@rapberrypi.local -c "/home/pi/vision/start_grip.sh
	frame_count=0;
	last_frame=0;
	delt=0;
	sumdt=0;
}

void Vision::Process() {
    GetTargetInfo();
    PublishTargetInfo();
}

#ifndef APP_TEST
void Vision::VisionThread(){
	std::shared_ptr<NetworkTable> table2=NetworkTable::GetTable("datatable");
#ifdef SIMULATION
	cv::VideoCapture vcap;

    const std::string videoStreamAddress = "http://localhost:5002/?action=stream";
    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress))
        std::cout << "Error opening video stream "<<videoStreamAddress << std::endl;
    else
        std::cout << "Video Stream captured "<<videoStreamAddress << std::endl;
#else
	// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo("Logitech");
#endif
	// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Rectangle", 320, 240);

	cv::Mat mat;

	cv::Point tl=cv::Point(10, 10);
	cv::Point br=cv::Point(20, 20);

	llvm::ArrayRef<double>  hsvThresholdHue = {70, 110};
	llvm::ArrayRef<double>  hsvThresholdSaturation = {100, 255};
	llvm::ArrayRef<double>  hsvThresholdValue = {100, 255};

	frc::SmartDashboard::PutNumber("HueMax", hsvThresholdHue[1]);
	frc::SmartDashboard::PutNumber("HueMin", hsvThresholdHue[0]);
	frc::SmartDashboard::PutNumber("SaturationMax", hsvThresholdSaturation[1]);
	frc::SmartDashboard::PutNumber("SaturationMin", hsvThresholdSaturation[0]);
	frc::SmartDashboard::PutNumber("ValueMax", hsvThresholdValue[1]);
	frc::SmartDashboard::PutNumber("ValueMin", hsvThresholdValue[0]);
	frc::SmartDashboard::PutBoolean("showColorThreshold", false);
	frc::SmartDashboard::PutNumber("Rectangles", 0);
	frc::SmartDashboard::PutBoolean("showGoodRects", true);

	GripPipeline gp;
    auto start = std::chrono::high_resolution_clock::now();


	while(true){
#ifdef SIMULATION
		if(!vcap.read(mat)) {
#else
		if (cvSink.GrabFrame(mat) == 0) {
#endif
			//outputStream.NotifyError(cvSink.GetError());
			continue;
		}
		bool showColorThreshold = frc::SmartDashboard::GetBoolean("showColorThreshold", false);

		hsvThresholdHue={SmartDashboard::GetNumber("HueMin",70),SmartDashboard::GetNumber("HueMax", 100)};
		hsvThresholdValue={SmartDashboard::GetNumber("ValueMin", 100),SmartDashboard::GetNumber("ValueMax",255)};
		hsvThresholdSaturation={SmartDashboard::GetNumber("SaturationMin", 100),SmartDashboard::GetNumber("SaturationMax",255)};

		gp.setHSVThresholdHue(hsvThresholdHue);
		gp.setHSVThresholdValue(hsvThresholdValue);
		gp.setHSVThresholdSaturation(hsvThresholdSaturation);

		gp.process(mat);

		if(showColorThreshold){
			cv::Mat* mat2=gp.getColorThresholdOutput();
			mat2->copyTo(mat);
		}

		int minx = 1000, maxx = 0, miny = 1000, maxy = 0;
		std::vector<cv::Rect> rects= *gp.getRectangles();

		bool showGoodRects = frc::SmartDashboard::GetBoolean("showGoodRects", true);

		std::vector<cv::Rect> rectsPointer=rects;
		if (showGoodRects){
			rectsPointer=GoodRects(rects);
		}

		frc::SmartDashboard::PutNumber("Rectangles",rectsPointer.size());
		int n=rectsPointer.size();
		table2->PutNumber("NumRects", n);
		for (unsigned int i = 0; i < rectsPointer.size(); i++) {
			cv::Rect r= rectsPointer[i];
			cv::Point p= r.tl();
			minx = p.x < minx ? p.x : minx;
			miny = p.y < miny ? p.y : miny;
			p= r.br();
			maxy = p.y > maxy ? p.y : maxy;
			maxx = p.x > maxx ? p.x : maxx;
			rectangle(mat, r.tl(), r.br(), cv::Scalar(0, 255, 255), 1);
		}

		tl=cv::Point(minx, miny);
		br=cv::Point(maxx, maxy);

		rectangle(mat, cv::Point(minx, miny), cv::Point(maxx, maxy), cv::Scalar(255, 255, 255), 1);

		outputStream.PutFrame(mat);
		table2->PutNumber("TopLeftX", tl.x);
		table2->PutNumber("TopLeftY", tl.y);
		table2->PutNumber("BotRightX", br.x);
		table2->PutNumber("BotRightY", br.y);
//#define DEBUG
#ifdef DEBUG
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        time_t currentTime;
        struct tm *localTime;
        time( &currentTime );                   // Get the current time
        localTime = localtime( &currentTime );  // Convert the current time to the local time
        std::cout<<"calc time :"<<localTime->tm_min<<":"<<localTime->tm_sec<<" dt:"<<elapsed.count()<< " ms num targets:"<<n<<std::endl;
        start=end;
#endif
	}
}

std::vector<cv::Rect> Vision::GoodRects(std::vector<cv::Rect> rects) {
	if(rects.size()<2){
		return rects;
	}
	std::vector<cv::Rect> goodrects;
	int gfw=5;
	double maxarea=0;
	cv::Rect Rect2=rects[0];

	for (unsigned int i = 0; i < rects.size(); i++) {
		int score = 0;
		cv::Rect Rect1=rects[i];
		int cx1=0.5*(Rect1.tl().x+Rect1.br().x);
		int cy1=0.5*(Rect1.tl().y+Rect1.br().y);
		double w1=rects[i].width;
		double area=Rect1.area();

		if(i>0 && area>maxarea){
			Rect2=Rect1;
			maxarea=area;
		}
		for (unsigned int j = 0; j < rects.size(); j++) {
			if (i==j)
				continue;
			cv::Rect Rect2=rects[j];
			int cx2=0.5*(Rect2.tl().x+Rect2.br().x);
			int cy2=0.5*(Rect2.tl().y+Rect2.br().y);
			int dx=cx1-cx2;
			int dy=cy1-cy2;
			int d=sqrt(dx*dx+dy*dy);
			double r1=d/w1;
			if (r1<gfw )
				score++;
		}
		if (score>0)
			goodrects.push_back(Rect1);
	}
	if(goodrects.size()==0)
		goodrects.push_back(Rect2);
	return goodrects;
}
#endif
void Vision::SetCameraInfo(int width, int height, double fov, double hoff) {
	cameraInfo.screenWidth = width;
	cameraInfo.screenHeight = height;
	cameraInfo.fov = fov;
	cameraInfo.fovFactor = cameraInfo.screenWidth/(2*tan(RPD(fov/2.0)));
	cameraInfo.HorizontalOffset=hoff;
	cout<<"fovFactor: "<<cameraInfo.fovFactor<<endl;
}

void Vision::CalcTargetInfo(int n,cv::Point top, cv::Point bottom) {
	targetInfo.numrects=n;
	//if(n>0){
		targetInfo.Height=bottom.y-top.y; // screen y is inverted
		targetInfo.Width=bottom.x-top.x;
		targetInfo.Center.x=0.5*(bottom.x+top.x);
		targetInfo.Center.y=0.5*(bottom.y+top.y);
		targetInfo.HorizontalOffset=targetInfo.Center.x-cameraInfo.screenWidth/2;
		targetInfo.ActualHeight=5.0;
		targetInfo.ActualWidth=(n==1?2.0:10.25);	//inches
		targetInfo.Distance=cameraInfo.fovFactor*targetInfo.ActualHeight/targetInfo.Height;
		GetTargetAngle();
	//}
}

double Vision::GetTargetAngle() {
	cameraInfo.HorizontalOffset=frc::SmartDashboard::GetNumber("HOffset", HOFFSET);
	double xoffset=0;
	if(targetInfo.numrects==1){
		if(targetInfo.HorizontalOffset<0)
			xoffset-=2*targetInfo.Width;
		else
			xoffset+=2*targetInfo.Width;
	}
    double cam_adjust=cameraInfo.fovFactor*cameraInfo.HorizontalOffset/targetInfo.Distance;
    double p=targetInfo.Center.x+xoffset+cam_adjust-0.5*cameraInfo.screenWidth;
    targetInfo.HorizontalAngle=p*cameraInfo.fov/cameraInfo.screenWidth;
	return targetInfo.HorizontalAngle;
}
double Vision::GetTargetDistance() {
	return targetInfo.Distance;
}
int Vision::GetNumTargets() {
	return targetInfo.numrects;
}

void Vision::GetTargetInfo() {
	cv::Point top;
	cv::Point bot;
	int n=table->GetNumber("NumRects", 0);
	top.x=table->GetNumber("TopLeftX", 10);
	top.y=table->GetNumber("TopLeftY", 10);
	bot.x=table->GetNumber("BotRightX",20);
	bot.y=table->GetNumber("BotRightY",20);
//#define DEBUG
#ifdef DEBUG
    time_t currentTime;
    struct tm *localTime;
    time( &currentTime );                   // Get the current time
    localTime = localtime( &currentTime );  // Convert the current time to the local time
    std::cout<<"time :"<<localTime->tm_min<<":"<<localTime->tm_sec<<" num targets:"<<n<<std::endl;
#endif

	CalcTargetInfo(n,top,bot);
#ifdef APP_TEST

	bool showColorThreshold=SmartDashboard::GetBoolean("showColorThreshold", false);
	bool showGoodRects=SmartDashboard::GetBoolean("showGoodRects", true);
	table->PutBoolean("showGoodRects",showGoodRects);
	table->PutBoolean("showColorThreshold",showColorThreshold);
    frc::SmartDashboard::PutNumber("Rectangles", n);

	int frame=table->GetNumber("Frame", 0);
    frc::SmartDashboard::PutNumber("Frame",frame);
    frc::SmartDashboard::PutNumber("FPS",table->GetNumber("FPS", 0));

    frc::SmartDashboard::PutNumber("TargetType",table->GetNumber("TargetType", 0));
    frc::SmartDashboard::PutNumber("TargetScore",table->GetNumber("TargetScore", 0));


    last_frame=frame;
#endif

}

void Vision::PublishTargetInfo() {
	frc::SmartDashboard::PutNumber("Distance", targetInfo.Distance);
	frc::SmartDashboard::PutNumber("HorizontalOffset", targetInfo.HorizontalOffset);
	frc::SmartDashboard::PutNumber("HorizontalAngle", targetInfo.HorizontalAngle);
	frc::SmartDashboard::PutNumber("TargetHeight", targetInfo.Height);
	frc::SmartDashboard::PutNumber("TargetWidth", targetInfo.Width);
	frc::SmartDashboard::PutNumber("TargetX", targetInfo.Center.x);
	frc::SmartDashboard::PutNumber("TargetY", targetInfo.Center.y);
}

