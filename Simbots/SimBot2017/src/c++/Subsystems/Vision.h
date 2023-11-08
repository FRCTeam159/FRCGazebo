#ifndef Vision_H
#define Vision_H
#include <Commands/Subsystem.h>
#include <opencv2/core/types.hpp>

//using namespace grip;

class Vision : public Subsystem {
public:
	struct CameraInfo {
		int screenWidth;
		int screenHeight;
		double fov;
		double fovFactor;
		double HorizontalOffset;
	};

	CameraInfo cameraInfo;

	struct TargetInfo {
		cv::Point Center;
		double Distance;
		double HorizontalOffset;
		double HorizontalAngle;
		double Height;
		double Width;
		double ActualWidth;
		double ActualHeight;
		int numrects;
	};
	TargetInfo targetInfo;

private:

	std::shared_ptr<NetworkTable> table;

#ifndef APP_TEST
	static std::vector<cv::Rect> GoodRects(std::vector<cv::Rect> r);
#endif
	static void VisionThread();

public:
	Vision();
	void InitDefaultCommand();
	void Process();
	void Init();
	double GetTargetDistance();
	double GetTargetAngle();
	int GetNumTargets();
	void SetCameraInfo(int width, int height, double fov, double hoff);
	void CalcTargetInfo(int n,cv::Point top, cv::Point bottom);
	void GetTargetInfo();
	void PublishTargetInfo();
};

#endif  // Vision_H
