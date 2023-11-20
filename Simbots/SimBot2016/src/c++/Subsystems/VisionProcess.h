#ifndef VisionProcess_H
#define VisionProcess_H

#include <Commands/Subsystem.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <CameraServer.h>
#include "llvm/StringRef.h"
#include "llvm/ArrayRef.h"

enum BlurType {
	BOX, GAUSSIAN, MEDIAN, BILATERAL
};

class VisionProcess : public Subsystem {
private:
    static std::shared_ptr<NetworkTable> table;

	static void VisionThread();
	static llvm::ArrayRef<double>  hsvThresholdHue;
	static llvm::ArrayRef<double>  hsvThresholdSaturation;
	static llvm::ArrayRef<double>  hsvThresholdValue;
	static cv::Mat thresholdImage;
	static std::vector<cv::Rect> returnRectangles;

	static void process(cv::Mat source0);
	static void blur(cv::Mat &, BlurType &, double);
	static void hsvThreshold(cv::Mat &input, llvm::ArrayRef<double>, llvm::ArrayRef<double>, llvm::ArrayRef<double>);
	static void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
	static void convexHulls(std::vector<std::vector<cv::Point> > &, std::vector<std::vector<cv::Point> > &);
	static void filterContours(std::vector<std::vector<cv::Point> > &, double , double , double , double , double , double , double [], double , double , double , double , std::vector<std::vector<cv::Point> > &);
	static void findRectangles(std::vector<std::vector<cv::Point> > &inputContours, std::vector<cv::Rect> &output);
	static void publishTables(std::vector<cv::Rect> &rects);
public:
	VisionProcess();
	void InitDefaultCommand();
	void Init();
	static cv::Mat* getColorThresholdImage() { return &thresholdImage;}
	static std::vector<cv::Rect>*getRectangles(){ return &returnRectangles;}

};

#endif  // VisionProcess_H
