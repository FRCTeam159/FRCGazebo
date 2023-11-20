
#include "Subsystems/VisionProcess.h"
#include "RobotMap.h"

using namespace frc;

llvm::ArrayRef<double>  VisionProcess::hsvThresholdHue = {61.6, 102.75};
llvm::ArrayRef<double>  VisionProcess::hsvThresholdSaturation = {31.2, 176.9};
llvm::ArrayRef<double>  VisionProcess::hsvThresholdValue = {122.46, 255.0};
cv::Mat VisionProcess::thresholdImage;
std::vector<cv::Rect> VisionProcess::returnRectangles;
std::shared_ptr<NetworkTable> VisionProcess::table;


VisionProcess::VisionProcess() : Subsystem("VisionProcess"){

}

void VisionProcess::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//SetDefaultCommand(new VisionTest());
}

void VisionProcess::Init() {
	table = NetworkTable::GetTable("GRIP/targets");
	SmartDashboard::PutBoolean("ShowTargets", false);
	std::thread visionThread(VisionThread);
	visionThread.detach();
}

void VisionProcess::VisionThread(){

	cv::Mat image;
	cs::CvSource outputStream;
	cv::VideoCapture vcap;

    const std::string videoStreamAddress = "http://localhost:5002/?action=stream";
    //open the video stream and make sure it's opened
    if(!vcap.open(videoStreamAddress))
        std::cout << "Error opening video stream "<<videoStreamAddress << std::endl;
    else
        std::cout << "Video Stream captured "<<videoStreamAddress << std::endl;

	outputStream=CameraServer::GetInstance()->PutVideo("Rectangle", 320, 240);
	while (true) {//
		if(!vcap.read(image))
			 std::cout << "No frame" << std::endl;
		else{
			process(image);
			std::vector<cv::Rect> rects=*getRectangles();
			bool showrects=SmartDashboard::GetBoolean("ShowTargets", false);
			if(showrects){
				for (unsigned int i = 0; i < rects.size(); i++) {
					cv::Rect r= rects [i];
					rectangle(image, r.tl(), r.br(), cv::Scalar(0, 255, 255), 1);
				}
			}
			outputStream.PutFrame(image);
			//if(rects.size()>0)
				publishTables(rects);
		}
		//std::this_thread::sleep_for(std::chrono::duration<double>(0.05));
	}
}

void VisionProcess::publishTables(std::vector<cv::Rect> &rects){
	std::vector<double> arr1; // area
	std::vector<double> arr2; // centerX
	std::vector<double> arr3; // centerY
	std::vector<double> arr4; // width
	std::vector<double> arr5; // height
//	arr1.clear();
//	arr2.clear();
//	arr3.clear();
//	arr4.clear();
//	arr5.clear();
	for (unsigned int i = 0; i < rects.size(); i++) {
		cv::Rect r= rects[i];
		arr1.push_back(r.area());
		arr2.push_back(r.x+0.5*r.width);
		arr3.push_back(r.y+0.5*r.height);
		arr4.push_back(r.width);
		arr5.push_back(r.height);
	};
    table->PutNumberArray("area",arr1);
    table->PutNumberArray("centerX",arr2);
    table->PutNumberArray("centerY",arr3);
    table->PutNumberArray("width",arr4);
    table->PutNumberArray("height",arr5);
}

void VisionProcess::process(cv::Mat source){
	//Step Blur0:
	//input
	BlurType blurType = BlurType::MEDIAN;
	double blurRadius = 1.0;  // default Double
	cv::Mat image;
	source.copyTo(image);

	blur(image, blurType, blurRadius);

	hsvThreshold(image, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue);
	image.copyTo(thresholdImage);
	//Step Find_Contours0:
	//input
	bool findContoursExternalOnly = true;  // default Boolean
	std::vector<std::vector<cv::Point> > contours;

	findContours(image, findContoursExternalOnly, contours);
	//Step Convex_Hulls0:
	//input

	std::vector<std::vector<cv::Point> > hulls;
	convexHulls(contours, hulls);
	//Step Filter_Contours0:
	//input
	double filterContoursMinArea = 50.0;  // default Double
	double filterContoursMinPerimeter = 0.0;  // default Double
	double filterContoursMinWidth = 5.0;  // default Double
	double filterContoursMaxWidth = 1000.0;  // default Double
	double filterContoursMinHeight = 5.0;  // default Double
	double filterContoursMaxHeight = 1000.0;  // default Double
	double filterContoursSolidity[] = {0, 100};
	double filterContoursMaxVertices = 20.0;  // default Double
	double filterContoursMinVertices = 4.0;  // default Double
	double filterContoursMinRatio = 1.0;  // default Double
	double filterContoursMaxRatio = 2.0;  // default Double
	std::vector<std::vector<cv::Point> > filtered_hulls;
	filterContours(hulls, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filtered_hulls);

	findRectangles(filtered_hulls, returnRectangles);
}
/**
 * Softens an image using one of several filters.
 *
 * @param input The image on which to perform the blur.
 * @param type The blurType to perform.
 * @param doubleRadius The radius for the blur.
 * @param output The image in which to store the output.
 */
void VisionProcess::blur(cv::Mat &input, BlurType &type, double doubleRadius) {
	cv::Mat output;
	int radius = (int)(doubleRadius + 0.5);
	int kernelSize;
	switch(type) {
		case BOX:
			kernelSize = 2 * radius + 1;
			cv::blur(input,output,cv::Size(kernelSize, kernelSize));
			break;
		case GAUSSIAN:
			kernelSize = 6 * radius + 1;
			cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
			break;
		case MEDIAN:
			kernelSize = 2 * radius + 1;
			cv::medianBlur(input, output, kernelSize);
			break;
		case BILATERAL:
			cv::bilateralFilter(input, output, -1, radius, radius);
			break;
    }
	output.copyTo(input);
}
/**
 * Segment an image based on hue, saturation, and value ranges.
 *
 * @param input The image on which to perform the HSL threshold.
 * @param hue The min and max hue.
 * @param sat The min and max saturation.
 * @param val The min and max value.
 * @param output The image in which to store the output.
 */
void VisionProcess::hsvThreshold(cv::Mat &input, llvm::ArrayRef<double> hue, llvm::ArrayRef<double> sat,llvm::ArrayRef<double> val) {
	cv::Mat output;
	cv::cvtColor(input, output, cv::COLOR_BGR2HSV);
	cv::inRange(output,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), output);
	output.copyTo(input);
}
/**
 * Finds contours in an image.
 *
 * @param input The image to find contours in.
 * @param externalOnly if only external contours are to be found.
 * @param contours vector of contours to put contours in.
 */
void VisionProcess::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
	std::vector<cv::Vec4i> hierarchy;
	contours.clear();
	int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
	int method = cv::CHAIN_APPROX_SIMPLE;
	cv::findContours(input, contours, hierarchy, mode, method);
}
/**
 * Compute the convex hulls of contours.
 *
 * @param inputContours The contours on which to perform the operation.
 * @param outputContours The contours where the output will be stored.
 */
void VisionProcess::convexHulls(std::vector<std::vector<cv::Point> > &inputContours, std::vector<std::vector<cv::Point> > &outputContours) {
	std::vector<std::vector<cv::Point> > hull (inputContours.size());
	outputContours.clear();
	for (size_t i = 0; i < inputContours.size(); i++ ) {
		cv::convexHull(cv::Mat((inputContours)[i]), hull[i], false);
	}
	outputContours = hull;
}
/**
 * Filters through contours.
 * @param inputContours is the input vector of contours.
 * @param minArea is the minimum area of a contour that will be kept.
 * @param minPerimeter is the minimum perimeter of a contour that will be kept.
 * @param minWidth minimum width of a contour.
 * @param maxWidth maximum width.
 * @param minHeight minimum height.
 * @param maxHeight  maximimum height.
 * @param solidity the minimum and maximum solidity of a contour.
 * @param minVertexCount minimum vertex Count of the contours.
 * @param maxVertexCount maximum vertex Count.
 * @param minRatio minimum ratio of width to height.
 * @param maxRatio maximum ratio of width to height.
 * @param output vector of filtered contours.
 */
void VisionProcess::filterContours(std::vector<std::vector<cv::Point> > &inputContours, double minArea, double minPerimeter, double minWidth, double maxWidth, double minHeight, double maxHeight, double solidity[], double maxVertexCount, double minVertexCount, double minRatio, double maxRatio, std::vector<std::vector<cv::Point> > &output) {
	std::vector<cv::Point> hull;
	output.clear();
	for (std::vector<cv::Point> contour: inputContours) {
		cv::Rect bb = boundingRect(contour);
		if (bb.width < minWidth || bb.width > maxWidth) continue;
		if (bb.height < minHeight || bb.height > maxHeight) continue;
		double area = cv::contourArea(contour);
		if (area < minArea) continue;
		if (arcLength(contour, true) < minPerimeter) continue;
		double ratio = bb.width / bb.height;
		if (ratio < minRatio || ratio > maxRatio) continue;
		output.push_back(contour);
	}
}
void VisionProcess::findRectangles(std::vector<std::vector<cv::Point> > &inputContours, std::vector<cv::Rect> &output){
	output.clear();
	for (std::vector<cv::Point> contour: inputContours) {
		cv::Rect bb = boundingRect(contour);
		output.push_back(bb);
	}
}
