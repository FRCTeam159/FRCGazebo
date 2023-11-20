#ifndef APP_TEST

#include "GripPipeline.h"
/**
 * Initializes a GripPipeline.
 */

GripPipeline::GripPipeline() {
}
/**
 * Runs an iteration of the Pipeline and updates outputs.
 *
 * Sources need to be set before calling this method.
 *
 */
//#define BLUR
#define FILTERCONTOURS

void GripPipeline::process(cv::Mat source0) {
	//Step Blur0:
	//input
#ifdef BLUR
	BlurType blurType = BlurType::MEDIAN;
	double blurRadius = 4.5;  // default Double
	blur(source0, blurType, blurRadius, this->blurOutput);
	cv::Mat colorThresholdInput = blurOutput;
#else
	cv::Mat colorThresholdInput = source0;
#endif

	hsvThreshold(colorThresholdInput, hsvThresholdHue, hsvThresholdSaturation,
			hsvThresholdValue, this->colorThresholdOutput);
	//Step Find_Contours0:rgbThresholdGreen
	//input
	cv::Mat findContoursInput;
	colorThresholdOutput.copyTo(findContoursInput);
	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly,
			this->findContoursOutput);
	//Step Convex_Hulls0:
	//input
	std::vector<std::vector<cv::Point> > convexHullsContours =
			findContoursOutput;
	convexHulls(convexHullsContours, this->convexHullsOutput);
	//Step Filter_Contours0:
	//input
#ifdef FILTERCONTOURS
	std::vector<std::vector<cv::Point> > filterContoursContours =
			convexHullsOutput;
	double filterContoursMinArea = 200.0;  // default Double
	double filterContoursMinPerimeter = 0;  // default Double
	double filterContoursMinWidth = 5.0;  // default Double
	double filterContoursMaxWidth = 100;  // default Double
	double filterContoursMinHeight = 5.0;  // default Double
	double filterContoursMaxHeight = 1000;  // default Double
	double filterContoursSolidity[] = { 0, 100 };
	double filterContoursMaxVertices = 100;  // default Double
	double filterContoursMinVertices = 3;  // default Double
	double filterContoursMinRatio = 0;  // default Double
	double filterContoursMaxRatio = 1.2;  // default Double
	filterContours(filterContoursContours, filterContoursMinArea,
			filterContoursMinPerimeter, filterContoursMinWidth,
			filterContoursMaxWidth, filterContoursMinHeight,
			filterContoursMaxHeight, filterContoursSolidity,
			filterContoursMaxVertices, filterContoursMinVertices,
			filterContoursMinRatio, filterContoursMaxRatio,
			this->filterContoursOutput);
	returnVector = this->filterContoursOutput;
#else
	returnVector=this->convexHullsOutput;
#endif
	findRectangles(returnVector, this->returnRectangles);

}

/**
 * This method is a generated setter for source0.
 * @param source the Mat to set
 */
void GripPipeline::setsource0(cv::Mat &source0) {
	source0.copyTo(this->source0);
}
/**
 * This method is a generated getter for the output of a Resize_Image.
 * @return Mat output from Resize_Image.
 */
cv::Mat* GripPipeline::getresizeImageOutput() {
	return &(this->resizeImageOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* GripPipeline::getblurOutput() {
	return &(this->blurOutput);
}
/**
 * This method is a generated getter for the output of a color_Threshold.
 * @return Mat output from color_Threshold.
 */
cv::Mat* GripPipeline::getColorThresholdOutput() {
	return &(this->colorThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::getfindContoursOutput() {
	return &(this->findContoursOutput);
}
/**
 * This method is a generated getter for the output of a Convex_Hulls.
 * @return ContoursReport output from Convex_Hulls.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::getconvexHullsOutput() {
	return &(this->convexHullsOutput);
}
/**
 * This method is a generated getter for the output of a Filter_Contours.
 * @return ContoursReport output from Filter_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::getfilterContoursOutput() {
	return &(this->filterContoursOutput);
}
/**
 * Scales and image to an exact size.
 *
 * @param input The image on which to perform the Resize.
 * @param width The width of the output in pixels.
 * @param height The height of the output in pixels.
 * @param interpolation The type of interpolation.
 * @param output The image in which to store the output.
 */
void GripPipeline::resizeImage(cv::Mat &input, double width, double height,
		int interpolation, cv::Mat &output) {
	cv::resize(input, output, cv::Size(width, height), 0.0, 0.0, interpolation);
}

/**
 * Softens an image using one of several filters.
 *
 * @param input The image on which to perform the blur.
 * @param type The blurType to perform.
 * @param doubleRadius The radius for the blur.
 * @param output The image in which to store the output.
 */
void GripPipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius,
		cv::Mat &output) {
	int radius = (int) (doubleRadius + 0.5);
	int kernelSize;
	switch (type) {
	case BOX:
		kernelSize = 2 * radius + 1;
		cv::blur(input, output, cv::Size(kernelSize, kernelSize));
		break;
	case GAUSSIAN:
		kernelSize = 6 * radius + 1;
		cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize),
				radius);
		break;
	case MEDIAN:
		kernelSize = 2 * radius + 1;
		cv::medianBlur(input, output, kernelSize);
		break;
	case BILATERAL:
		cv::bilateralFilter(input, output, -1, radius, radius);
		break;
	}
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
void GripPipeline::hsvThreshold(cv::Mat &inputCpu, llvm::ArrayRef<double> hue,
		llvm::ArrayRef<double> sat, llvm::ArrayRef<double> val, cv::Mat &out) {
#ifdef USE_GPU
    GpuMat input(inputCpu);
    GpuMat output;
    cuda::cvtColor(input, output, cv::COLOR_BGR2HSV);
#else
	cv::cvtColor(inputCpu, out, cv::COLOR_BGR2HSV);
	cv::inRange(out, cv::Scalar(hue[0], sat[0], val[0]),
			cv::Scalar(hue[1], sat[1], val[1]), out);
#endif
}
/**
 * Finds contours in an image.
 *
 * @param input The image to find contours in.
 * @param externalOnly if only external contours are to be found.
 * @param contours vector of contours to put contours in.
 */
void GripPipeline::findContours(cv::Mat &input, bool externalOnly,
		std::vector<std::vector<cv::Point> > &contours) {
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
void GripPipeline::convexHulls(
		std::vector<std::vector<cv::Point> > &inputContours,
		std::vector<std::vector<cv::Point> > &outputContours) {
	std::vector<std::vector<cv::Point> > hull(inputContours.size());
	outputContours.clear();
	for (size_t i = 0; i < inputContours.size(); i++) {
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
void GripPipeline::filterContours(
		std::vector<std::vector<cv::Point> > &inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight,
		double maxHeight, double solidity[], double maxVertexCount,
		double minVertexCount, double minRatio, double maxRatio,
		std::vector<std::vector<cv::Point> > &output) {
	std::vector<cv::Point> hull;
	output.clear();
	for (std::vector<cv::Point> contour : inputContours) {
		cv::Rect bb = boundingRect(contour);
		if (bb.width < minWidth || bb.width > maxWidth)
			continue;
		if (bb.height < minHeight || bb.height > maxHeight)
			continue;
		double area = cv::contourArea(contour);
		if (area < minArea)
			continue;
		if (arcLength(contour, true) < minPerimeter)
			continue;
		//cv::convexHull(cv::Mat(contour, true), hull);
		//double solid = 100 * area / cv::contourArea(hull);
		//if (solid < solidity[0] || solid > solidity[1]) continue;
		//if (contour.size() < minVertexCount || contour.size() > maxVertexCount)	continue;
		double ratio = bb.width / bb.height;
		if (ratio < minRatio || ratio > maxRatio)
			continue;
		output.push_back(contour);
	}
}
void GripPipeline::findRectangles(
		std::vector<std::vector<cv::Point> > &inputContours,
		std::vector<cv::Rect> &output) {
	output.clear();
	for (std::vector<cv::Point> contour : inputContours) {
		cv::Rect bb = boundingRect(contour);
		output.push_back(bb);
	}
}

std::vector<std::vector<cv::Point> >* GripPipeline::getResultVector() {
	return &(this->returnVector);
}
#endif
