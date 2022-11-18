/*
 * DetectorDebugger.cpp
 *
 *  Created on: 25 nov. 2022
 *      Author: marsevil
 */

#include "DetectorDebugger.hpp"

#include <opencv2/opencv.hpp>

const std::string DetectorDebugger::TRACKBAR_THRESH_VALUE = "Threshold value";
const int DetectorDebugger::MAX_THRESH_VALUE = 255;
const std::string DetectorDebugger::TRACKBAR_MIN_AREA_VALUE = "Min area value";
const int DetectorDebugger::MAX_AREA_VALUE = 1500;

const int DetectorDebugger::NO_POINT_CIRCLE_SIZE = 100;
const cv::Vec3b DetectorDebugger::NO_POINT_CIRCLE_COLOR = {0, 0, 255};
const int DetectorDebugger::POINT_CIRCLE_SIZE = 20;
const cv::Vec3b DetectorDebugger::POINT_CIRCLE_COLOR = {255, 0, 0};

/*void DetectorDebugger::trackbarCallback(int trackbarValue, void* userData) {
	int& value = *static_cast<int*>(userData);
	value = trackbarValue;
}*/

DetectorDebugger::DetectorDebugger() :
	Detector(),
	_debugRenderWindowName("Detector(" + std::to_string(reinterpret_cast<size_t>(this)) + ") pre-process"),
	_debugVariablesWindowName("Detector(" + std::to_string(reinterpret_cast<size_t>(this)) + "pre-process variable")
{
	// Declare windows
	cv::namedWindow(_debugRenderWindowName, cv::WINDOW_NORMAL);
	cv::namedWindow(_debugVariablesWindowName, cv::WINDOW_AUTOSIZE);

	// Create trackbar to configure threshold value
	cv::createTrackbar(
			TRACKBAR_THRESH_VALUE,
			_debugVariablesWindowName,
			nullptr,
			MAX_THRESH_VALUE,
			trackbarCallback<int>,
			&this->thresholdParams.threshValue);
	cv::setTrackbarPos(
			TRACKBAR_THRESH_VALUE,
			_debugVariablesWindowName,
			this->thresholdParams.threshValue);

	cv::createTrackbar(
			TRACKBAR_MIN_AREA_VALUE,
			_debugVariablesWindowName,
			nullptr,
			MAX_AREA_VALUE,
			trackbarCallback<float>,
			&this->minObjectArea);
	cv::setTrackbarPos(
			TRACKBAR_MIN_AREA_VALUE,
			_debugVariablesWindowName,
			this->minObjectArea);
}

std::optional<cv::Point2f> DetectorDebugger::detect(cv::Mat_<cv::Vec3b> const& frame) {
	cv::Mat_<uint8_t> grayFrame;

	cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

	std::optional<cv::Point2f> detectedPoint = detectFromGray(grayFrame);

	// Draw for debugging
	cv::Mat_<cv::Vec3b> outputImage;
	cv::cvtColor(grayFrame, outputImage, cv::COLOR_GRAY2BGR);
	if (detectedPoint) {
		cv::circle(outputImage, *detectedPoint, POINT_CIRCLE_SIZE, POINT_CIRCLE_COLOR, -1);
	} else {
		cv::circle(
				outputImage,
				{outputImage.cols/2, outputImage.rows/2},
				NO_POINT_CIRCLE_SIZE,
				NO_POINT_CIRCLE_COLOR,
				-1);
	}
	cv::imshow(_debugRenderWindowName, outputImage);

	return detectedPoint;
}
