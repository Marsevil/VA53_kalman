/*
 * Detector.cpp
 *
 *  Created on: 18 nov. 2022
 *      Author: marsevil
 */

#include "Detector.hpp"

#include <opencv2/opencv.hpp>

const uint8_t Detector::DEFAULT_THRESH_VALUE = 30;
const float Detector::DEFAULT_MIN_AREA = 800;

Detector::Detector() :
thresholdParams{255, DEFAULT_THRESH_VALUE},
minObjectArea(DEFAULT_MIN_AREA)
{
	// Nothing to do.
}

void Detector::preProcess(cv::Mat_<uint8_t>& ioFrame) const {
	const cv::Mat_<uint8_t> kernel = cv::getStructuringElement(cv::MORPH_RECT, {20, 10});

	// Process tresholding
	cv::threshold(
			ioFrame,
			ioFrame,
			thresholdParams.threshValue,
			thresholdParams.maxValue,
			cv::THRESH_BINARY_INV);

	// Process erode & dilate
	cv::morphologyEx(ioFrame, ioFrame, cv::MORPH_OPEN, kernel);
}

/**
 * Process the detection on the gray scale image.
 * @param frame gray frame. Consume the frame.
 * @return The position of the detected object in the frame.
 */
std::optional<cv::Point2f> Detector::detectFromGray(cv::Mat_<uint8_t> frame) const {
	// TODO: Divide this process into multiple function.
	preProcess(frame);

	// Process contours
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(frame, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

	// If there is no contours
	if (contours.empty()) return std::nullopt;

	// Filter contours by area
	// TODO: Use the area of the bounding box in order to simplify processing
	std::vector<float> areas(contours.size());
	std::transform(
			contours.begin(),
			contours.end(),
			areas.begin(),
			[](auto val) {
		return cv::contourArea(val, false);
	});
	auto biggerContoursIt = std::max_element(areas.begin(), areas.end(),
			[](float a, float b) {
		return a < b;
	});
	// Check if the found contours is wide enough
	if (*biggerContoursIt < this->minObjectArea) return std::nullopt;
	size_t biggerContoursIdx = std::distance(areas.begin(), biggerContoursIt);

	// Process bounding box
	cv::Rect boundingBox = cv::boundingRect(contours[biggerContoursIdx]);

	// Process center
	cv::Point2f center = cv::Point2f(
			boundingBox.x + boundingBox.width/2,
			boundingBox.y + boundingBox.height/2);

	return center;
}

std::optional<cv::Point2f> Detector::detect(cv::Mat_<cv::Vec3b> const& frame) {
	// Convert to gray image
	cv::Mat_<uint8_t> grayFrame;
	cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

	return detectFromGray(grayFrame);
}
