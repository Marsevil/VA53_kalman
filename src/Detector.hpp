/*
 * Detector.hpp
 *
 *  Created on: 18 nov. 2022
 *      Author: marsevil
 */

#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include "Utils.hpp"
#include "IDetector.hpp"
#include <opencv2/core/mat.hpp>

class DetectorDebugger;

class Detector : public IDetector {
private: // Variables
	static const uint8_t DEFAULT_THRESH_VALUE;
	static const float DEFAULT_MIN_AREA;
	/**
	 * Specifies the minimum area of the tracked object
	 */
	float minObjectArea;

	/**
	 * Params of the thresholding
	 */
	utils::ThresholdParams thresholdParams;

private: // Functions
	/**
	 * Prepare the image for the detection. Process thresholding & opening
	 * @param ioFrame a gray image.
	 */
	void preProcess(cv::Mat_<uint8_t>& ioFrame) const;
public:
	Detector();
	virtual ~Detector() = default;

	// Getters & Setters
	inline utils::ThresholdParams const& getThresholdParams() const { return thresholdParams; }

	// Functions
	/**
	 * Detect the object in the frame.
	 * @param frame gray image.
	 * @return image coordinate of the object.
	 */
	std::optional<cv::Point2f> detectFromGray(cv::Mat_<uint8_t> frame) const;

	virtual std::optional<cv::Point2f> detect(cv::Mat_<cv::Vec3b> const& frame);

	friend DetectorDebugger;
};

#endif /* DETECTOR_HPP_ */
