/*
 * DetectorDebugger.hpp
 *
 *  Created on: 25 nov. 2022
 *      Author: marsevil
 */

#ifndef SRC_DETECTORDEBUGGER_HPP_
#define SRC_DETECTORDEBUGGER_HPP_

#include <opencv2/core.hpp>

#include "Detector.hpp"

/**
 * Same as the Detector Class but show frame & trackbar in order to parameterize, test & debug
 */
class DetectorDebugger : public Detector {
	static const std::string TRACKBAR_THRESH_VALUE;
	static const int MAX_THRESH_VALUE;
	static const std::string TRACKBAR_MIN_AREA_VALUE;
	static const int MAX_AREA_VALUE;

	static const int NO_POINT_CIRCLE_SIZE;
	static const cv::Vec3b NO_POINT_CIRCLE_COLOR;
	static const int POINT_CIRCLE_SIZE;
	static const cv::Vec3b POINT_CIRCLE_COLOR;

	// Window names
	const std::string _debugRenderWindowName;
	const std::string _debugVariablesWindowName;

	/**
	 * Static function use to set @p userData value after callback
	 * @tparam T type to cast
	 * @param trackbarValue current value of the trackback
	 * @param userData data pass during the trackbar create
	 */
	template <typename T>
	static void trackbarCallback(int trackbarValue, void* userData) {
		T& value = *static_cast<T*>(userData);
		value = trackbarValue;
	}

public:
	DetectorDebugger();
	virtual ~DetectorDebugger() = default;

	virtual std::optional<cv::Point2f> detect(cv::Mat_<cv::Vec3b> const& frame);
};

#endif /* SRC_DETECTORDEBUGGER_HPP_ */
