/*
 * IDetector.hpp
 *
 *  Created on: 25 nov. 2022
 *      Author: marsevil
 */

#ifndef SRC_IDETECTOR_HPP_
#define SRC_IDETECTOR_HPP_

#include <opencv2/core.hpp>
#include <optional>

class IDetector {
public:
	IDetector() = default;
	virtual ~IDetector() = default;

	/**
	 * Return the position in the frame of the detected object.
	 * @param frame colored image (BGR)
	 * @return Position of the detected object in the image
	 */
	virtual std::optional<cv::Point2f> detect(cv::Mat_<cv::Vec3b> const& frame) = 0;
};


#endif /* SRC_IDETECTOR_HPP_ */
