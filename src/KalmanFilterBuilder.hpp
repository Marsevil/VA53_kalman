/*
 * KalmanFilterBuilder.hpp
 *
 *  Created on: 27 nov. 2022
 *      Author: marsevil
 */

#ifndef SRC_KALMANFILTERBUILDER_HPP_
#define SRC_KALMANFILTERBUILDER_HPP_

#include <opencv2/opencv.hpp>
#include <optional>

class KalmanFilterBuilder {
public: // Declare linked type and enums.
	enum KalmanModelType {
		POSITION,
		SPEED,
		ACCELERATION,
	};

private: // Attributes
	KalmanModelType modelType;
	std::optional<cv::Mat_<float>> initialState;

private: // Functions
	cv::KalmanFilter buildWithPositionModel() const;
	cv::KalmanFilter buildWithSpeedModel() const;
	cv::KalmanFilter buildWithAccelerationModel() const;

public: // Canonical
	KalmanFilterBuilder();
	virtual ~KalmanFilterBuilder() = default;

public: // Setters
	inline void setModelType(KalmanModelType modelType) { this->modelType = modelType; }
	inline KalmanModelType KalmanModelType() const { return this->modelType; }

public: // Functions
	cv::KalmanFilter build() const;
};

#endif /* SRC_KALMANFILTERBUILDER_HPP_ */
