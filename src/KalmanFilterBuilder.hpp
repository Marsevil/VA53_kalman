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
	float dt;
	std::optional<cv::Mat_<float>> initialState;

private: // Functions
	cv::KalmanFilter buildWithPositionModel() const;
	cv::KalmanFilter buildWithSpeedModel() const;
	cv::KalmanFilter buildWithAccelerationModel() const;

	void setErrorCovMatrix(cv::KalmanFilter& kf) const;

public: // Canonical
	KalmanFilterBuilder();
	virtual ~KalmanFilterBuilder() = default;

public: // Getters & Setters
	inline void setModelType(KalmanModelType modelType) { this->modelType = modelType; }
	inline KalmanModelType const& KalmanModelType() const { return this->modelType; }
	inline void setInitialState(std::optional<cv::Mat_<float>> initialState) { this->initialState = initialState; }
	inline std::optional<cv::Mat_<float>> const& getInitialState() const { return this->initialState; }
	inline void setDt(float dt) { this->dt = dt; }
	inline float getDt() const { return this->dt; }

public: // Functions
	cv::KalmanFilter build() const;
};

#endif /* SRC_KALMANFILTERBUILDER_HPP_ */
