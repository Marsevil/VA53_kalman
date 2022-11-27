/*
 * KalmanFilterBuilder.cpp
 *
 *  Created on: 27 nov. 2022
 *      Author: marsevil
 */

#include "KalmanFilterBuilder.hpp"

KalmanFilterBuilder::KalmanFilterBuilder():
modelType(POSITION),
initialState(std::nullopt) {
	// Nothing to do.
}

cv::KalmanFilter KalmanFilterBuilder::build() const {
	switch(this->modelType) {
	case KalmanModelType::POSITION:
	case KalmanModelType::SPEED:
	case KalmanModelType::ACCELERATION:
	default:
		return buildWithPositionModel();
	}
}

cv::KalmanFilter KalmanFilterBuilder::buildWithPositionModel() const {
	cv::KalmanFilter kf(2, 2, 0);

	// Define matrix.
	cv::setIdentity(kf.transitionMatrix);
	cv::setIdentity(kf.measurementMatrix);
	cv::setIdentity(kf.processNoiseCov);
	cv::setIdentity(kf.measurementNoiseCov);
	cv::setIdentity(kf.errorCovPost);

	// Define initial state
	if (this->initialState) {
		auto state = *initialState;
		kf.statePre.setTo(state);
		kf.statePost.setTo(state);
	} else {
		kf.statePre.setTo(0);
		kf.statePost.setTo(0);
	}

	return kf;
}
