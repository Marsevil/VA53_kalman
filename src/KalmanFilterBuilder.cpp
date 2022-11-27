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

void KalmanFilterBuilder::setErrorCovMatrix(cv::KalmanFilter& kf) const {
	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(.03));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(.03));
}

cv::KalmanFilter KalmanFilterBuilder::build() const {
	switch(this->modelType) {
	case KalmanModelType::SPEED:
		return buildWithSpeedModel();
	case KalmanModelType::ACCELERATION:
		return buildWithAccelerationModel();
	case KalmanModelType::POSITION:
	default:
		return buildWithPositionModel();
	}
}

cv::KalmanFilter KalmanFilterBuilder::buildWithPositionModel() const {
	cv::KalmanFilter kf(2, 2, 0);

	// Define matrix.
	cv::setIdentity(kf.transitionMatrix);
	cv::setIdentity(kf.measurementMatrix);
	setErrorCovMatrix(kf);

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

cv::KalmanFilter KalmanFilterBuilder::buildWithSpeedModel() const {
	cv::KalmanFilter kf(4, 2, 0);

	// Define matrix
	kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
			1, 0, 1, 0,
			0, 1, 0, 1,
			0, 0, 1, 0,
			0, 0, 0, 1);
	kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
			1, 0, 0, 0,
			0, 1, 0, 0);
	setErrorCovMatrix(kf);

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

cv::KalmanFilter KalmanFilterBuilder::buildWithAccelerationModel() const {
	cv::KalmanFilter kf(6, 2, 0);

	// Define matrix
	kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
			1, 0, 1, 0, 0, 0,
			0, 1, 0, 1, 0, 0,
			0, 0, 1, 0, 1, 0,
			0, 0, 0, 1, 0, 1,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1);
	kf.measurementMatrix = (cv::Mat_<float>(2, 6) <<
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0);
	setErrorCovMatrix(kf);

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
