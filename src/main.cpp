#include <iostream>
#include <opencv2/opencv.hpp>
#include "Detector.hpp"
#include "KalmanFilterBuilder.hpp"
#if DEBUG_DETECTOR
#include "DetectorDebugger.hpp"
#endif

using std::string;
using cv::Mat_;
using cv::Vec3b;

void onTrackbar(int trackbarValue, void* userData) {
	int& value = *static_cast<int*>(userData);
	value = trackbarValue;
}

int main(int c, char** argv) {
	std::cout << "BEGIN" << std::endl;
#ifdef DEBUG_DETECTOR
	std::cout << "DEBUG_DETECTOR" << std::endl;
#endif
	// Declare some constants
	const string MAIN_WINDOW_NAME = "Video";
	// Open main window
	cv::namedWindow(MAIN_WINDOW_NAME, cv::WINDOW_NORMAL);

	// Set up detection
	//Detector detector;
	std::unique_ptr<IDetector> detector;
#if DEBUG_DETECTOR
	detector.reset(new DetectorDebugger);
#else
	detector.reset(new Detector);
#endif

	// Build KalmanFilter
	KalmanFilterBuilder kfBuilder;
	kfBuilder.setModelType(KalmanFilterBuilder::KalmanModelType::SPEED);
	cv::KalmanFilter kf = kfBuilder.build();

	// Open video
	cv::VideoCapture cap("./Dataset/Vidéo 1 60 FPS HD.MOV");

	std::vector<std::optional<cv::Point2f>> precedentDetections;
	std::vector<cv::Point2f> precedentPredictions;
	while(cap.isOpened()) {
		if (cv::waitKey(15) == 27) break;
		Mat_<Vec3b> currentFrame;
		if (!cap.read(currentFrame)) break;

		std::optional<cv::Point2f> detectedPosition = detector->detect(currentFrame);

		cv::Point2f predictedPoint;
		{
			cv::Mat_<float> predictedMatrix = kf.predict();
			predictedPoint.x = predictedMatrix(0, 0);
			predictedPoint.y = predictedMatrix(1, 0);
		}

		const cv::Scalar_<uint8_t> PREDICTED_POINT_COLOR({0, 255, 0});
		const cv::Scalar_<uint8_t> DETECTED_POINT_COLOR({255, 0, 0});
		const int POINT_RADIUS(10);
		const int LINE_WIDTH(2);
		cv::Mat_<Vec3b> outputFrame = currentFrame.clone();
		cv::circle(outputFrame, predictedPoint, POINT_RADIUS, PREDICTED_POINT_COLOR, -1);
		if (detectedPosition) {
			cv::circle(outputFrame, *detectedPosition, POINT_RADIUS, DETECTED_POINT_COLOR, -1);
		}

		// Correct Kalman Filter
		if (detectedPosition) {
			cv::Mat_<float> measurement({
				{detectedPosition->x},
				{detectedPosition->y}
			});
			kf.correct(measurement);
		}

		precedentDetections.push_back(detectedPosition);
		precedentPredictions.push_back(predictedPoint);

		//draw trajectories
		for (auto p1 = precedentDetections.begin(), p2 = p1+1; p1 != precedentDetections.end()-1; ++p1, ++p2) {
			if (!*p1 || !*p2) continue;
			cv::line(outputFrame, **p1, **p2, DETECTED_POINT_COLOR, LINE_WIDTH);
		}

		for (auto p1 = precedentPredictions.begin(), p2 = p1+1; p1 != precedentPredictions.end()-1; ++p1, ++p2) {
			cv::line(outputFrame, *p1, *p2, PREDICTED_POINT_COLOR, LINE_WIDTH);
		}

		// Show window
		cv::imshow(MAIN_WINDOW_NAME, outputFrame);
	}

	cv::destroyAllWindows();
	std::cout << "END" << std::endl;
	return EXIT_SUCCESS;
}
