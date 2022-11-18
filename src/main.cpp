#include <iostream>
#include <opencv2/opencv.hpp>
#include "Detector.hpp"
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

	// Open video
	cv::VideoCapture cap("./Dataset/Vidéo 1 60 FPS HD.MOV");

	while(cap.isOpened()) {
		if (cv::waitKey(15) == 27) break;
		Mat_<Vec3b> currentFrame;
		if (!cap.read(currentFrame)) break;

		detector->detect(currentFrame);

		// Show window
		cv::imshow(MAIN_WINDOW_NAME, currentFrame);
	}

	cv::destroyAllWindows();
	std::cout << "END" << std::endl;
	return EXIT_SUCCESS;
}
