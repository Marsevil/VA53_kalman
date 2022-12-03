#include <iostream>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

#include "Detector.hpp"
#include "KalmanFilterBuilder.hpp"

#if DEBUG_DETECTOR
#include "DetectorDebugger.hpp"
#endif

using std::string;
using cv::Mat_;
using cv::Vec3b;

namespace fs = std::filesystem;
namespace po = boost::program_options;

const string MAIN_WINDOW_NAME = "Video";
const cv::Scalar_<uint8_t> PREDICTED_POINT_COLOR({0, 255, 0});
const cv::Scalar_<uint8_t> DETECTED_POINT_COLOR({255, 0, 0});
const cv::Scalar_<uint8_t> CORRECTED_POINT_COLOR({0, 0, 255});
const int POINT_RADIUS(10);
const int LINE_WIDTH(2);

void onTrackbar(int trackbarValue, void* userData) {
	int& value = *static_cast<int*>(userData);
	value = trackbarValue;
}

void showHelp() {
	std::cout << "Help !!" << std::endl;
}

void showVersion() {
	std::cout << "Version !!" << std::endl;
}

int main(int argc, char** argv) {
	// Print if some compilation variable are enabled.
#ifdef DEBUG_DETECTOR
	std::cout << "DEBUG_DETECTOR" << std::endl;
#endif

	// TODO : Add detector parameters.
	// TODO : Repair command line arguments for video path
	// TODO : Complete outputs
	// Define command line arguments
	po::positional_options_description argsDesc;
	argsDesc
//		.add("video_path", 1)
		.add("output_stat_path", 1);

	po::options_description genericDesc("Generic options");
	genericDesc.add_options()
			("help,h", "Show help")
			("version,v", "Print version")
			//("video_path", po::value<string>(), "Path to the video")
			("output_stat_path", po::value<string>(), "Path to the output file")
			("optional", po::value<std::vector<string>>(), "Optional arguments");

	po::options_description optionalDesc("Optional options");
	optionalDesc.add_options()
			("model-type", po::value<string>()->default_value("position"), "Specifies Kalman model.\n"
					"position (default)\n"
					"speed\n"
					"acceleration");

	// Parse command line arguments
	po::variables_map args;
	po::parsed_options parsed = po::command_line_parser(argc, argv)
					.options(genericDesc)
					.positional(argsDesc)
					.allow_unregistered()
					.run();
	po::store(parsed, args);
	po::notify(args);

	// Process arguments
	if (args.count("help")) {
		showHelp();
		return EXIT_SUCCESS;
	}
	if (args.count("version")) {
		showVersion();
		return EXIT_SUCCESS;
	}
	if (/*!args.count("video_path") ||*/ !args.count("output_stat_path")) {
		std::cerr << "Video path and output file path must be provided" << std::endl;
		showHelp();
		return EXIT_FAILURE;
	}

	//const fs::path videoPath = static_cast<fs::path>(args["video_path"].as<string>());
	const fs::path outputStatPath = static_cast<fs::path>(args["output_stat_path"].as<string>());

	std::vector<string> optionalArgsStr = po::collect_unrecognized(parsed.options, po::exclude_positional);
	po::variables_map optionalArgs;
	po::store(po::command_line_parser(optionalArgsStr)
			.options(optionalDesc)
			.run(), optionalArgs);
	auto modelType = KalmanFilterBuilder::KalmanModelType::POSITION;
	if (optionalArgs.count("model-type")) {
		const string modelTypeStr = optionalArgs["model-type"].as<string>();
		if (modelTypeStr == "position") {}
		else if (modelTypeStr == "speed") modelType = KalmanFilterBuilder::KalmanModelType::SPEED;
		else if (modelTypeStr == "acceleration") modelType = KalmanFilterBuilder::KalmanModelType::ACCELERATION;
		else {
			std::cerr << "Unknown model type : " << modelTypeStr << std::endl;
			showHelp();
			return EXIT_FAILURE;
		}
	}

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
	kfBuilder.setModelType(modelType);
	kfBuilder.setDt(1.0/60.0);
	cv::KalmanFilter kf = kfBuilder.build();

	// Open video
	//cv::VideoCapture cap(videoPath);
	cv::VideoCapture cap(/* Place video path */);

	// Start video reading
	std::vector<std::optional<cv::Point2f>> precedentDetections;
	std::vector<std::optional<cv::Point2f>> precedentCorrected;
	std::vector<cv::Point2f> precedentPredictions;
	while(cap.isOpened()) {
		// Pre-conditions
		if (cv::waitKey(15) == 27) break;
		Mat_<Vec3b> currentFrame;
		if (!cap.read(currentFrame)) break;

		// Detect & predict object position
		const std::optional<cv::Point2f> detectedPosition = detector->detect(currentFrame);
		std::optional<cv::Point2f> correctedPosition = std::nullopt;
		cv::Point2f predictedPoint;
		{
			cv::Mat_<float> predictedMatrix = kf.predict();
			predictedPoint.x = predictedMatrix(0, 0);
			predictedPoint.y = predictedMatrix(1, 0);
		}

		// Correct Kalman Filter
		if (detectedPosition) {
			cv::Mat_<float> measurement({
				{detectedPosition->x},
				{detectedPosition->y}
			});
			cv::Mat_<float> correctedMatrix = kf.correct(measurement);
			correctedPosition = cv::Point2f(
					correctedMatrix(0, 0),
					correctedMatrix(1, 0));
		}

		// Draw points on output frame
		cv::Mat_<Vec3b> outputFrame = currentFrame.clone();
		cv::circle(outputFrame, predictedPoint, POINT_RADIUS, PREDICTED_POINT_COLOR, -1);
		if (detectedPosition) {
			cv::circle(outputFrame, *detectedPosition, POINT_RADIUS, DETECTED_POINT_COLOR, -1);
		}
		if (correctedPosition) {
			cv::circle(outputFrame, *correctedPosition, POINT_RADIUS, CORRECTED_POINT_COLOR, -1);
		}

		// Save points
		precedentDetections.push_back(detectedPosition);
		precedentPredictions.push_back(predictedPoint);
		precedentCorrected.push_back(correctedPosition);

		// Draw trajectories on the frame
		for (auto p1 = precedentDetections.begin(), p2 = p1+1; p1 != precedentDetections.end()-1; ++p1, ++p2) {
			// Don't draw if a point is missing
			if (!*p1 || !*p2) continue;
			cv::line(outputFrame, **p1, **p2, DETECTED_POINT_COLOR, LINE_WIDTH);
		}
		for (auto p1 = precedentPredictions.begin(), p2 = p1+1; p1 != precedentPredictions.end()-1; ++p1, ++p2) {
			cv::line(outputFrame, *p1, *p2, PREDICTED_POINT_COLOR, LINE_WIDTH);
		}
		for (auto p1 = precedentCorrected.begin(), p2 = p1+1; p1 != precedentCorrected.end()-1; ++p1, ++p2) {
			if (!*p1 || !*p2) continue;
			cv::line(outputFrame, **p1, **p2, CORRECTED_POINT_COLOR, LINE_WIDTH);
		}

		// Show image
		cv::imshow(MAIN_WINDOW_NAME, outputFrame);
	}

	// Give the time to observe the result
	cv::waitKey(0);
	// End with graphical components
	cv::destroyAllWindows();

	// Write datas as a csv file
	std::ofstream csv(outputStatPath);
	csv << "measured X, " << "measured Y, "
			<< "predicted X, " << "predicted Y, "
			<< "corrected X, " << "corrected Y"
			<< std::endl;
	for (auto [pMeasured, pPredicted, pCorrected] = std::tuple{
			precedentDetections.begin(),
			precedentPredictions.begin(),
			precedentCorrected.begin()};
		pMeasured != precedentDetections.end()
		&& pPredicted != precedentPredictions.end()
		&& pCorrected != precedentCorrected.end();
			++pMeasured, ++pPredicted, ++pCorrected) {
		csv << (*pMeasured ? std::to_string((**pMeasured).x) : "") << ", "
				<< (*pMeasured ? std::to_string((**pMeasured).y) : "") << ", "
				<< (*pPredicted).x << ", "
				<< (*pPredicted).y << ", "
				<< (*pCorrected ? std::to_string((**pCorrected).x) : "") << ", "
				<< (*pCorrected ? std::to_string((**pCorrected).y) : "") << std::endl;
	}
	return EXIT_SUCCESS;
}
