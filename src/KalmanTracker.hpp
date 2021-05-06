#ifndef KalmanTracker_HPP
#define KalmanTracker_HPP
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include "ShowManyImages.hpp"


typedef cv::Rect Blob;

enum class MotionModel {
	ConstantVelocity = 0,
	ConstantAcceleration = 1
};

class KalmanTracker {
public:
	KalmanTracker(std::string fileName, MotionModel Model);
	void Tracking();
	MotionModel Model;  
	size_t currentStep = 0;
	cv::KalmanFilter kalmanFilter;
	std::vector<cv::Point> measuredTrajectory;
	std::vector<cv::Point> predictedTrajectory;
	std::vector<cv::Point> estimatedTrajectory;
	std::vector<cv::Point> finalTrajectory;

private:
	cv::VideoCapture cap;	
	cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2; // used for foreground sehmentation
	// Colors
	cv::Scalar red =   cv::Scalar (0, 50, 255);
	cv::Scalar green = cv::Scalar (50, 255, 0);
	cv::Scalar blue =  cv::Scalar (255, 50, 0);
	cv::Scalar white = cv::Scalar (255, 255, 255);

	void ExtractForeground(const cv::Mat& frame, cv::Mat& fgmask, double learningRate);
	Blob ExtractBlob(const cv::Mat& fgmask, cv::Size minSize) const;

	// Kalman Methods
	void KalmanInit(const cv::Point& initialPosition);
	void Prediction();
	void Update(const cv::Point& measurement);

	cv::Mat	BlobCenterPloting(const cv::Mat& frame, const Blob& blob) const;
	cv::Mat	PlotTrajectory(const cv::Mat& frame) const;
	void PlotFinalTrajectory(const cv::Mat& frame) const;
	void ShowImages(const cv::Mat& fgmask, const cv::Mat& frame, const cv::Mat& frameWithBlob, const cv::Mat& trajectories) const;



};

#endif 
