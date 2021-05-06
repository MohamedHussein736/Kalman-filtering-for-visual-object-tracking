#include "opencv2/opencv.hpp"
#include "KalmanTracker.hpp"

using namespace cv;
using namespace std;

#define HISTORY 50
#define VAR_THRESHOLD 16
#define LEARNING_RATE 0.001
#define BLOBSIZE 10
KalmanTracker::KalmanTracker(string fileName, MotionModel Model): Model(Model)
{
	// try opening the video file
	cap.open (fileName);
	if (!cap.isOpened ()) {cerr << "Could not open video file " << fileName << endl;}

	// inititalize the background subtractor
	pMOG2 = createBackgroundSubtractorMOG2 (HISTORY,VAR_THRESHOLD,true);

	if (Model == MotionModel::ConstantVelocity) {
		kalmanFilter.init (4, 2);
		kalmanFilter.transitionMatrix  = (Mat_<float> (4, 4) << 1, 1, 0, 0,
																0, 1, 0, 0,
																0, 0, 1, 1,
																0, 0, 0, 1); // A

		kalmanFilter.processNoiseCov   = (Mat_<float> (4, 4) << 25, 0, 0, 0,
																0, 10, 0, 0,
																0, 0, 25, 0,
																0, 0, 0, 10); // Q

		kalmanFilter.measurementMatrix = (Mat_<float> (2, 4) << 1, 0, 0, 0,
																0, 0, 1, 0); // H
	}
	else if (Model == MotionModel::ConstantAcceleration) {
		kalmanFilter.init (6, 2);
		kalmanFilter.transitionMatrix  = (Mat_<float> (6, 6) << 1, 1, 0.5, 0, 0, 0,
																0, 1, 1, 0, 0, 0,
																0, 0, 1, 0, 0, 0,
																0, 0, 0, 1, 1, 0.5,
																0, 0, 0, 0, 1, 1,
																0, 0, 0, 0, 0, 1); // A


		kalmanFilter.processNoiseCov   = (Mat_<float> (6, 6) << 25, 0, 0, 0, 0, 0,
																0, 10, 0, 0, 0, 0,
																0, 0, 1, 0, 0, 0,
																0, 0, 0, 25, 0, 0,
																0, 0, 0, 0, 10, 0,
																0, 0, 0, 0, 0, 1); // Q

		kalmanFilter.measurementMatrix = (Mat_<float> (2, 6) << 1, 0, 0, 0, 0, 0, 
																0, 0, 0, 1, 0, 0); // H
	}
	setIdentity(kalmanFilter.measurementNoiseCov, Scalar::all(50)); // R	
	setIdentity(kalmanFilter.errorCovPost, Scalar::all(1e5));	// P
}
/************************************************************************
 * Kalman Filter Pipeline loop - Start
 ************************************************************************/
void KalmanTracker::Tracking()
{
	// check if the video openable
	if (!cap.isOpened()) {return;}
	// current Frame, foreground mask
	Mat img, frame, fgmask;	
	// tracked object
	Blob object;	
	// Booleans for kalman initialization, object disappearance
	bool initFoundObject, Disappear = false;


	// Tracking pipeline
	while (true) {
		// get frame
		cap >> img;
		// check if we achieved the end of the file 
		if (!img.data) {break;}
		// copy current image
		img.copyTo(frame);

		// extract the foreground 
		ExtractForeground(frame, fgmask, LEARNING_RATE);

		// extract the biggest blob
		object = ExtractBlob(fgmask, Size(BLOBSIZE, BLOBSIZE));

		// until there hasn't been a ball in the scene we don't start the kalman filter predictions
		if(initFoundObject){Prediction();}

		// get the measurement, check the blob area > o
		Disappear = (object.area() > 0) ? false : true;

		if(!Disappear){
			Point measurement = Point (object.x + object.width / 2, object.y + object.height / 2);
			//  initialized with this first position
			if (!initFoundObject) {
				KalmanInit(measurement);
				initFoundObject = true;
				}
			else
				{Update(measurement);}
		}
		// wait ESC 
		if(waitKey(30) == 27){break;}

		// show the current measurements
		ShowImages (fgmask, frame, BlobCenterPloting(frame, object), PlotTrajectory(frame));
	} 

	// show trajectories
	ShowImages (fgmask, frame, BlobCenterPloting(frame, object), PlotTrajectory (frame));
	PlotFinalTrajectory(frame);
	// wait
	waitKey(0);
	cap.release ();
	destroyAllWindows ();
}
/************************************************************************
 * Kalman Filter Pipeline loop - End
 ************************************************************************/

/************************************************************************
 * Foreground and Blob Extraction method - Start
 ************************************************************************/
// applies the background subtractor and a morphological opening to get the foreground
void KalmanTracker::ExtractForeground(const Mat& frame, Mat& fgmask, double learningRate)
{
	int morph_size_x = 3;
	int morph_size_y = 3;
	pMOG2->apply(frame, fgmask, learningRate);
	Mat kernel = getStructuringElement (MORPH_RECT, Size (2 * morph_size_x + 1, 2 * morph_size_y + 1), Point (morph_size_x, morph_size_y));
	morphologyEx (fgmask, fgmask, MORPH_OPEN, kernel);
}

// Extract the biggest object
Blob KalmanTracker::ExtractBlob(const Mat& fgmask, Size minSize) const
{
	Mat aux; // image to be updated blob
	fgmask.convertTo (aux, CV_32SC1);
	// component analysis
	Blob rect, result;
	for (int i = 0; i < aux.rows; i++) {
		for (int j = 0; j < aux.cols; j++) {
			if (aux.at<int> (i, j) == 255) {
				floodFill (aux, Point (j, i), 1024, &rect, Scalar (), Scalar (), 8);
				// extract size
				bool sizeCheck  = rect.size().width > minSize.width && rect.size().height > minSize.height && rect.size().area () > result.area ();
				// extract the biggest blob
				if (sizeCheck){result = rect;}
			}
		}
	}
	return result;
}
/************************************************************************
 * Foreground and Blob Extraction method - End
 ************************************************************************/

/************************************************************************
 * Kalman Filter Method - Start
 ************************************************************************/
void KalmanTracker::KalmanInit(const Point& measurement)
{

	//setIdentity(kalmanFilter.errorCovPre);
	measuredTrajectory.push_back(measurement);

	// ConstantVelocity
	if (Model == MotionModel::ConstantVelocity) {
		kalmanFilter.statePost.at<float> (0) = measurement.x;
		kalmanFilter.statePost.at<float> (2) = measurement.y;
	}
	// ConstantAccelration
	else {
		kalmanFilter.statePost.at<float> (0) = measurement.x;
		kalmanFilter.statePost.at<float> (3) = measurement.y;
	}
}

// prediction step
void KalmanTracker::Prediction()
{
	Mat prediction = kalmanFilter.predict();
	// select the motion model
	if (Model == MotionModel::ConstantVelocity) 
		{predictedTrajectory.push_back(Point2f(prediction.at<float>(0), prediction.at<float>(2)));}
	else 
		{predictedTrajectory.push_back(Point2f(prediction.at<float>(0), prediction.at<float>(3)));}
	// store the final trajectory
	finalTrajectory.push_back(predictedTrajectory.back());
}

// Update step 
void KalmanTracker::Update(const cv::Point& measurement)
{
	Mat estimation = kalmanFilter.correct ((Mat_<float>(2, 1) << (float)measurement.x, (float)measurement.y));
	measuredTrajectory.push_back(measurement);	
	if (Model == MotionModel::ConstantVelocity)
		{estimatedTrajectory.push_back (Point2f (estimation.at<float> (0), estimation.at<float> (2)));}
	else
		{estimatedTrajectory.push_back (Point2f (estimation.at<float> (0), estimation.at<float> (3)));}
	// store the final trajectory
	finalTrajectory.back() = estimatedTrajectory.back();
}

/************************************************************************
 * Kalman Filter Method - End
 ************************************************************************/

/************************************************************************
 * Plotting Method - Start
 ************************************************************************/
// paint a given blob on the frame
Mat	KalmanTracker::BlobCenterPloting (const Mat& frame, const Blob& blob) const
{
	Mat result;
	Point point;
	frame.copyTo (result);
	int radius = blob.width / 2;
	point.x = blob.x + blob.width / 2;
	point.y = blob.y + blob.height / 2;
	circle(result, Point(blob.x + blob.width / 2, blob.y + blob.height / 2), blob.width / 2 , red, 3);
	line  (result, Point(point.x - radius/2, point.y), Point (point.x + radius/2, point.y)  , red, 2);
	line  (result, Point(point.x, point.y - radius / 2), Point (point.x, point.y + radius/2), red, 2);
	return result;
}


// draws all the saved trajectories on the frame
Mat	KalmanTracker::PlotTrajectory (const Mat& frame) const
{
	Mat result;
	int radius = 10;
	frame.copyTo (result);	

	for (Point point : predictedTrajectory) {circle(result, Point(point.x , point.y), 5, green, -1);}
	for (Point point : measuredTrajectory ) {circle(result, Point(point.x , point.y), 5, red, -1);}
	for (Point point : estimatedTrajectory) {circle(result, Point(point.x , point.y), 5, blue, -1);}
	return result;
}


// shows the final trajectory in a new window
void KalmanTracker::PlotFinalTrajectory (const Mat& frame) const
{
	Mat result;
	int radius = 5;
	frame.copyTo (result);
	
	// iterate over the measurements trajectory
	for (Point point : measuredTrajectory){circle (result, point, 3, red, -1);}
	// plot the measurements points
	for (size_t i = 0; i < measuredTrajectory.size () - 1; i++) {line (result, measuredTrajectory[i], measuredTrajectory[i + 1], red, 2.5);}
	// iterate over the predictions trajectory
	for (Point point : predictedTrajectory) {circle (result, point, 3, blue,-1);}
	// plot the predictions points
	for (size_t i = 0; i < predictedTrajectory.size () - 1; i++) {line (result, predictedTrajectory[i], predictedTrajectory[i + 1], blue, 2.5);}

	putText (result, "Measurements", Point (5, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar (0, 0, 255));
	putText (result, "Final Estimation Trajectory", Point (5, 30), FONT_HERSHEY_SIMPLEX, 0.5, Scalar (255, 0, 0));
	namedWindow ("Kalman filtering", WINDOW_AUTOSIZE);
	imshow ("Final Trajectory", result);
}


void KalmanTracker::ShowImages(const Mat& fgmask, const Mat& frame, const Mat& Blobframe, const Mat& trajectories) const
{
	Mat TargetBlob; Blobframe.copyTo (TargetBlob);
	Mat trajectory; trajectories.copyTo (trajectory);
	putText (trajectory, "Measurements", Point (5, 15), FONT_HERSHEY_DUPLEX, 0.5, Scalar (0, 0, 255));
	putText (trajectory, "Predictions" , Point (5, 30), FONT_HERSHEY_DUPLEX, 0.5, Scalar (0, 255, 0));
	putText (trajectory, "Estimations" , Point (5, 45), FONT_HERSHEY_DUPLEX, 0.5, Scalar (255, 0, 0));
	ShowManyImages ("Kalman filtering" , 2, TargetBlob, trajectory);
}

/************************************************************************
 * Plotting Method - End
 ************************************************************************/
