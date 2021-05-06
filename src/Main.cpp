//Builtin libraries C/C++
#include <stdio.h>
#include <iostream>
#include <sstream>

//handcrafted libraries
#include "KalmanTracker.hpp"

// main function
int main (int argc, char** argv)
{
	MotionModel Model;

	// dataset
	string path = "/dataset/";
	string videoNum = string(argv[2]);	
	string format = ".mp4";
	string DataSetDir = path + videoNum + format;

	// print the inputs
	cout<< DataSetDir << "\n";

	// Choose the motion model
	if(string(argv[1]) == "velocity") {Model = MotionModel::ConstantVelocity; cout<<"Constant Velocity Model\n";}
	else if(string(argv[1]) == "acceleration") {Model = MotionModel::ConstantAcceleration;cout<<"Constant Acceleration Model\n";}
	else{Model = MotionModel::ConstantAcceleration;}
	

	// initialize the Tracker
	KalmanTracker Tracker(DataSetDir, Model);
	
	// Start the Tracker
	Tracker.Tracking();

	return 0;
}




