#ifndef __kinect__
#define __kinect__

#include "Includes/includes.h"

// Projekt
#include "RGBD/RGBDprocessing.h"
#include "Show/Show.h"
#include "Filtry/Filtry.h"
#include "Track/track.h"

class Kinect
{
public:
	// Program parameters
	ProgramParameters programParameters;

	// Calibration parameters
	CalibrationParameters calibrationParams;
	double depthInvScale;

	// Kinect - initial transformation
	Eigen::Matrix4f KinectInitial;
	
	// Log
	ofstream zapis;

	// New Class
	RGBDclass *RGBD;
	Track *tracking;

	// Konstruktor
	Kinect();	

	// Destruktor
	~Kinect();

	// Reading working parameters
	void ReadParameters();
	
	// Reading camera parameters
	void ReadCalibrationParameters();

	// Reading the initial position in better way
	void ReadInitialTransformation();

	// Save Trasformation
	void saveTransformation(char *wzg, Eigen::Matrix4f transformation);

	/// Tracking
	void TrackingRun();
};
#endif
