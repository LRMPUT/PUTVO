#ifndef __kinect__
#define __kinect__

#include "Includes/includes.h"

// Projekt
#include "RGBD/RGBDprocessing.h"
#include "Show/Show.h"
#include "Filtry/Filtry.h"
#include "Track/track.h"


struct geometricDistance
{
	int i, j;
	double distance;
};



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

private:
	void saveTrajectory(Eigen::Matrix4f transformation,
			std::ofstream & estTrajectory, const std::string& timestamp);
	void saveG20Vertex(Eigen::Matrix4f transformation,
			 std::ofstream& g2o, const kabschVertex& tmpVertex);
	void saveG20Vertex(Eigen::Matrix4f transformation,
				 std::ofstream& g2o, const int id);
	void saveG20Edge(std::ofstream& g2o, const kabschVertex& tmpVertex,
			int i);
	void saveG20Edge(std::ofstream& g2o, const int vertex1Id,
			const int vertex2Id, Eigen::Matrix4f transformation);
	void saveG20Fix(std::ofstream& g2o, const int id);

	void findAndSaveG20Features(kabschVertex& secondFrame, std::ofstream& g2o, const int firstOrSecond);
};
#endif
