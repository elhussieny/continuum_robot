/*
 * Continuum.h
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */

#ifndef CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_
#define CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_
#include <ros/ros.h>
#include <math.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <fstream>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <termios.h>

using namespace std;
#define PI 3.1415926
#define RESOLUTION 100
#define INTERFACE 0
#define delay 1
#define HEAD 1
#define TAIL 0
#define FLIPPED 1
#define NORMAL 0
#define UPDATERATE 50
class Continuum {
private:
	 tf::Transform* endEffectorPose;
	 tf::Transform* basePose;
	 tf::Transform** segTFFrame; // array of array segTFFrame[segID][diskNo]
	 tf::TransformBroadcaster* segTFBroadcaster;
	 visualization_msgs::MarkerArray* cableMarkers;
	 visualization_msgs::MarkerArray headMarkers;
	 ros::Publisher* cablePublisher;
	 ros::Publisher headPublisher;
	 ros::Timer frame_timer;
	 bool hasHead;
	 int headDisks;
	 double headLength;
	 double* arrayOfKappa;
	 double* arrayOfPhi;
	 double* segmentLength;
	 int* noOfDisks;
	 int* segmentMode;
	 double* segKappa;
	 double* segPhi;
	 double headPhi;
	 int headMode;
	 double headKappa;
	 struct termios initial_settings,
	                new_settings;
	 int rateOfUpdate;
	 //-------------------------------------------------------------
	 std_msgs::String robotName;
	 ofstream robotURDFfile;
	 void createURDF(int segID, double length, int n_disks, double radius);
	 void initCableMarker(int segID);
	 tf::Quaternion getDiskQuaternion(int segID, int diskID);
	 tf::Quaternion getHeadQuaternion(int diskID);

	 tf::Vector3 getDiskPosition(int segID, int i);
	 void timerScanning(const ros::TimerEvent&);
public:
	Continuum();
	 int numberOfSegments;

	void addSegment(int segID, double length, int n_disks, double radius);

	void setSegmentBasePose(int segID, tf::Vector3 basePos, tf::Quaternion baseRot);
	void setSegmentShape(int segID, double kappa, double phi);
	void update(void);
	void addHead(double len, int disks, double rad);
	void setHeadParameters(double headKap, double headPhi, int MODE);


	virtual ~Continuum();
};

#endif /* CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_ */
