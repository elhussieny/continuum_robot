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

using namespace std;
#define PI 3.1415926
#define RESOLUTION 100

class Continuum {
private:
	 tf::Transform* endEffectorPose;
	 tf::Transform* basePose;
	 tf::Transform** segTFFrame; // array of array segTFFrame[segID][diskNo]
	 tf::TransformBroadcaster* segTFBroadcaster;
	 visualization_msgs::MarkerArray* cableMarkers;
	 ros::Publisher* cablePublisher;

	 double* segmentLength;
	 int* noOfDisks;
	 int numberOfSegments;
	 double* segKappa;
	 double* segPhi;

	 //-------------------------------------------------------------
	 std_msgs::String robotName;
	 ofstream robotURDFfile;
	 void createURDF(int segID, double length, int n_disks, double radius);
	 void initCableMarker(int segID);

public:
	Continuum(int noOfSeg);
	void addSegment(int segID, double length, int n_disks, double radius);

	void setSegmentBasePose(int segID, tf::Vector3 basePos, tf::Quaternion baseRot);
	void setSegmentShape(int segID, double kappa, double phi);
	void update(void);


	virtual ~Continuum();
};

#endif /* CONTINUUM_ROBOT_INCLUDE_CONTINUUM_ROBOT_CONTINUUM_H_ */
