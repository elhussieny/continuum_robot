/*
 * simulator.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */
#include <iostream>
#include "continuum_robot/Continuum.h"


int main( int argc, char** argv )
{
  ros::init(argc, argv, "continuum_core");
  Continuum robot(2);
 robot.addSegment(0,6,6, .4); // SegID , Length, noOfDisks, radius
 robot.setSegmentBasePose(0,tf::Vector3(0,0,0),tf::createQuaternionFromRPY(0.0, 90*PI/180,0.0));
  robot.setSegmentShape(0,0.00001,0,TAIL); // SegID , Kappa, Phi
robot.addSegment(1,6,6,.3); // SegID , Length, noOfSegments
robot.setSegmentShape(1,0.0001,0, TAIL); // SegID , Kappa, Phi
 /* robot.addSegment(2,6,6,.2); // SegID , Length, noOfSegments
 robot.setSegmentShape(2,0.00001,0, TAIL); // SegID , Kappa, Phi*/
 robot.addHead(6,6,.6); // Length, noOfDisks, radius
 robot.setHeadParameters(0.0001,0, NORMAL);
  while (ros::ok())
  { // set a pattern

/*for(double i=0.0001;i<=0.3;i=i+0.01)
{
//robot.setSegmentShape(0,i,0,TAIL); // SegID , Kappa, Phi
robot.update();
}*/

/*for(double i=0.25;i>=-0.25;i=i-0.01)
{
	robot.setSegmentShape(0,i,0); // SegID , Kappa, Phi
//robot.update();
}*/
/*
for(double i=0.1;i<=.5;i=i+0.01)
{
	robot.setSegmentShape(1,i,PI/2); // SegID , Kappa, Phi
robot.update();
}

for(double i=.5;i>=0.1;i=i-0.01)
{
	robot.setSegmentShape(1,i,PI/2); // SegID , Kappa, Phi
robot.update();
}

for(double i=-.5;i<=.5;i=i+0.01)
{
	robot.setSegmentShape(2,i,PI/7); // SegID , Kappa, Phi
robot.update();
}

for(double i=.5;i>=-.5;i=i-0.01)
{
	robot.setSegmentShape(2,i,PI/7); // SegID , Kappa, Phi
robot.update();
}
*/


robot.update();
 ros::spinOnce();
  }
}
