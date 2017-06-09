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
  Continuum robot(1);
  robot.addSegment(0,5,4, .4); // SegID , Length, noOfDisks, radius
  robot.setSegmentBasePose(0,tf::Vector3(0,0,0),tf::Quaternion(0,0.707,0,0.707));
  robot.setSegmentShape(0,.1,0); // SegID , Kappa, Phi
// robot.addSegment(1,10,20,.3); // SegID , Length, noOfSegments
//  robot.setSegmentShape(1,.3,PI/6); // SegID , Kappa, Phi
//  robot.addSegment(2,4,10,.2); // SegID , Length, noOfSegments
// robot.setSegmentShape(2,.6,PI/7); // SegID , Kappa, Phi
  while (ros::ok())
  {
for(double i=0.1;i<2;i=i+0.05)
{
	robot.setSegmentShape(0,i,0); // SegID , Kappa, Phi
robot.update();
}

  }

}
