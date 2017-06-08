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
  Continuum robot(3);
  robot.addSegment(0,5,10, .4); // SegID , Length, noOfDisks, radius
  robot.setSegmentBasePose(0,tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1));
  robot.setSegmentShape(0,.5,0); // SegID , Kappa, Phi
  robot.addSegment(1,10,20,.3); // SegID , Length, noOfSegments
  robot.setSegmentShape(1,.3,PI/6); // SegID , Kappa, Phi
  robot.addSegment(2,4,10,.2); // SegID , Length, noOfSegments
 robot.setSegmentShape(2,.6,PI/7); // SegID , Kappa, Phi
  while (ros::ok())
  {
robot.update();

  }

}
