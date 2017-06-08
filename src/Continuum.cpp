/*
 * Continuum.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */

#include"continuum_robot/Continuum.h"

Continuum::Continuum(int noOfSeg){	// TODO Auto-generated constructor stub
// TODO Robot name
this->numberOfSegments = noOfSeg;
this->segmentLength = new double(noOfSeg);
this->noOfDisks = new int(noOfSeg);
this->endEffectorPose = new tf::Transform[noOfSeg];
this->basePose = new tf::Transform[noOfSeg];
this->segKappa = new double[noOfSeg];
this->segPhi = new double[noOfSeg];
this->segTFBroadcaster = new tf::TransformBroadcaster[noOfSeg];
this->segTFFrame  = (tf::Transform**) malloc(noOfSeg * sizeof(tf::Transform*)); // https://stackoverflow.com/questions/455960/dynamic-allocating-array-of-arrays-in-c
}

//
void Continuum::addSegment(int segID, double segLength, int n_disks, double radius){	// TODO Auto-generated constructor stub
	// Continuum robot segment
	this->createURDF(segID, segLength, n_disks, radius);
	segTFFrame[segID] = (tf::Transform*) malloc(n_disks*sizeof(tf::Transform));
	segmentLength[segID] = segLength;
	noOfDisks[segID] = n_disks;
if(segID >0)
{
	basePose[segID].setOrigin(endEffectorPose[segID-1].getOrigin());
	basePose[segID].setRotation(endEffectorPose[segID-1].getRotation());
}
else
{
	basePose[segID].setOrigin(tf::Vector3(0,0,0));
	basePose[segID].setRotation(tf::Quaternion(0,0,0,1));

	endEffectorPose[segID].setOrigin(tf::Vector3(0,0,segLength));
	endEffectorPose[segID].setRotation(tf::Quaternion(0,0,0,1));

}
	segKappa[segID] = 0.00001; // Very small number
	segPhi[segID] = 0.0; // zero
}


void 	Continuum::setSegmentBasePose(int segID, tf::Vector3 basePos, tf::Quaternion baseRot){
	basePose[segID].setOrigin(basePos);
	basePose[segID].setRotation(baseRot);
	}


void Continuum::setSegmentShape(int segID, double kappa, double phi){
	segKappa[segID] = kappa;
	segPhi[segID] = phi;

tf::Matrix3x3 Rot;
tf::Quaternion qRot;
Rot.setValue(pow(cos(phi),2) * (cos(kappa*segmentLength[segID]) - 1) + 1, sin(phi)*cos(phi)*( cos(kappa*segmentLength[segID]) - 1), -cos(phi)*sin(kappa*segmentLength[segID]),
						sin(phi)*cos(phi)*( cos(kappa*segmentLength[segID]) - 1), pow(cos(phi),2) * ( 1 - cos(kappa*segmentLength[segID]) ) + cos( kappa * segmentLength[segID] ),  -sin(phi)*sin(kappa*segmentLength[segID]),
						 cos(phi)*sin(kappa*segmentLength[segID]),  sin(phi)*sin(kappa*segmentLength[segID]), cos(kappa*segmentLength[segID]));
Rot.getRotation(qRot);
endEffectorPose[segID].setRotation(basePose[segID].getRotation() * qRot);

tf::Vector3 eePosition = basePose[segID].getOrigin() + ( tf::Matrix3x3(basePose[segID].getRotation())*tf::Vector3(cos(phi)*( cos(kappa*segmentLength[segID]) - 1)/kappa, sin(phi)*( cos(kappa*segmentLength[segID]) - 1)/kappa, sin(kappa*segmentLength[segID])/kappa));


endEffectorPose[segID].setOrigin(eePosition);

}

void Continuum::update(void) {
char childFrameName[30];
ros::Rate rate(1);
for (int segID = 0;segID<this->numberOfSegments;segID++)
{
	tf::Quaternion slerpQuaternion;
	tf::Quaternion slerpQuaternionCable;
tf::Vector3 eeP;
tf::Vector3 eePc;
for(int i=0;i<noOfDisks[segID];i++){
	eeP[0] = cos(segPhi[segID])*(cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
	eeP[1] = sin(segPhi[segID])*( cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
	eeP[2] = (sin(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID]))/segKappa[segID]);
	eeP =  tf::Matrix3x3(basePose[segID].getRotation())*eeP;
	this->segTFFrame[segID][i].setOrigin(tf::Vector3(basePose[segID].getOrigin().x() + eeP.getX(), basePose[segID].getOrigin().y() + eeP.getY(), basePose[segID].getOrigin().z() + eeP.getZ()) );

		// Slerp for spherical interpolation
		slerpQuaternion = basePose[segID].getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)noOfDisks[segID]-1))));
		this->segTFFrame[segID][i].setRotation(slerpQuaternion);
		sprintf(childFrameName, "S%dL%d", segID,i);
		segTFBroadcaster[segID].sendTransform(tf::StampedTransform(segTFFrame[segID][i], ros::Time::now(),"base_link",childFrameName));

	}

	for(int i=0;i<RESOLUTION;i++){
		eePc[0] = cos(segPhi[segID])*(cos(segKappa[segID]*((i/((double)RESOLUTION-1))*segmentLength[segID])) - 1)/segKappa[segID];
		eePc[1] =  sin(segPhi[segID])*(cos(segKappa[segID]*((i/((double)RESOLUTION-1))*segmentLength[segID])) - 1)/segKappa[segID];
		eePc[2] = (sin(segKappa[segID]*((i/((double)RESOLUTION-1))*segmentLength[segID]))/segKappa[segID]);
		eePc =  tf::Matrix3x3(basePose[segID].getRotation())*eePc;
		// Slerp for spherical interpolation
			slerpQuaternionCable = basePose[segID].getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)RESOLUTION-1))));

		}
rate.sleep();
}
}

void Continuum::createURDF(int segID, double segLength, int n_disks, double radius){	// TODO Auto-generated constructor stub
	 std::string path= ros::package::getPath("continuum_robot");  path =path+ "/urdf/robot_model.urdf";
if(segID == 0)
{ // if the first time to create the robot, delete the previous file
	remove(path.c_str());

robotURDFfile.open (path.c_str(), std::fstream::app);
robotURDFfile << "<?xml version=\"1.0\"?>" <<endl;
robotURDFfile << "<robot name=\"continuum_robot\">"<<endl;
robotURDFfile << "<link name=\"base_link\"/>"<<endl;
}
else{robotURDFfile.open (path.c_str(), std::fstream::app);}

robotURDFfile<<endl;
	  for(int disk=0;disk<n_disks;disk++)
	  {
		  robotURDFfile << "<link name=\"S"<<segID<<"L"<<disk<<"\">"<<endl;
		  robotURDFfile << "<visual>"<<endl;
		  robotURDFfile << "<geometry>"<<endl;
		  robotURDFfile << "<cylinder length=\"0.1\" radius=\""<<radius<<"\"/>"<<endl;
		  robotURDFfile << "<origin rpy=\"0 0 0\" xyz=\"0 0 "<<(disk/(n_disks-1))*segLength<<"\"/>"<<endl;
		  robotURDFfile << "</geometry>"<<endl;
		  robotURDFfile <<"</visual>"<<endl;
		  robotURDFfile <<"<collision>"<<endl;
		  robotURDFfile <<"<geometry>"<<endl;
		  robotURDFfile << "<cylinder length=\"0.1\" radius=\""<<radius<<"\"/>"<<endl;
		  robotURDFfile << "</geometry>"<<endl;
		  robotURDFfile <<"</collision>"<<endl;
		  robotURDFfile <<"</link>"<<endl;
		  robotURDFfile <<endl;
		  robotURDFfile<<"<joint name=\"S"<<segID<<"J"<<disk<<"\" type=\"floating\">"<<endl;
		  robotURDFfile<<"<parent link=\"base_link\"/>"<<endl;
		  robotURDFfile<<"<child link=\"S"<<segID<<"L"<<disk<<"\"/>"<<endl;
		  robotURDFfile<<"</joint>"<<endl;
		  robotURDFfile<<endl;

	  }
	 if(segID == (this->numberOfSegments - 1)) robotURDFfile <<"</robot>"<<endl; // a closing tag
	  robotURDFfile.close();

}



Continuum::~Continuum() {
	// TODO Auto-generated destructor stub
}


