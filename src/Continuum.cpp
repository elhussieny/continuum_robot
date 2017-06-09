/*
 * Continuum.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */

#include"continuum_robot/Continuum.h"

Continuum::Continuum(int noOfSeg){	// TODO Auto-generated constructor stub
// TODO Robot name
	ros::NodeHandle nh_;
	char cableTopic[30];
this->numberOfSegments = noOfSeg;
this->segmentLength = new double(noOfSeg);
this->noOfDisks = new int(noOfSeg);
this->endEffectorPose = new tf::Transform[noOfSeg];
this->basePose = new tf::Transform[noOfSeg];
this->segKappa = new double[noOfSeg];
this->segPhi = new double[noOfSeg];
this->segTFBroadcaster = new tf::TransformBroadcaster[noOfSeg];
this->segTFFrame  = (tf::Transform**) malloc(noOfSeg * sizeof(tf::Transform*)); // https://stackoverflow.com/questions/455960/dynamic-allocating-array-of-arrays-in-c
this->cableMarkers = new visualization_msgs::MarkerArray[noOfSeg];
this->cablePublisher = new ros::Publisher[noOfSeg];
for(int s=0; s<noOfSeg; s++)
{
	sprintf(cableTopic, "cable_%d", s);
	cableMarkers[s].markers.resize(RESOLUTION);
	cablePublisher[s] = nh_.advertise<visualization_msgs::MarkerArray>(cableTopic,1);
}

}
/******************************************************/

void Continuum::addSegment(int segID, double segLength, int n_disks, double radius){	// TODO Auto-generated constructor stub
	// Continuum robot segment
	this->createURDF(segID, segLength, n_disks, radius);
	segTFFrame[segID] = (tf::Transform*) malloc(n_disks*sizeof(tf::Transform));
	segmentLength[segID] = segLength;
	noOfDisks[segID] = n_disks;
	initCableMarker(segID);

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
/******************************************************/

void Continuum::setSegmentBasePose(int segID, tf::Vector3 basePos, tf::Quaternion baseRot){
	basePose[segID].setOrigin(basePos);
	basePose[segID].setRotation(baseRot);
	}

/******************************************************/
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
/******************************************************/

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
	if(segKappa[segID]*segmentLength[segID]>PI)
	slerpQuaternion = basePose[segID].getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)noOfDisks[segID]-1))));
	else
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
		cableMarkers[segID].markers[i].pose.position.x = basePose[segID].getOrigin().x()+ eePc[0];
				cableMarkers[segID].markers[i].pose.position.y = basePose[segID].getOrigin().y()+ eePc[1];
				cableMarkers[segID].markers[i].pose.position.z = basePose[segID].getOrigin().z()+ eePc[2];
		// Slerp for spherical interpolation
			slerpQuaternionCable = basePose[segID].getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)RESOLUTION-1))));
			cableMarkers[segID].markers[i].pose.orientation.x = 0;//slerpQuaternionCable.x();
			cableMarkers[segID].markers[i].pose.orientation.y = 0;//slerpQuaternionCable.y();
			cableMarkers[segID].markers[i].pose.orientation.z = 0;//slerpQuaternionCable.z();
			cableMarkers[segID].markers[i].pose.orientation.w = 1;//slerpQuaternionCable.w();

		}
cablePublisher[segID].publish(cableMarkers[segID]);
}
rate.sleep();

}
/******************************************************/

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
/*		  robotURDFfile <<"<collision>"<<endl;
		  robotURDFfile <<"<geometry>"<<endl;
		  robotURDFfile << "<cylinder length=\"0.1\" radius=\""<<radius<<"\"/>"<<endl;
		  robotURDFfile << "</geometry>"<<endl;
		  robotURDFfile <<"</collision>"<<endl;*/
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
/******************************************************/
void Continuum::initCableMarker(int segID){
	uint32_t shape = visualization_msgs::Marker::SPHERE;

	  for(int r=0; r<RESOLUTION; r++)
	  {

	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    cableMarkers[segID].markers[r].header.frame_id = "base_link";
	    cableMarkers[segID].markers[r].header.stamp = ros::Time::now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    cableMarkers[segID].markers[r].ns = "basic_shapes";
	    cableMarkers[segID].markers[r].id = r;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    cableMarkers[segID].markers[r].type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    cableMarkers[segID].markers[r].action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    cableMarkers[segID].markers[r].scale.x = .1;
	    cableMarkers[segID].markers[r].scale.y = .1;
	    cableMarkers[segID].markers[r].scale.z = .1;

	    // Set the color -- be sure to set alpha to something non-zero!
	    cableMarkers[segID].markers[r].color.b = 1.0f;
	    cableMarkers[segID].markers[r].color.a = 1.0;
	    cableMarkers[segID].markers[r].lifetime = ros::Duration();
	  }
}
/******************************************************/
Continuum::~Continuum() {
	// TODO Auto-generated destructor stub
}


