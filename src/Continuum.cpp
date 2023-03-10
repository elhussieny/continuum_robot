/*
 * Continuum.cpp
 *
 *  Created on: Apr 1, 2017
 *      Author: haitham
 */

#include"continuum_robot/Continuum.h"

Continuum::Continuum(){	// TODO Auto-generated constructor stub
// TODO Robot name
	ros::NodeHandle nh_;
	char cableTopic[30];
nh_.param("number_of_sections",this->numberOfSegments, 3);
// this->numberOfSegments = nh_.getP("number_of_sections", 3);
int noOfSeg = this->numberOfSegments;
this->segmentLength = new double(noOfSeg);
this->segmentMode = new int(noOfSeg);
this->noOfDisks = new int(noOfSeg);
this->endEffectorPose = new tf::Transform[noOfSeg];
this->basePose = new tf::Transform[noOfSeg];
this->segKappa = new double[noOfSeg];
this->segPhi = new double[noOfSeg];
this->segTFBroadcaster = new tf::TransformBroadcaster[noOfSeg];
this->segTFFrame  = (tf::Transform**) malloc(noOfSeg * sizeof(tf::Transform*)); // https://stackoverflow.com/questions/455960/dynamic-allocating-array-of-arrays-in-c
this->arrayOfKappa = new double[(noOfSeg+1)*delay];
this->arrayOfPhi = new double[(noOfSeg+1)*delay];

this->cableMarkers = new visualization_msgs::MarkerArray[noOfSeg+1];
this->cablePublisher = new ros::Publisher[noOfSeg+1];
this->frame_timer = nh_.createTimer(ros::Duration(0.1),&Continuum::timerScanning,this);
for(int s=0; s<=noOfSeg; s++)
{
	sprintf(cableTopic, "cable_%d", s);
	cableMarkers[s].markers.resize(RESOLUTION);
	cablePublisher[s] = nh_.advertise<visualization_msgs::MarkerArray>(cableTopic,1);
}
headPublisher = nh_.advertise<visualization_msgs::MarkerArray>("headMarkers",1);
// Keyboard
  tcgetattr(0,&initial_settings);

   new_settings = initial_settings;
   new_settings.c_lflag &= ~ICANON;
   new_settings.c_lflag &= ~ECHO;
  // new_settings.c_lflag &= ~ISIG;
   new_settings.c_cc[VMIN] = 0;
   new_settings.c_cc[VTIME] = 0;
	this->hasHead = false;
this->headDisks = 0;
this->headLength = 0;
this->headKappa = 0.00001;
this->headPhi =0;
   tcsetattr(0, TCSANOW, &new_settings);
this->rateOfUpdate =0;
this->headMode = NORMAL;
}
/******************************************************/
void Continuum::timerScanning(const ros::TimerEvent&){ // for keyboard interface (Not Used~!)
/*	if(noOfDisks[0]<1) return; // to make sure the robot isinitialized
	static double thetaPitch = 0;
		static double thetaYaw = 0;
		static double targetKappa=0.00001;
		static double targetPhi=0.0;
		char childFrameName[30];

		int n = getchar();

		  if (n == '\033') { // if the first value is esc
			  getchar(); // skip the [
		      switch(getchar()) { // the real value
		          case 'A':

		              cout<<"ARROW UP"<<endl;// code for arrow up
		              thetaPitch-=90;
		              targetKappa = float(thetaPitch*PI)/(180*segmentLength[0]);
		              arrayOfKappa[0] = targetKappa;
		              thetaYaw=0;
		              targetPhi = float (thetaYaw*PI)/180;
		              arrayOfPhi[0] = targetPhi;
		              break;

		          case 'B':
		              cout<<"ARROW DOWN"<<endl;// code for down up
		              thetaPitch+=90;
		              targetKappa = float(thetaPitch*PI)/(180*segmentLength[0]);
		              arrayOfKappa[0] = targetKappa;
		              thetaYaw=0;
		              targetPhi = float (thetaYaw*PI)/180;
		              arrayOfPhi[0] = targetPhi;
		              break;

		          case 'C':
		        	  cout<<"ARROW right"<<endl;// code for arrow right
		        	  thetaPitch+=90;
		        	  targetKappa = float(thetaPitch*PI)/(180*segmentLength[0]);
		        	  arrayOfKappa[0] = targetKappa;
		        	  thetaYaw=90;
		        	  targetPhi = float (thetaYaw*PI)/180;
		              arrayOfPhi[0] = targetPhi;

		        	  break;

		          case 'D':
		        	  cout<<"ARROW left"<<endl;// code for arrow left
		        	  thetaPitch-=90;
		        	  targetKappa = float(thetaPitch*PI)/(180*segmentLength[0]);
		        	  arrayOfKappa[0] = targetKappa;
		        	  thetaYaw=90;
		        	  targetPhi = float (thetaYaw*PI)/180;
		        	  arrayOfPhi[0] = targetPhi;
		        	  break;

		      }
		      this->setHeadParameters(arrayOfKappa[0],arrayOfPhi[0],NORMAL);
		  }
// Update rotation

if(++rateOfUpdate==UPDATERATE){
	for(int i=0; i<=numberOfSegments; i++)
	{
	if(i==0){if(arrayOfKappa[0]==0)this->setHeadParameters(0,0,FLIPPED);} // SegID , Kappa, Phi
	else 	this->setSegmentShape(i-1,-arrayOfKappa[i*delay],targetPhi); // SegID , Kappa, Phi

	}
	for (int i=((numberOfSegments+1)*delay)-1;i>0;i--)
		arrayOfKappa[i] = arrayOfKappa[i-1];
	targetKappa = 0; arrayOfKappa[0] = targetKappa;thetaPitch=0;

	rateOfUpdate = 0;
}


update();
//		  this->setSegmentShape(1,0.000001,0); // SegID , Kappa, Phi

		//	basePose[0].setRotation(tf::createQuaternionFromRPY(0.0, (thetaPitch*PI)/180,0.0/*(thetaYaw*PI)/180)));
		//	basePose[0].setOrigin(basePose[0].getOrigin()+ tf::Matrix3x3(basePose[0].getRotation())*tf::Vector3(0,0,-(float(1)/140.0) * 2.0));
//
*/

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

	for (int s=segID+1;s<this->numberOfSegments;s++)

	{

		basePose[s].setOrigin(basePose[s-1].getOrigin() + (tf::Matrix3x3(basePose[s-1].getRotation())*getDiskPosition(s-1,(noOfDisks[s-1]-1))));
		basePose[s].setRotation(basePose[s-1].getRotation()*getDiskQuaternion(s-1,(noOfDisks[s-1]-1)));

	}
	}

	/******************************************************/
void Continuum::setSegmentShape(int segID, double kappa, double phi){
	if(kappa == 0) kappa = 0.0000001;
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

for (int s=segID+1;s<this->numberOfSegments;s++)

{

	basePose[s].setOrigin(endEffectorPose[s-1].getOrigin());
	basePose[s].setRotation(endEffectorPose[s-1].getRotation());
	if(s==1){
			endEffectorPose[s].setOrigin(basePose[s].getOrigin() + tf::Matrix3x3(basePose[s].getRotation())*getDiskPosition(s,(noOfDisks[s]-1)));
			endEffectorPose[s].setRotation(basePose[s].getRotation()*getDiskQuaternion(s,(noOfDisks[s]-1)));
			}//

	}
}
/******************************************************/

tf::Quaternion Continuum::getDiskQuaternion(int segID, int diskID){
tf::Matrix3x3 Rot;
tf::Quaternion qRot;
Rot.setValue(pow(cos(segPhi[segID]),2) * (cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1) + 1, sin(segPhi[segID])*cos(segPhi[segID])*( cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1), -cos(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),
						sin(segPhi[segID])*cos(segPhi[segID])*( cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1), pow(cos(segPhi[segID]),2) * ( 1 - cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])) ) + cos( segKappa[segID] * ((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),  -sin(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),
						 cos(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])),  sin(segPhi[segID])*sin(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])), cos(segKappa[segID]*((diskID/((double)noOfDisks[segID]-1))*segmentLength[segID])));
Rot.getRotation(qRot);
//endEffectorPose[segID].setRotation(basePose[segID].getRotation() * qRot);
return qRot;

}

/******************************************************/
tf::Quaternion Continuum::getHeadQuaternion(int diskID){ // Not Used~!
/*tf::Matrix3x3 Rot;
tf::Quaternion qRot;
Rot.setValue(pow(cos(headPhi),2) * (cos(headKappa*((diskID/((double)headDisks-1))*headLength)) - 1) + 1, sin(headPhi)*cos(headPhi)*( cos(headKappa*((diskID/((double)headDisks-1))*headLength)) - 1), -cos(headPhi)*sin(headKappa*((diskID/((double)headDisks-1))*headLength)),
						sin(headPhi)*cos(headPhi)*( cos(headKappa*((diskID/((double)headDisks-1))*headLength)) - 1), pow(cos(headPhi),2) * ( 1 - cos(headKappa*((diskID/((double)headDisks-1))*headLength)) ) + cos( headKappa * ((diskID/((double)headDisks-1))*headLength)),  -sin(headPhi)*sin(headKappa*((diskID/((double)headDisks-1))*headLength)),
						 cos(headPhi)*sin(headKappa*((diskID/((double)headDisks-1))*headLength)),  sin(headPhi)*sin(headKappa*((diskID/((double)headDisks-1))*headLength)), cos(headKappa*((diskID/((double)headDisks-1))*headLength)));
Rot.getRotation(qRot);
//endEffectorPose[segID].setRotation(basePose[segID].getRotation() * qRot);
return qRot;*/

}

/******************************************************/

tf::Vector3 Continuum::getDiskPosition(int segID, int i){
	tf::Vector3 eeP;
	eeP[0] = cos(segPhi[segID])*(cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
		eeP[1] = sin(segPhi[segID])*( cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
		eeP[2] = (sin(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID]))/segKappa[segID]);
		return eeP;
}
/******************************************************/
void Continuum::update(void) {
char childFrameName[30];
ros::Rate rate(15);
tf::Vector3 heeP;
/*************************************** WITH HEAD *********************************************/

/*if(this->hasHead)  // Not Used~!
{

	if(headMode == FLIPPED)
	{
		tf::Quaternion headTopQuat = tf::Quaternion(headMarkers.markers[headMarkers.markers.size()-1].pose.orientation.x,headMarkers.markers[headMarkers.markers.size()-1].pose.orientation.y,headMarkers.markers[headMarkers.markers.size()-1].pose.orientation.z,headMarkers.markers[headMarkers.markers.size()-1].pose.orientation.w);
		for(int i=0;i<this->headMarkers.markers.size();i++){
				heeP[0] = cos(headPhi)*(cos(headKappa*((i/((double)headDisks-1))*headLength)) - 1)/headKappa;
				heeP[1] = sin(headPhi)*( cos(headKappa*((i/((double)headDisks-1))*headLength)) - 1)/headKappa;
				heeP[2] = (sin(headKappa*((i/((double)headDisks-1))*headLength))/headKappa);
				heeP = tf::Matrix3x3(headTopQuat)*heeP;

				headMarkers.markers[headMarkers.markers.size()-1-i].pose.position.x = headMarkers.markers[this->headMarkers.markers.size()-1].pose.position.x + heeP.getX();
				headMarkers.markers[headMarkers.markers.size()-1-i].pose.position.y = headMarkers.markers[this->headMarkers.markers.size()-1].pose.position.y + heeP.getY();
				headMarkers.markers[headMarkers.markers.size()-1-i].pose.position.z = headMarkers.markers[this->headMarkers.markers.size()-1].pose.position.z + heeP.getZ();
				headMarkers.markers[headMarkers.markers.size()-1-i].pose.orientation.x = (headTopQuat*getHeadQuaternion(i)).x();
				headMarkers.markers[headMarkers.markers.size()-1-i].pose.orientation.y = (headTopQuat*getHeadQuaternion(i)).y();
				headMarkers.markers[headMarkers.markers.size()-1-i].pose.orientation.z = (headTopQuat*getHeadQuaternion(i)).z();
				headMarkers.markers[headMarkers.markers.size()-1-i].pose.orientation.w = (headTopQuat*getHeadQuaternion(i)).w();
		}

		for(int i=0;i<RESOLUTION&&ros::ok();i++){
				heeP[0] = cos(headPhi)*(cos(headKappa*((i/((double)RESOLUTION-1))*headLength)) - 1)/headKappa;
						heeP[1] = sin(headPhi)*( cos(headKappa*((i/((double)RESOLUTION-1))*headLength)) - 1)/headKappa;
						heeP[2] = (sin(headKappa*((i/((double)RESOLUTION-1))*headLength))/headKappa);
						heeP = tf::Matrix3x3(headTopQuat)*heeP;

				cableMarkers[this->numberOfSegments].markers[RESOLUTION-1-i].pose.position.x = headMarkers.markers[this->headMarkers.markers.size()-1].pose.position.x + heeP[0];
				cableMarkers[this->numberOfSegments].markers[RESOLUTION-1-i].pose.position.y = headMarkers.markers[this->headMarkers.markers.size()-1].pose.position.y + heeP[1];
				cableMarkers[this->numberOfSegments].markers[RESOLUTION-1-i].pose.position.z = headMarkers.markers[this->headMarkers.markers.size()-1].pose.position.z + heeP[2];

				}

		this->setSegmentBasePose(0,tf::Vector3(headMarkers.markers[0].pose.position.x,headMarkers.markers[0].pose.position.y,headMarkers.markers[0].pose.position.z),
				tf::Quaternion(headMarkers.markers[0].pose.orientation.x, headMarkers.markers[0].pose.orientation.y, headMarkers.markers[0].pose.orientation.z, headMarkers.markers[0].pose.orientation.w));

	}
	else{
	for(int i=0;i<this->headMarkers.markers.size();i++){
		heeP[0] = cos(headPhi)*(cos(headKappa*((i/((double)headDisks-1))*headLength)) - 1)/headKappa;
		heeP[1] = sin(headPhi)*( cos(headKappa*((i/((double)headDisks-1))*headLength)) - 1)/headKappa;
		heeP[2] = (sin(headKappa*((i/((double)headDisks-1))*headLength))/headKappa);
		heeP = tf::Matrix3x3(basePose[0].getRotation())*heeP;

		headMarkers.markers[i].pose.position.x = basePose[0].getOrigin().x() - heeP.getX();
		headMarkers.markers[i].pose.position.y = basePose[0].getOrigin().y() - heeP.getY();
		headMarkers.markers[i].pose.position.z = basePose[0].getOrigin().z() - heeP.getZ();
		headMarkers.markers[i].pose.orientation.x = (basePose[0].getRotation() * getHeadQuaternion(i)).x();
		headMarkers.markers[i].pose.orientation.y = (basePose[0].getRotation() * getHeadQuaternion(i)).y();
		headMarkers.markers[i].pose.orientation.z = (basePose[0].getRotation() * getHeadQuaternion(i)).z();
		headMarkers.markers[i].pose.orientation.w = (basePose[0].getRotation() * getHeadQuaternion(i)).w();
		}


	for(int i=0;i<RESOLUTION&&ros::ok();i++){
		heeP[0] = cos(headPhi)*(cos(headKappa*((i/((double)RESOLUTION-1))*headLength)) - 1)/headKappa;
				heeP[1] = sin(headPhi)*( cos(headKappa*((i/((double)RESOLUTION-1))*headLength)) - 1)/headKappa;
				heeP[2] = (sin(headKappa*((i/((double)RESOLUTION-1))*headLength))/headKappa);
				heeP = tf::Matrix3x3(basePose[0].getRotation())*heeP;

		cableMarkers[this->numberOfSegments].markers[i].pose.position.x = basePose[0].getOrigin().x()- heeP[0];
		cableMarkers[this->numberOfSegments].markers[i].pose.position.y = basePose[0].getOrigin().y()- heeP[1];
		cableMarkers[this->numberOfSegments].markers[i].pose.position.z = basePose[0].getOrigin().z()- heeP[2];

		}
	}
	headPublisher.publish(headMarkers);
	cablePublisher[this->numberOfSegments].publish(cableMarkers[this->numberOfSegments]);

}*/
/*************************************** NO HEAD *********************************************/
for (int segID = 0;segID<this->numberOfSegments;segID++)
{

	tf::Vector3 eeP;

	tf::Vector3 eePc;


	for(int i=0;i<noOfDisks[segID]&&ros::ok();i++){
	eeP[0] = cos(segPhi[segID])*(cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
	eeP[1] = sin(segPhi[segID])*( cos(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID])) - 1)/segKappa[segID];
	eeP[2] = (sin(segKappa[segID]*((i/((double)noOfDisks[segID]-1))*segmentLength[segID]))/segKappa[segID]);
	eeP =  tf::Matrix3x3(basePose[segID].getRotation())*eeP;


	this->segTFFrame[segID][i].setOrigin(tf::Vector3(basePose[segID].getOrigin().x() + eeP.getX(), basePose[segID].getOrigin().y() + eeP.getY(), basePose[segID].getOrigin().z() + eeP.getZ()) );
	this->segTFFrame[segID][i].setRotation(basePose[segID].getRotation() * getDiskQuaternion(segID,i));

	sprintf(childFrameName, "S%dL%d", segID,i);
	segTFBroadcaster[segID].sendTransform(tf::StampedTransform(segTFFrame[segID][i], ros::Time::now(),"base_link",childFrameName));
	//	slerpQuaternion = basePose[segID].getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)noOfDisks[segID]-1))));

	}


	for(int i=0;i<RESOLUTION&&ros::ok();i++){
		eePc[0] = cos(segPhi[segID])*(cos(segKappa[segID]*((i/((double)RESOLUTION-1))*segmentLength[segID])) - 1)/segKappa[segID];
		eePc[1] =  sin(segPhi[segID])*(cos(segKappa[segID]*((i/((double)RESOLUTION-1))*segmentLength[segID])) - 1)/segKappa[segID];
		eePc[2] = (sin(segKappa[segID]*((i/((double)RESOLUTION-1))*segmentLength[segID]))/segKappa[segID]);

		eePc =  tf::Matrix3x3(basePose[segID].getRotation())*eePc;
		cableMarkers[segID].markers[i].pose.position.x = basePose[segID].getOrigin().x()+ eePc[0];
				cableMarkers[segID].markers[i].pose.position.y = basePose[segID].getOrigin().y()+ eePc[1];
				cableMarkers[segID].markers[i].pose.position.z = basePose[segID].getOrigin().z()+ eePc[2];
		// Slerp for spherical interpolation
		//	slerpQuaternionCable = basePose[segID].getRotation().slerp(endEffectorPose[segID].getRotation(),(double)((i/((double)RESOLUTION-1))));
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
robotURDFfile << "<material name=\"while\">"<<endl;
robotURDFfile << "<color rgba=\"0 1 0 1\"/>"<<endl;
robotURDFfile << "</material>"<<endl;
}
else{robotURDFfile.open (path.c_str(), std::fstream::app);}

robotURDFfile<<endl;
	  for(int disk=0;disk<n_disks;disk++)
	  {
		  robotURDFfile << "<link name=\"S"<<segID<<"L"<<disk<<"\">"<<endl;
		  robotURDFfile << "<visual>"<<endl;
		  robotURDFfile << "<geometry>"<<endl;
		  if(segID==0&&disk==0)
			  robotURDFfile << "<box size=\"1 1 0.5\"/>"<<endl;
		  else
		  robotURDFfile << "<cylinder length=\"0.1\" radius=\""<<radius<<"\"/>"<<endl;
		  robotURDFfile << "<origin rpy=\"0 0 0\" xyz=\"0 0 "<<(disk/(n_disks-1))*segLength<<"\"/>"<<endl;
		  robotURDFfile << "</geometry>"<<endl;
		  if(segID==0&&disk==0)
		  			  robotURDFfile << "<material name=\"white\"/>"<<endl;
		  		 // else
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
void Continuum::addHead(double len, int disks, double rad){
	this->hasHead = true;
this->headDisks = disks;
this->headLength = len;
this->headMarkers.markers.resize(disks);
this->headPhi = 0;
this->headKappa = 0.00001;
uint32_t shape = visualization_msgs::Marker::CYLINDER;

	  for(int r=0; r<disks; r++)
	  {

	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		  headMarkers.markers[r].header.frame_id = "base_link";
		  headMarkers.markers[r].header.stamp = ros::Time::now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
		  headMarkers.markers[r].ns = "head";
		  headMarkers.markers[r].id = r;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		  headMarkers.markers[r].type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		  headMarkers.markers[r].action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
		  headMarkers.markers[r].scale.x = 2*rad;
		  headMarkers.markers[r].scale.y = 2*rad;
		  headMarkers.markers[r].scale.z = 0.1;

	    // Set the color -- be sure to set alpha to something non-zero!
		  headMarkers.markers[r].color.g = 1.0f;
		  headMarkers.markers[r].color.a = 1.0;
		  headMarkers.markers[r].lifetime = ros::Duration();
	  }
	  headMarkers.markers[disks-1].color.r = 1.0f;
	  initCableMarker(this->numberOfSegments);

}

void Continuum::setHeadParameters(double headKap, double headPhi, int MODE){
/*	if(headKap==0)headKap=0.0000001;
	this->headKappa = headKap;
	this->headMode = MODE;
	this->headPhi = headPhi;*/
}
Continuum::~Continuum() {
	// TODO Auto-generated destructor stub
  tcsetattr(0, TCSANOW, &initial_settings);

}


