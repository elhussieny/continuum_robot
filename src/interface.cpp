#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <termios.h>
#include <math.h>
#define PI 3.1415926

using namespace visualization_msgs;
using namespace std;
struct termios initial_settings,
               new_settings;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
tf::Quaternion directionQauat;
// %EndTag(vars)%
// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 0.2;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back( control );

  return msg.controls.back();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	  std::ostringstream s;
	ROS_INFO_STREAM( s.str() << ": pose changed"
	          << "\nposition = "
	          << feedback->pose.position.x
	          << ", " << feedback->pose.position.y
	          << ", " << feedback->pose.position.z
	          << "\norientation = "
	          << feedback->pose.orientation.w
	          << ", " << feedback->pose.orientation.x
	          << ", " << feedback->pose.orientation.y
	          << ", " << feedback->pose.orientation.z
	          << "\nframe: " << feedback->header.frame_id
	          << " time: " << feedback->header.stamp.sec << "sec, "
	          << feedback->header.stamp.nsec << " nsec" );
	//directionQauat = tf::Quaternion(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

	server->applyChanges();
}
/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
// %Tag(6DOF)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "robotHead";
  int_marker.description = "robotHead";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
 }
// %EndTag(6DOF)%
// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
	static double thetaPitch = 0;
	static double thetaYaw = 0;
  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  static tf::Transform t;

  ros::Time time = ros::Time::now();
  int n = getchar();

  if (n == '\033') { // if the first value is esc
	  getchar(); // skip the [
      switch(getchar()) { // the real value
          case 'A':
          {
              cout<<"ARROW UP"<<endl;// code for arrow up
              thetaPitch-=90;
              break;
          }
          case 'B':
              cout<<"ARROW DOWN"<<endl;// code for down up
              thetaPitch+=90;

              break;
          case 'C':
        	  cout<<"ARROW right"<<endl;// code for arrow right
        	  thetaYaw-=90;
              break;
          case 'D':
        	  cout<<"ARROW left"<<endl;// code for arrow left
        	  thetaYaw+=90;

        	  break;
      }
      directionQauat = tf::createQuaternionFromRPY(0.0, (thetaPitch*PI)/180, (thetaYaw*PI)/180);

  }
  cout<<"p = ["<<t.getOrigin().x()<<",  "<<t.getOrigin().y()<<",  "<<t.getOrigin().z()<<"]"<<endl;

  t.setRotation(tf::createQuaternionFromRPY(0.0, (thetaPitch*PI)/180, (thetaYaw*PI)/180));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));
  t.setOrigin(t.getOrigin()+ tf::Matrix3x3(directionQauat)*tf::Vector3((float(1)/140.0) * 2.0,0,0));

 }
// %EndTag(frameCallback)%
// %Tag(Moving)%
void makeMovingMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "moving_frame";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(Moving)%

int main( int argc, char** argv )
{
  ros::init(argc, argv, "interface");
  ros::NodeHandle n;
  ros::Rate r(1);
  directionQauat = tf::Quaternion(0,0,0,1);
 // Keyboard
  tcgetattr(0,&initial_settings);

   new_settings = initial_settings;
   new_settings.c_lflag &= ~ICANON;
   new_settings.c_lflag &= ~ECHO;
   new_settings.c_lflag &= ~ISIG;
   new_settings.c_cc[VMIN] = 0;
   new_settings.c_cc[VTIME] = 0;

   tcsetattr(0, TCSANOW, &new_settings);


  // create a timer to update the published transforms
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.05), frameCallback);
  ros::Publisher robotShapePublisher = n.advertise<visualization_msgs::MarkerArray>("robot_shape", 1);
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
  ros::Duration(0.1).sleep();
  tf::Vector3 position;
    position = tf::Vector3(0, 0, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, false );
    server->applyChanges();

      ros::spin();

      server.reset();
      tcsetattr(0, TCSANOW, &initial_settings);
      return 0;

}


