#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;
using namespace std;

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
	directionQauat = tf::Quaternion(feedback->pose.orientation.x,feedback->pose.orientation.y,feedback->pose.orientation.z,feedback->pose.orientation.w);

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

  static uint32_t counter = 0;

  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

 // t.setOrigin(tf::Vector3((float(counter)/140.0) * 2.0,0,0));
  t.setOrigin(tf::Matrix3x3(directionQauat)*tf::Vector3((float(counter)/140.0) * 2.0,0,0));
  t.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

 /* t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
  br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));*/

  counter++;
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
 // initInteractiveHead();
  // create a timer to update the published transforms
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.05), frameCallback);
  ros::Publisher robotShapePublisher = n.advertise<visualization_msgs::MarkerArray>("robot_shape", 1);
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
  ros::Duration(0.1).sleep();
  tf::Vector3 position;
    position = tf::Vector3(-3, 3, 0);
    make6DofMarker( false, visualization_msgs::InteractiveMarkerControl::NONE, position, true );
    server->applyChanges();

      ros::spin();

      server.reset();
  // create an interactive marker server on the topic namespace simple_marker
 /*   interactive_markers::InteractiveMarkerServer server("robotHead");
    // create an interactive marker for our server
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = "base_link";
      int_marker.header.stamp=ros::Time::now();
      int_marker.name = "robotHead";
      int_marker.description = "Robot Head Control";

      // create a red box marker
      visualization_msgs::Marker box_marker;
      box_marker.type = visualization_msgs::Marker::CUBE;
      box_marker.scale.x = 0.45;
      box_marker.scale.y = 0.45;
      box_marker.scale.z = 0.45;
      box_marker.color.r = 1.0;
      box_marker.color.a = 1.0;

      // create a non-interactive control which contains the box
       visualization_msgs::InteractiveMarkerControl box_control;
       box_control.always_visible = true;
       box_control.markers.push_back(box_marker);
       // add the control to the interactive marker
         int_marker.controls.push_back( box_control );

         // create a control which will move the box
         // this control does not contain any markers,
         // which will cause RViz to insert two arrows
         visualization_msgs::InteractiveMarkerControl rotate_control;
         rotate_control.name = "move_x";
         rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

         // add the control to the interactive marker
         int_marker.controls.push_back(rotate_control);

         // add the interactive marker to our collection &
         // tell the server to call processFeedback() when feedback arrives for it
         server.insert(int_marker, &processFeedback);

         // 'commit' changes and send to all clients
         server.applyChanges();

         // start the ROS main loop
         ros::spin();*/














    // Set our initial shape type to be a cube
  //uint32_t shape = visualization_msgs::Marker::CUBE;

 /* while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }*/
}


