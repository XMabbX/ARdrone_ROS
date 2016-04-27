#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ar_pose/ARMarker.h"
#include "geometry_msgs/Twist.h"


#include <dynamic_reconfigure/server.h>
#include <simple_node/dynparamsConfig.h>

bool go_takeoff, go_land, change_cam;
float xp, yp, zp, kp=2;

float ex, ey, ez;

void callback(simple_node::dynparamsConfig &config, uint32_t level) {

go_takeoff = config.takeoff;
if(go_takeoff)
config.takeoff=false;

go_land = config.land;
if(go_land)
config.land=false;

change_cam=config.toggle_cam;
if(change_cam)
config.toggle_cam=false;

}

void chatterCallback(const ar_pose::ARMarker msg)
{
  xp=msg.pose.pose.position.x;
  yp=msg.pose.pose.position.y;
  zp=msg.pose.pose.position.z;

  ROS_INFO("Pose: %f %f %f", //This is the marker floor position respect robot
<<<<<<< HEAD
                  xp,yp,zp); // cameracd
=======
                            // cameracd
	msg.pose.pose.position.x,
  msg.pose.pose.position.y,
  msg.pose.pose.position.z);

>>>>>>> 2cfd372c719f65d5aa582090603bd3232407aeca
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Controller_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
  ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
  ros::Publisher velocity = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
  dynamic_reconfigure::Server<simple_node::dynparamsConfig> server;
  dynamic_reconfigure::Server<simple_node::dynparamsConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::ServiceClient camera = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam");
  std_srvs::Empty camera_srv;

  ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, chatterCallback);

  ROS_INFO("--Controller node");


  while (ros::ok())
  {
    std_msgs::Empty msg;
    geometry_msgs::Twist twist_msg; //Declare velocity variable
//Take off the drone.
    if(go_takeoff)
    {
      takeoff_pub.publish(msg);
      go_takeoff=false;
    }
//Land the drone
    if(go_land)
    {
      land_pub.publish(msg);
      go_land=false;
    }
//Change topic image_raw between the two cameras.
    if(change_cam)
    {
      camera.call(camera_srv);
      change_cam = false;
    }
// Control position of mark1
    twist_msg.linear.x=-kp*xp;
    twist_msg.linear.y=-kp*yp;
    velocity.publish(twist_msg);
    ROS_INFO("Velocity: %f %f",-kp*xp,-kp*yp);
    ros::spinOnce();
    loop_rate.sleep();


  }
  return 0;
}
