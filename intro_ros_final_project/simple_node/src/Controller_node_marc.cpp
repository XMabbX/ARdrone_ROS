#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ar_pose/ARMarker.h"
#include <geometry_msgs/Twist.h>


#include <dynamic_reconfigure/server.h>
#include <simple_node/dynparamsConfig.h>

bool go_takeoff, go_land, change_cam,cont_switch;

float xp, yp, zp, kp, ki, vx, vy, vz, xr, yr, zr, ex, ey, ez, int_ex,int_ey,int_ez;

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

cont_switch=config.cont_switch;

if(cont_switch){
  xr=config.x_ref; // posicions de referncia
  yr=config.y_ref;
  zr=config.z_ref;
  kp=config.kp;  // valors del controlador
  ki=config.ki;


}


}

void chatterCallback(const ar_pose::ARMarker msg)
{

  xp=msg.pose.pose.position.x;
  yp=msg.pose.pose.position.y;
  zp=msg.pose.pose.position.z;

  ROS_INFO("Pose: %f %f %f", //This is the marker floor position respect robot
                            // cameracd
  xp,yp,zp);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Controller_node_marc");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
  ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

  dynamic_reconfigure::Server<simple_node::dynparamsConfig> server;
  dynamic_reconfigure::Server<simple_node::dynparamsConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::ServiceClient camera = n.serviceClient <std_srvs::Empty> ("/ardrone/togglecam");
  std_srvs::Empty camera_srv;

  ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, chatterCallback);
  int_ex=0;
  int_ey=0;
  int_ez=0;


  ROS_INFO("--Controller node");


  while (ros::ok())
  {
    std_msgs::Empty msg;
    geometry_msgs::Twist cmd_msg;

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

    if(cont_switch){

      ex=(xp-xr); // calcul del errror respecte la referencia funciona
      ey=(yp-yr);
      ez=(zp-zr);

      int_ex += ex*0.1; // integracio
      int_ey += ey*0.1;
      int_ez += ez*0.1;

      vx=-kp*(ey)-ki*(int_ey); // integracio no funciona be
      vy=-kp*(ex)-ki*(-int_ex);
      vz=-kp*(ez)-ki*(int_ez);

      cmd_msg.linear.x = vx; // preparar missatge de les velocitats
      cmd_msg.linear.y = vy;
      cmd_msg.linear.z = vz;


      ROS_INFO("Error: %f %f %f", ex, ey, ez);
      ROS_INFO("Vel: %f %f %f", vx,vy,vz);
      vel_pub.publish(cmd_msg);


    }else{
      if(cmd_msg.linear.x!=0)  // sino esta el control actiu porta totes les velocitats a 0
      cmd_msg.linear.x = 0;
      cmd_msg.linear.y = 0;
      cmd_msg.linear.z = 0;
      vel_pub.publish(cmd_msg);
    }


    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
