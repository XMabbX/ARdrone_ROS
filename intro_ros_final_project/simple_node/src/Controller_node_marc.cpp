#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ar_pose/ARMarkers.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <vector>



#include <dynamic_reconfigure/server.h>
#include <simple_node/dynparamsConfig.h>

bool go_takeoff, go_land, change_cam, cont_bot, cont_front;

float xp_bot, yp_bot, zp;
float kp_bot, ki_bot, kd_bot;
float xr_bot, yr_bot, zr;
float ex_bot, ey_bot, ez_bot, int_ex_bot, int_ey_bot, int_ez_bot;

float vx, vy, vz;

float xp_front, yp_front;
float kp_front, ki_front, kd_front;
float xr_front, yr_front;
float ex_front, ey_front, ez_front, int_ex_front, int_ey_front, int_ez_front;

float ex_a=0,ey_a=0,ez_a=0;

float sonar_h;

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

  cont_bot=config.cont_bot;
  cont_front=config.cont_front;

  if(cont_bot||cont_front){
    xr_bot=config.x_ref; // posicions de referncia
    xr_front=config.x_ref; // posicions de referncia
    yr_bot=config.y_ref;
    yr_front=config.y_ref;
    zr=config.z_ref;
    kp_bot=config.kp;  // valors del controlador bot
    ki_bot=config.ki;
    kd_bot=config.kd;

    kp_front=config.kp;  // valors del controlador front
    ki_front=config.ki;
    kd_front=config.kd;


  }


  }

void chatterCallback(const ar_pose::ARMarkers::ConstPtr& msg)
{
  ar_pose::ARMarker ar_pose_marker;
  int i;

  for(i=0;i<msg->markers.size();i++){
      ar_pose_marker = msg->markers.at(i);
      if(ar_pose_marker.id==0){
        xp_bot=ar_pose_marker.pose.pose.position.x;
        yp_bot=ar_pose_marker.pose.pose.position.y;
      }else if(ar_pose_marker.id==1){
        xp_front=ar_pose_marker.pose.pose.position.x;
        yp_front=ar_pose_marker.pose.pose.position.y;
      }

  }

  //xp=msg.markers.pose.pose.position.x;
  //yp=msg.markers.pose.pose.position.y;
  //zp=msg.pose.pose.position.z;

  ROS_INFO("Pose: %f %f %d", //This is the marker floor position respect robot
                            // cameracd
  xp_bot,yp_bot,ar_pose_marker.id);

}

void sonarCallback(const sensor_msgs::Range msg)
{
  zp=msg.range;
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
  ros::Subscriber sonar_sub = n.subscribe("/sonar_height", 1000, sonarCallback);


  int_ex_bot=0; //incialitza integracio
  int_ey_bot=0;
  int_ez_bot=0;


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

    if(cont_bot){

      ex_bot=(xp_bot-xr_bot); // calcul del errror respecte la referencia funciona
      ey_bot=(yp_bot-yr_bot);
      ez_bot=(zp-zr);

      int_ex_bot += ex_bot*0.1; // integracio
      int_ey_bot += ey_bot*0.1;
      int_ez_bot += ez_bot*0.1;

      vx=-kp_bot*(ey_bot)-ki_bot*(int_ey_bot)-kd_bot*(ey_bot-ey_a)/0.1; // integracio no funciona be
      vy=-kp_bot*(ex_bot)-ki_bot*(int_ex_bot)-kd_bot*(ex_bot-ex_a)/0.1;
      vz=-kp_bot*(ez_bot)-ki_bot*(int_ez_bot)-kd_bot*(ez_bot-ez_a)/0.1;

      cmd_msg.linear.x = vx; // preparar missatge de les velocitats
      cmd_msg.linear.y = vy;
      cmd_msg.linear.z = vz;


      ROS_INFO("Error: %f %f %f", ex_bot, ey_bot, ez_bot);
      //ROS_INFO("Vel: %f %f %f", vx,vy,vz);
      vel_pub.publish(cmd_msg);
      // We save the previous error
      ex_a = ex_bot;
      ey_a = ey_bot;
      ez_a = ez_bot;

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
