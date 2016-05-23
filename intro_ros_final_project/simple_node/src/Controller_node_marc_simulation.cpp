#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ar_pose/ARMarkers.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <math.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <vector>
//#include "ros/Duration.h"
#include "ros/time.h"
#include <dynamic_reconfigure/server.h>
#include <simple_node/dynparamsConfig.h>
/* Include External Files */
#include "controller.cpp"



int idmarker = 1;
int state=0; // Variabel of the state machine
int groundMarker_Found = 0;
bool found=0;
bool go_takeoff, go_land, change_cam, cont_bot, cont_front, execute;

float tsample=0.05; // Sample time
// Bottom variables
float xp_bot, yp_bot, zp;
float kp_bot, ki_bot, kd_bot; // Variables PID Cam_bottom control
float xr_bot, yr_bot, zr; // Reference bottom
float ex_bot, ey_bot, ez_bot, int_ex_bot, int_ey_bot, int_ez_bot; // errors

tf::StampedTransform transform;//No le gusta en ninguna otra parte ¬¬
// Orientacion variables
tfScalar roll_bot, yaw_bot, pitch_bot;
float rollr_bot=0, yawr_bot=0, pitchr_bot=0;
float eroll_bot, eyaw_bot, epitch_bot;
float kp_yaw=0, ki_yaw=0, kd_yaw=0;
float int_eyaw=0; // Accumulated error
float eyaw_a=0; // Error anterior

float vyaw;
float vx, vy, vz;

// Front variables
float xp_front, yp_front, zp_front;
float kp_front, ki_front, kd_front;
float xr_front, yr_front,zr_front;
float ex_front, ey_front, ez_front, int_ex_front, int_ey_front, int_ez_front;

tfScalar roll_front, pitch_front, yaw_front;
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
  execute=config.execute;

    xr_bot=config.x_ref; // posicions de referncia
    xr_front=config.x_ref; // posicions de referncia
    yr_bot=config.y_ref;
    yr_front=config.y_ref;
    zr=config.z_ref;
    yawr_bot = config.yaw_ref;

    kp_bot=config.kp;  // valors del controlador bot
    ki_bot=config.ki;
    kd_bot=config.kd;

    kp_front=config.kp;  // valors del controlador front
    ki_front=config.ki;
    kd_front=config.kd;

    kp_yaw=config.kp_yaw;
    ki_yaw=config.ki_yaw;
    kd_yaw=config.kd_yaw;
  }

void chatterCallback(const ar_pose::ARMarkers::ConstPtr& msg)
{
  ar_pose::ARMarker ar_pose_marker;
  int i;

  for(i=0;i<msg->markers.size();i++){
      ar_pose_marker = msg->markers.at(i);
      if(ar_pose_marker.id==0){
        groundMarker_Found=1;
        idmarker = ar_pose_marker.id;
        // Position respect bottom marker
        xp_bot=ar_pose_marker.pose.pose.position.x;
        yp_bot=ar_pose_marker.pose.pose.position.y;
        //zp=ar_pose_marker.pose.pose.position.z;
        // Transform quaternions to roll pitch and yaw
        tf::Quaternion q(ar_pose_marker.pose.pose.orientation.x,
      			ar_pose_marker.pose.pose.orientation.y,
      			ar_pose_marker.pose.pose.orientation.z,
      			ar_pose_marker.pose.pose.orientation.w);
      	tf::Matrix3x3 m(q);
      	m.getRPY(roll_bot,pitch_bot,yaw_bot);
        //ROS_INFO("Yaw %f",yaw_bot);
      }else if(ar_pose_marker.id==1){
        found = ar_pose_marker.id;
        xp_front=ar_pose_marker.pose.pose.position.x;
        yp_front=ar_pose_marker.pose.pose.position.y;
        zp_front=ar_pose_marker.pose.pose.position.z;
        tf::Quaternion q(ar_pose_marker.pose.pose.orientation.x,
            ar_pose_marker.pose.pose.orientation.y,
            ar_pose_marker.pose.pose.orientation.z,
            ar_pose_marker.pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        m.getRPY(roll_front, pitch_front, yaw_front);
      }

  }

  //xp=msg.markers.pose.pose.position.x;
  //yp=msg.markers.pose.pose.position.y;
  //zp=msg.pose.pose.position.z;
  //ROS_INFO("Orientacio %f %f %f",roll_bot,pitch_bot,yaw_bot);
//  ROS_INFO("Pose: %f %f %d", //This is the marker floor position respect robot
                            // cameracd
  //xp_bot,yp_bot,ar_pose_marker.id);

}

void sonarCallback(const sensor_msgs::Range msg)
{
  //zp=msg.range;
}



//void transformPoint( const tf::TransformListener& listener){

//geometry_msgs::PointStamped bot_cam_point;
//bot_cam_point.header.frame_id = "ardrone_base_bottomcam";

//bot_cam_point.header.stamp = ros::Time();

//bot_cam_point.point.x=  0;
//bot_cam_point.point.y=  0;
//bot_cam_point.point.z=  0;


//geometry_msgs::PointStamped base_point;

//listener.transformPoint("base_link", bot_cam_point, base_point);

//ROS_INFO("transformada: %f %f %f", base_point.point.x, base_point.point.y, base_point.point.z);

//}

// Funcio controlador de velocitat

/*float controller(float error, float *integral, float *errora, float fkp, float fkd, float fki)
  float velocitat;

  *integral += *integral*tsample; // integracio

  velocitat=-fkp*(error)-fki*(*integral)-fkd*(error-*errora)/tsample;

  *errora = error;

  return velocitat;
}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Controller_node_marc");
  ros::NodeHandle n;
  ros::Rate loop_rate(50); // Miliseconds
          tf::TransformListener listener;
  double temps, begin;
  bool wait;
  std_msgs::Empty msg;
  geometry_msgs::Twist cmd_msg;
  //tf::TransformListener listener(ros::Duration(10),1);
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


  //ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(listener)));

  int_ex_bot=0; //incialitza integracio
  int_ey_bot=0;
  int_ez_bot=0;

  ROS_INFO("--Controller node");


  while (ros::ok())
  {
    //Land the drone
        if(go_land)
        {
          land_pub.publish(msg);
          go_land=false;
        }
    //Take off the drone.
        if(go_takeoff)
        {
          takeoff_pub.publish(msg);
          go_takeoff=false;
        }
        if(change_cam)
        {
          camera.call(camera_srv);
          change_cam = false;
        }
    //If the parameter execute is on the drone starts to execute all teh process, if turn off we can take back the ocntorl of the drone.
    if(execute){
      switch (state) {
        case 0:
          takeoff_pub.publish(msg);
          ros::Duration(10).sleep();
          camera.call(camera_srv);
          state = 1;
        case 1:
          //Controlar i orientar i esperar 5 segons
          ex_bot=(xp_bot-xr_bot);
          ey_bot=(yp_bot-yr_bot);
          //ez_bot=(zp-zr);
          //kp_bot=0.1;
          //ki_bot=0.01;
          vx = controller(ey_bot, &int_ey_bot, &ey_a,0.07,0.0,kd_bot,tsample);
          vy = controller(ex_bot, &int_ex_bot, &ex_a,0.07,0.0,kd_bot,tsample);
          //vz = controller(ez_bot, &int_ez_bot, &ez_a,0.07,0,kd_bot,tsample);
          //eyaw_bot=(yaw_bot-yawr_bot);
          //vyaw= controller(eyaw_bot, &int_eyaw, &eyaw_a, kp_yaw, ki_yaw, kd_yaw);
          cmd_msg.linear.x=vx;
          cmd_msg.linear.y=vy;
          cmd_msg.linear.z=vz;
          //ROS_INFO("Error: %f %f %f", vx, vy, vz);
          //cmd_msg.angular.z=vyaw;
          vel_pub.publish(cmd_msg);
            if(wait==0)
            {
              ROS_INFO("Error: %f", (ex_bot+ey_bot));
              if((fabs(ex_bot+ey_bot+ez_bot)<=0.01)&&(!idmarker))
              {
              wait = 1;
              begin = ros::Time::now().toSec();
              ROS_INFO("Alligned bot marker");
              }
            }else{
              if(temps<=10){
                temps = ros::Time::now().toSec() - begin;
                ROS_INFO("Time passed %f", temps);
              }else{
                state = 2;
              }
            }
          break;
        case 2:
          cmd_msg.linear.x=0;
          cmd_msg.linear.y=0;
          cmd_msg.linear.z=0;
          vel_pub.publish(cmd_msg);
          camera.call(camera_srv);
          state=3;
          ROS_INFO("Turning");
          wait = 0;
          temps = 5;

        break;
        case 3:
          if(found==0)
          {
            cmd_msg.angular.z=0.5;
            vel_pub.publish(cmd_msg);

          }else{
            ex_front =(xp_front-yr_front);
            //ez_front =(-xp_front);
            ROS_INFO("Error: %f", ex_front);
            vyaw = controller(ex_front, &int_ex_front, &ex_a,0.1,0,0,tsample);
            //vz = controller(ez_front, &int_ez_front, &ez_a,0.2,0,0,tsample);

            //cmd_msg.linear.z=vz;
            cmd_msg.angular.z=vyaw;
            //ROS_INFO("Error %f", ex_front);
            vel_pub.publish(cmd_msg);
            if(fabs(ex_front)<=0.1){
              state = 4;
              ROS_INFO("Alligned front");
              ROS_INFO("starting aproax");
            }
          }
          break;
        case 4:
        ez_front =(yp_front-0);
        ex_front =(0.2-zp_front);
          if(fabs(ex_front)>1){
            ROS_INFO("Error: %f %f", ex_front, ez_front);
            vz = controller(ez_front, &int_ez_front, &ez_a,0.3,0,0,tsample);
            vx = controller(ex_front, &int_ex_front, &ex_a,0.2,0,0,tsample);
            cmd_msg.linear.x=vx;
            cmd_msg.linear.z=vz;
            cmd_msg.angular.z=0;
            vel_pub.publish(cmd_msg);
          }else{
            ez_front =(yp_front-0);
            ex_front =(1-zp_front);
            vz = controller(ez_front, &int_ez_front, &ez_a,0.3,0,0,tsample);
            vx = controller(ex_front, &int_ex_front, &ex_a,0.2,0,0,tsample);
            cmd_msg.linear.x=vx;
            cmd_msg.linear.z=vz;
            vel_pub.publish(cmd_msg);
            if(wait==0)
            {
              //ROS_INFO("Error: %f", (ex_bot+ey_bot+ez_bot));
              wait = 1;
              begin = ros::Time::now().toSec();
              ROS_INFO("Final wait front");
            }else{
              if(temps<=5){
                temps = ros::Time::now().toSec() - begin;
                ROS_INFO("Time passed %f", temps);
              }else{
                state = 5;
              }
            }
          }
          break;
          case 5:
          land_pub.publish(msg);
    }
    }else{
      //We have the control of the dron.
      if(cont_bot){

        //Change topic image_raw between the two cameras.
        //ex_bot=(transform.getOrigin().x()-xr_bot); // calcul del errror respecte la referencia funciona
        //ey_bot=(transform.getOrigin().y()-yr_bot);
        //ez_bot=(transform.getOrigin().z()-zr);

        ex_bot=(xp_bot-xr_bot);
        ey_bot=(yp_bot-yr_bot);
        ez_bot=(zp-zr);

        vx = controller(ey_bot, &int_ey_bot, &ey_a,kp_bot,ki_bot,kd_bot,tsample);
        vy = controller(ex_bot, &int_ex_bot, &ex_a,kp_bot,ki_bot,kd_bot,tsample);
        vz = controller(ez_bot, &int_ez_bot, &ez_a,kp_bot,ki_bot,kd_bot,tsample);

        //control orientacio

        //exo_bot=(xo_bot-xor_bot); // calcul del errror respecte la referencia funciona
        //eyaw_bot=(yaw_bot-yawr_bot);

        //ROS_INFO("Error: %f",eyaw_bot);
        //ezo_bot=(zo_bot-zor_bot);

        //vyaw= controller(eyaw_bot, &int_eyaw, &eyaw_a, kp_yaw, ki_yaw, kd_yaw);
        //vay=-kp_bot*(ex_bot)-ki_bot*(int_ex_bot)-kd_bot*(ex_bot-ex_a)/0.1;
        //vaz=-kp_bot*(ez_bot)-ki_bot*(int_ez_bot)-kd_bot*(ez_bot-ez_a)/0.1;

        cmd_msg.linear.x=vx; // Give to the drone the velocity in vx
        cmd_msg.linear.y=vy;
        //cmd_msg.linear.z=vz;
        //cmd_msg.angular.z=vyaw;
        //ROS_INFO("Velocitat: %f",eyaw_bot);
        ROS_INFO("Error: %f %f %f", ex_bot, ey_bot, ez_bot);
        ROS_INFO("Vel: %f %f %f", vx,vy,vz);
        vel_pub.publish(cmd_msg);
        // We save the previous error
      }else{

        // sino esta el control actiu porta totes les velocitats a 0
        cmd_msg.linear.x = 0;
        cmd_msg.linear.y = 0;
        cmd_msg.linear.z = 0;

        cmd_msg.angular.z=0;
        vel_pub.publish(cmd_msg);
      }

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
