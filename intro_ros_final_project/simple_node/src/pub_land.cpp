#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_land");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Publisher chatter_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1000);

  ROS_INFO("-- Land publisher node");

    while (ros::ok())
    {
      std_msgs::Empty msg;

    	chatter_pub.publish(msg);
      ROS_INFO("--Land ardrone");

      ros::spinOnce();
      loop_rate.sleep();
    }

  return 0;
}
