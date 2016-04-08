#include "ros/ros.h"
#include "intro_services_example/service_intro.h"

bool add(intro_services_example::service_intro::Request  &req, intro_services_example::service_intro::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");

  while (ros::ok())
  {
	ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}