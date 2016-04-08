#include "ros/ros.h"
#include "ar_pose/ARMarker.h"
#include <dynamic_reconfigure/server.h>
#include <my_node/my_nodeparamsConfig.h>

double xd,yd,zd,xg,yg,zg;

void chatterCallback(const ar_pose::ARMarker msg)
{
	xg = msg.pose.pose.position.x;
	yg = msg.pose.pose.position.y;
	zg = msg.pose.pose.position.z;
}

void callback(my_node::my_nodeparamsConfig &config, uint32_t level) {
ROS_INFO("Reconfigure Request: %f %f %f",
config.x,config.y,config.z);
}

int main(int argc, char **argv)
{
float ex,ey,ez,e;
ros::init(argc, argv, "my_node");
ros::NodeHandle n;
ros::Rate loop_rate(10);
ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, chatterCallback);

dynamic_reconfigure::Server<my_node::my_nodeparamsConfig> server;
dynamic_reconfigure::Server<my_node::my_nodeparamsConfig>::CallbackType f;

f = boost::bind(&callback, _1, _2);
server.setCallback(f);

while (ros::ok())
{

ros::spinOnce();
loop_rate.sleep();


ex = xd-xg;
ey = yd-yg;
ez = zd-zg;
e = sqrt(pow(ex,2)+pow(ey,2)+pow(ez,2));

ROS_INFO("Error: %f", e);

}
return 0;
}
