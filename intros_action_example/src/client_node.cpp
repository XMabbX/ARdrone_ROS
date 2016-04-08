#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <intros_action_example/FibonacciAction.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_client");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  actionlib::SimpleActionClient<intros_action_example::FibonacciAction> ac("action_client/fibonacci",true);
  
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action client node started.");
  int count = 1;
  
  while (ros::ok())
  {
	 	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	intros_action_example::FibonacciGoal goal;
	goal.order = count;
	//reset seed and send goal
	if (count==20) count=0;
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
		std::cout << *(ac.getResult()) << std::endl;
    } else {
		ROS_INFO("Action did not finish before the time out.");
	}
    ++count;
	ros::spinOnce();
	loop_rate.sleep();
  }
  return 0;
}
