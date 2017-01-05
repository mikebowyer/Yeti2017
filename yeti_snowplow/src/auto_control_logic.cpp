#include "ros/ros.h"
#include "std_msgs/String.h"
#include "yeti_snowplow/robot_position"
#include <sstream>

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "auto_control_logic");  //initialize Node

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /***
  yeti_snowplow = package name, motor_speed = msg name
  ***/
  ros::Publisher moto_pub = n.advertise<yeti_snowplow::motor_speed>("motor_speed", 1000);   // topic = motor_speed
  
  ros::Rate loop_rate(10);   //loop frequency

  
  int count = 0;
  while (ros::ok())
  {
    /**
     * msg is a message object. You stuff it with data, and then publish it.
     */
    yeti_snowplow::motor_speed msg;

  
    msg.data;

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    moto_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
