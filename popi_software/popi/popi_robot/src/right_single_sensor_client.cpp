#include <stdexcept>
#include "ros/ros.h"
#include "popi_robot/Single_sensor_read.h"
#include <cstdlib>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_single_sensor_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<popi_robot::Single_sensor_read>("right_single_sensor_server");
  popi_robot::Single_sensor_read srv;
  if (client.call(srv))
  {
    //ROS_INFO(srv.response.mesure);
    ROS_INFO("valeur: %f", (float)srv.response.mesure);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}