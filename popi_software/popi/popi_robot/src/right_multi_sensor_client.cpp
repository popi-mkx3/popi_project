#include <stdexcept>
#include "ros/ros.h"
#include "popi_robot/Multi_sensor_read.h"
#include "popi_robot/Float64Array.h"
#include <cstdlib>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "right_multi_sensor_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<popi_robot::Multi_sensor_read>("right_multi_sensor_server");
  popi_robot::Multi_sensor_read srv;
  if (client.call(srv))
  {
    //ROS_INFO(srv.response.mesure.aile_av);
    ROS_INFO("valeur aile av : %f", (float)srv.response.mesure.aile_av);
    ROS_INFO("valeur aile ar : %f", (float)srv.response.mesure.aile_ar);
    ROS_INFO("valeur epaule av : %f", (float)srv.response.mesure.epaule_av);
    ROS_INFO("valeur epaule ar : %f", (float)srv.response.mesure.epaule_ar);
    ROS_INFO("valeur coude av : %f", (float)srv.response.mesure.aile_av);
    ROS_INFO("valeur coude ar : %f", (float)srv.response.mesure.coude_ar);
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  return 0;
}