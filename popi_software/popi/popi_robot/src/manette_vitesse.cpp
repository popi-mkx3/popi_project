#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include "popi_robot/CmdPosArray.h"

class Manette_vitesse
{
public:
  Manette_vitesse();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  int linear_,LT_, RT_, A_, B_, Y_, LB_, RB_, RJB_;
  double l_scale_;
  bool flag_avd, flag_avg, flag_ard, flag_arg;
  popi_robot::CmdPosArray cmd;
  ros::Publisher pub;
  ros::Subscriber joy_sub_;
  ros::ServiceClient client_avd;
  ros::ServiceClient client_ard;
  ros::ServiceClient client_avg;
  ros::ServiceClient client_arg;
  std_srvs::Empty srv;
};


Manette_vitesse::Manette_vitesse():
  linear_(1),
  LT_(2),
  RT_(3),
  A_(1),
  B_(2),
  Y_(3),
  LB_(4),
  RB_(5),
  RJB_(10)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("left_trigger", LT_, LT_);
  nh_.param("right_trigger", RT_, RT_);
  nh_.param("button_A", A_, A_);
  nh_.param("button_B", B_, B_);
  nh_.param("button_Y", Y_, Y_);
  nh_.param("button_LB", LB_, LB_);
  nh_.param("button_RB", RB_, RB_);
  nh_.param("button_right_joystick", RJB_, RJB_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  pub = nh_.advertise<popi_robot::CmdPosArray>("cmd_mot", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Manette_vitesse::joyCallback, this);
  client_avd = nh_.serviceClient<std_srvs::Empty>("right_init_epaule_av_server");
  client_ard = nh_.serviceClient<std_srvs::Empty>("right_init_epaule_ar_server");
  client_avg = nh_.serviceClient<std_srvs::Empty>("left_init_epaule_av_server");
  client_arg = nh_.serviceClient<std_srvs::Empty>("left_init_epaule_ar_server");
  flag_avd = true;
  flag_avg = true;
  flag_ard = true;
  flag_arg = true;
}


void Manette_vitesse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  cmd.aile_avd = 0.0;
  cmd.aile_avg = 0.0;
  cmd.aile_ard = 0.0;
  cmd.aile_arg = 0.0;

  cmd.epaule_avd = 0.0;
  cmd.epaule_avg = 0.0;
  cmd.epaule_ard = 0.0;
  cmd.epaule_arg = 0.0;

  cmd.coude_avd = 0.0;
  cmd.coude_avg = 0.0;
  cmd.coude_ard = 0.0;
  cmd.coude_arg = 0.0;
  
  // set 0 epaule AVD 
  if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==1 &&
    joy->buttons[RJB_]==1 &&
    flag_avd
    )
  {
    if (client_avd.call(srv))
    {
      flag_avd = false;
      ROS_INFO("epaule avd initialisee");
    }
    else ROS_INFO("probleme d'initialisation epaule AVD");
  }
  
  // set 0 epaule AVG 
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==1 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[RJB_]==1 &&
    flag_avg
    )
  {
    if (client_avg.call(srv))
    {
      flag_avg = false;
      ROS_INFO("epaule avg initialisee");
    }
    else ROS_INFO("probleme d'initialisation epaule AVG");
  }
  
  // set 0 epaule ARD 
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] < 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[RJB_]==1 &&
    flag_ard
    )
  {
    if (client_ard.call(srv))
    {
      flag_ard = false;
      ROS_INFO("epaule ard initialisee");
    }
    else ROS_INFO("probleme d'initialisation epaule ARD");
  }
  
  // set 0 epaule ARG 
  else if (joy->axes[LT_] < 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[RJB_]==1 &&
    flag_arg
    )
  {
    if (client_arg.call(srv))
    {
      flag_arg = false;
      ROS_INFO("epaule arg initialisee");
    }
    else ROS_INFO("probleme d'initialisation epaule ARG");
  }
  
  //aile AVD
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==1
    )
  {
    cmd.aile_avd = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
  }
  //aile AVG
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==1 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.aile_avg = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
  }
  //aile ARD
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] < 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.aile_ard = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
  }
  //aile ARG
  else if (joy->axes[LT_] < 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.aile_arg = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
  }
  //epaule AVD
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==1 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==1
    )
  {
    cmd.epaule_avd = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
    flag_avd = true;
  }
  //epaule AVG
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==1 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==1 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.epaule_avg = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
    flag_avg = true;
  }
  //epaule ARD
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_]< 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==1 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.epaule_ard = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
    flag_ard = true;
  }
  //epaule ARG
  else if (joy->axes[LT_]< 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==1 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.epaule_arg = l_scale_*joy->axes[linear_];
    pub.publish(cmd);
    flag_arg = true;
  }
  //coude AVD
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==1 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==1
    )
  {
    cmd.coude_avd = 100*joy->axes[linear_];
    pub.publish(cmd);
  }
  //coude AVG
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==1 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==1 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.coude_avg = 100*joy->axes[linear_];
    pub.publish(cmd);
  }
  //coude ARD
  else if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] < 0 &&
    joy->buttons[A_]==1 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.coude_ard = 100*joy->axes[linear_];
    pub.publish(cmd);
  }
  //coude ARG
  else if (joy->axes[LT_] < 0 &&
    joy->axes[RT_]> 0 &&
    joy->buttons[A_]==1 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0
    )
  {
    cmd.coude_arg = 100*joy->axes[linear_];
    pub.publish(cmd);
  }
  else pub.publish(cmd);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "manette");
  Manette_vitesse manette;

  ros::spin();
}
