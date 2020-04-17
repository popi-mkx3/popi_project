#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <popi_robot/Multi_sensor_read.h>

class Manette
{
public:
  Manette();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void init();
  float limite_aile(float value);
  float limite_epaule(float value);
  float limite_coude(float value);
  float limite_aile_haute = 0.4;
  float limite_aile_basse = -0.4;
  float limite_epaule_haute = 3.14;
  float limite_epaule_basse = 0;
  float limite_coude_haute = 2.2;
  float limite_coude_basse = 0.2;
  ros::NodeHandle nh_;

  int linear_,LT_, RT_, A_, B_, Y_, LB_, RB_;
  double l_scale_;
  bool simu;

  std_msgs::Float64 cmd_aile_avd, cmd_aile_avg, cmd_aile_ard, cmd_aile_arg;
  std_msgs::Float64 cmd_epaule_avd, cmd_epaule_avg, cmd_epaule_ard, cmd_epaule_arg;
  std_msgs::Float64 cmd_coude_avd, cmd_coude_avg, cmd_coude_ard, cmd_coude_arg;

  ros::Publisher pub_aile_avd, pub_aile_avg, pub_aile_ard, pub_aile_arg;
  ros::Publisher pub_epaule_avd, pub_epaule_avg, pub_epaule_ard, pub_epaule_arg;
  ros::Publisher pub_coude_avd, pub_coude_avg, pub_coude_ard, pub_coude_arg;

  ros::Subscriber joy_sub_;
  ros::ServiceClient right_client;
  ros::ServiceClient left_client;
  popi_robot::Multi_sensor_read srv;
};


Manette::Manette():
  linear_(1),
  LT_(2),
  RT_(3),
  A_(1),
  B_(2),
  Y_(3),
  LB_(4),
  RB_(5),
  simu(false)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("left_trigger", LT_, LT_);
  nh_.param("right_trigger", RT_, RT_);
  nh_.param("button_A", A_, A_);
  nh_.param("button_B", B_, B_);
  nh_.param("button_Y", Y_, Y_);
  nh_.param("button_LB", LB_, LB_);
  nh_.param("button_RB", RB_, RB_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("simu", simu, simu);


  pub_aile_avd = nh_.advertise<std_msgs::Float64>("/popi/aileAVD_eff_position_controller/command", 1);
  pub_aile_avg = nh_.advertise<std_msgs::Float64>("/popi/aileAVG_eff_position_controller/command", 1);
  pub_aile_ard = nh_.advertise<std_msgs::Float64>("/popi/aileARD_eff_position_controller/command", 1);
  pub_aile_arg = nh_.advertise<std_msgs::Float64>("/popi/aileARG_eff_position_controller/command", 1);

  pub_epaule_avd = nh_.advertise<std_msgs::Float64>("/popi/epauleAVD_eff_position_controller/command", 1);
  pub_epaule_avg = nh_.advertise<std_msgs::Float64>("/popi/epauleAVG_eff_position_controller/command", 1);
  pub_epaule_ard = nh_.advertise<std_msgs::Float64>("/popi/epauleARD_eff_position_controller/command", 1);
  pub_epaule_arg = nh_.advertise<std_msgs::Float64>("/popi/epauleARG_eff_position_controller/command", 1);

  pub_coude_avd = nh_.advertise<std_msgs::Float64>("/popi/coudeAVD_eff_position_controller/command", 1);
  pub_coude_avg = nh_.advertise<std_msgs::Float64>("/popi/coudeAVG_eff_position_controller/command", 1);
  pub_coude_ard = nh_.advertise<std_msgs::Float64>("/popi/coudeARD_eff_position_controller/command", 1);
  pub_coude_arg = nh_.advertise<std_msgs::Float64>("/popi/coudeARG_eff_position_controller/command", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Manette::joyCallback, this);
  right_client = nh_.serviceClient<popi_robot::Multi_sensor_read>("/popi/right_multi_sensor_server");
  left_client = nh_.serviceClient<popi_robot::Multi_sensor_read>("/popi/left_multi_sensor_server");

  if (simu == false)
    init();
}
float Manette::limite_aile(float value)
{
  if (value > limite_aile_haute)
    return limite_aile_haute;
  else if (limite_aile_basse > value)
    return limite_aile_basse;
  else
    return value;
}
float Manette::limite_epaule(float value)
{
  if (value > limite_epaule_haute)
    return limite_epaule_haute;
  else if (limite_epaule_basse > value)
    return limite_epaule_basse;
  else
    return value;
}
float Manette::limite_coude(float value)
{
  if (value > limite_coude_haute)
    return limite_coude_haute;
  else if (limite_coude_basse > value)
    return limite_coude_basse;
  else
    return value;
}
void Manette::init()
{
  ROS_INFO("waiting for sensor services");
  ros::service::waitForService("/popi/right_multi_sensor_server", -1);
  ros::service::waitForService("/popi/left_multi_sensor_server", -1);
  
  if (right_client.call(srv))
  {
    cmd_aile_avd.data = limite_aile(srv.response.mesure.aile_av);
    pub_aile_avd.publish(cmd_aile_avd);
    cmd_aile_ard.data = limite_aile(srv.response.mesure.aile_ar);
    pub_aile_ard.publish(cmd_aile_ard);
    cmd_epaule_avd.data = limite_epaule(srv.response.mesure.epaule_av);
    pub_epaule_avd.publish(cmd_epaule_avd);
    cmd_epaule_ard.data = limite_epaule(srv.response.mesure.epaule_ar);
    pub_epaule_ard.publish(cmd_epaule_ard);
    cmd_coude_avd.data = limite_coude(srv.response.mesure.coude_av);
    pub_coude_avd.publish(cmd_coude_avd);
    cmd_coude_ard.data = limite_coude(srv.response.mesure.coude_ar);
    pub_coude_ard.publish(cmd_coude_ard);
  }
  else
  {
    ROS_ERROR("Failed to call service right_multi_multi_sensor_server");
  }
    if (right_client.call(srv))
  {
    cmd_aile_avg.data = limite_aile(srv.response.mesure.aile_av);
    pub_aile_avg.publish(cmd_aile_avg);
    cmd_aile_arg.data = limite_aile(srv.response.mesure.aile_ar);
    pub_aile_arg.publish(cmd_aile_arg);
    cmd_epaule_avg.data = limite_epaule(srv.response.mesure.epaule_av);
    pub_epaule_avg.publish(cmd_epaule_avg);
    cmd_epaule_arg.data = limite_epaule(srv.response.mesure.epaule_ar);
    pub_epaule_arg.publish(cmd_epaule_arg);
    cmd_coude_avg.data = limite_coude(srv.response.mesure.coude_av);
    pub_coude_avg.publish(cmd_coude_avg);
    cmd_coude_arg.data = limite_coude(srv.response.mesure.coude_ar);
    pub_coude_arg.publish(cmd_coude_arg);
  }
  else
  {
    ROS_ERROR("Failed to call service left_multi_sensor_server");
  }

  ROS_INFO("communication beagles ok");
}

void Manette::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//aile AVD
  if (joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==1
    )
  {
    cmd_aile_avd.data = limite_aile (cmd_aile_avd.data + l_scale_*joy->axes[linear_]);
    pub_aile_avd.publish(cmd_aile_avd);
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
    cmd_aile_avg.data = limite_aile (cmd_aile_avg.data + l_scale_*joy->axes[linear_]);
    pub_aile_avg.publish(cmd_aile_avg);
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
    cmd_aile_ard.data = limite_aile (cmd_aile_ard.data + l_scale_*joy->axes[linear_]);
    pub_aile_ard.publish(cmd_aile_ard);
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
    cmd_aile_arg.data = limite_aile (cmd_aile_arg.data + l_scale_*joy->axes[linear_]);
    pub_aile_arg.publish(cmd_aile_arg);
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
    cmd_epaule_avd.data = limite_epaule (cmd_epaule_avd.data + l_scale_*joy->axes[linear_]);
    pub_epaule_avd.publish(cmd_epaule_avd);
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
    cmd_epaule_avg.data = limite_epaule (cmd_epaule_avg.data + l_scale_*joy->axes[linear_]);
    pub_epaule_avg.publish(cmd_epaule_avg);
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
    cmd_epaule_ard.data = limite_epaule (cmd_epaule_ard.data + l_scale_*joy->axes[linear_]);
    pub_epaule_ard.publish(cmd_epaule_ard);
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
    cmd_epaule_arg.data = limite_epaule (cmd_epaule_arg.data + l_scale_*joy->axes[linear_]);
    pub_epaule_arg.publish(cmd_epaule_arg);
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
    cmd_coude_avd.data = limite_coude (cmd_coude_avd.data + l_scale_*joy->axes[linear_]);
    pub_coude_avd.publish(cmd_coude_avd);
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
    cmd_coude_avg.data = limite_coude (cmd_coude_avg.data + l_scale_*joy->axes[linear_]);
    pub_coude_avg.publish(cmd_coude_avg);
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
    cmd_coude_ard.data = limite_coude (cmd_coude_ard.data + l_scale_*joy->axes[linear_]);
    pub_coude_ard.publish(cmd_coude_ard);
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
    cmd_coude_arg.data = limite_coude (cmd_coude_arg.data + l_scale_*joy->axes[linear_]);
    pub_coude_arg.publish(cmd_coude_arg);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "manette");
  Manette manette;

  ros::spin();
}
