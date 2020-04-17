#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/Config.h>
#include <sensor_msgs/Joy.h>
#include <popi_robot/Potards.h>
#include <popi_robot/RouesCodeuses.h>

class Manette
{
public:
  Manette();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void rightCallback(const popi_robot::Potards::ConstPtr& mes);
  void leftCallback(const popi_robot::Potards::ConstPtr& mes);
  void rightEpCallback(const popi_robot::RouesCodeuses::ConstPtr& mes);
  void leftEpCallback(const popi_robot::RouesCodeuses::ConstPtr& mes);

  void init();
  void controllersOn();
  void controllersOff();
  float limite_aile(float value);
  float limite_epaule(float value);
  float limite_coude(float value);
  void moveAllPosition(double* targetPos, double nb_boucles);
  void stand();
  void sit();
  void reversesit();
  void down();
  void twerk();
  void left();
  void right();
  float limite_aile_haute = 0.5;
  float limite_aile_basse = -0.5;
  float limite_epaule_haute = 3.14;
  float limite_epaule_basse = 0;
  float limite_coude_haute = 1.92;
  float limite_coude_basse = 0.26;

  ros::NodeHandle nh_;

  int linear_, RT_, LT_, A_, B_, X_, Y_, LB_, RB_, start_;
  double l_scale_;
  bool simu;

  std_msgs::Float64 cmd_aile_avd, cmd_aile_avg, cmd_aile_ard, cmd_aile_arg;
  std_msgs::Float64 cmd_epaule_avd, cmd_epaule_avg, cmd_epaule_ard, cmd_epaule_arg;
  std_msgs::Float64 cmd_coude_avd, cmd_coude_avg, cmd_coude_ard, cmd_coude_arg;

  ros::Publisher pub_cmd_aile_avd, pub_cmd_aile_avg, pub_cmd_aile_ard, pub_cmd_aile_arg;
  ros::Publisher pub_cmd_epaule_avd, pub_cmd_epaule_avg, pub_cmd_epaule_ard, pub_cmd_epaule_arg;
  ros::Publisher pub_cmd_coude_avd, pub_cmd_coude_avg, pub_cmd_coude_ard, pub_cmd_coude_arg;


  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config pid_off;
  dynamic_reconfigure::Config pid_aile_av, pid_aile_ar;
  dynamic_reconfigure::Config pid_epaule_av, pid_epaule_ar;
  dynamic_reconfigure::Config pid_coude_av, pid_coude_ar;

  ros::Publisher pub_pid_aile_avd, pub_pid_aile_avg, pub_pid_aile_ard, pub_pid_aile_arg;
  ros::Publisher pub_pid_epaule_avd, pub_pid_epaule_avg, pub_pid_epaule_ard, pub_pid_epaule_arg;
  ros::Publisher pub_pid_coude_avd, pub_pid_coude_avg, pub_pid_coude_ard, pub_pid_coude_arg;

  ros::Subscriber joy_sub_;
  ros::Subscriber right_sub;
  ros::Subscriber left_sub;
  ros::Subscriber right_ep_sub;
  ros::Subscriber left_ep_sub;

};


Manette::Manette():
  linear_(1),
  LT_(2),
  RT_(5),
  A_(0),
  B_(1),
  X_(2),
  Y_(3),
  LB_(4),
  RB_(5),
  start_(7),
  simu(false)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("left_trigger", LT_, LT_);
  nh_.param("right_trigger", RT_, RT_);
  nh_.param("button_A", A_, A_);
  nh_.param("button_B", B_, B_);
  nh_.param("button_X", X_, X_);
  nh_.param("button_Y", Y_, Y_);
  nh_.param("button_LB", LB_, LB_);
  nh_.param("button_RB", RB_, RB_);
  nh_.param("button_start", start_, start_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("simu", simu, simu);


  pub_cmd_aile_avd = nh_.advertise<std_msgs::Float64>("/popi/aileAVD_eff_position_controller/command", 1);
  pub_cmd_aile_avg = nh_.advertise<std_msgs::Float64>("/popi/aileAVG_eff_position_controller/command", 1);
  pub_cmd_aile_ard = nh_.advertise<std_msgs::Float64>("/popi/aileARD_eff_position_controller/command", 1);
  pub_cmd_aile_arg = nh_.advertise<std_msgs::Float64>("/popi/aileARG_eff_position_controller/command", 1);

  pub_cmd_epaule_avd = nh_.advertise<std_msgs::Float64>("/popi/epauleAVD_eff_position_controller/command", 1);
  pub_cmd_epaule_avg = nh_.advertise<std_msgs::Float64>("/popi/epauleAVG_eff_position_controller/command", 1);
  pub_cmd_epaule_ard = nh_.advertise<std_msgs::Float64>("/popi/epauleARD_eff_position_controller/command", 1);
  pub_cmd_epaule_arg = nh_.advertise<std_msgs::Float64>("/popi/epauleARG_eff_position_controller/command", 1);

  pub_cmd_coude_avd = nh_.advertise<std_msgs::Float64>("/popi/coudeAVD_eff_position_controller/command", 1);
  pub_cmd_coude_avg = nh_.advertise<std_msgs::Float64>("/popi/coudeAVG_eff_position_controller/command", 1);
  pub_cmd_coude_ard = nh_.advertise<std_msgs::Float64>("/popi/coudeARD_eff_position_controller/command", 1);
  pub_cmd_coude_arg = nh_.advertise<std_msgs::Float64>("/popi/coudeARG_eff_position_controller/command", 1);


  pub_pid_aile_avd = nh_.advertise<dynamic_reconfigure::Config>("/popi/aileAVD_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_aile_avg = nh_.advertise<dynamic_reconfigure::Config>("/popi/aileAVG_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_aile_ard = nh_.advertise<dynamic_reconfigure::Config>("/popi/aileARD_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_aile_arg = nh_.advertise<dynamic_reconfigure::Config>("/popi/aileARG_eff_position_controller/pid/parameter_updates", 1);

  pub_pid_epaule_avd = nh_.advertise<dynamic_reconfigure::Config>("/popi/epauleAVD_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_epaule_avg = nh_.advertise<dynamic_reconfigure::Config>("/popi/epauleAVG_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_epaule_ard = nh_.advertise<dynamic_reconfigure::Config>("/popi/epauleARD_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_epaule_arg = nh_.advertise<dynamic_reconfigure::Config>("/popi/epauleARG_eff_position_controller/pid/parameter_updates", 1);

  pub_pid_coude_avd = nh_.advertise<dynamic_reconfigure::Config>("/popi/coudeAVD_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_coude_avg = nh_.advertise<dynamic_reconfigure::Config>("/popi/coudeAVG_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_coude_ard = nh_.advertise<dynamic_reconfigure::Config>("/popi/coudeARD_eff_position_controller/pid/parameter_updates", 1);
  pub_pid_coude_arg = nh_.advertise<dynamic_reconfigure::Config>("/popi/coudeARG_eff_position_controller/pid/parameter_updates", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Manette::joyCallback, this);
  right_sub = nh_.subscribe<popi_robot::Potards>("right_multi_potards_publisher", 1, &Manette::rightCallback, this);
  left_sub = nh_.subscribe<popi_robot::Potards>("left_multi_potards_publisher", 1, &Manette::leftCallback, this);
  right_ep_sub = nh_.subscribe<popi_robot::RouesCodeuses>("right_multi_roues_codeuses_publisher", 1, &Manette::rightEpCallback, this);
  left_ep_sub = nh_.subscribe<popi_robot::RouesCodeuses>("left_multi_roues_codeuses_publisher", 1, &Manette::leftEpCallback, this);

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

  ros::spinOnce();
  right_sub.shutdown();
  left_sub.shutdown();
  right_ep_sub.shutdown();
  left_ep_sub.shutdown();

  pub_cmd_aile_avg.publish(cmd_aile_avg);
  pub_cmd_aile_avd.publish(cmd_aile_avd);
  pub_cmd_aile_arg.publish(cmd_aile_arg);
  pub_cmd_aile_ard.publish(cmd_aile_ard);
  pub_cmd_epaule_avg.publish(cmd_epaule_avg);
  pub_cmd_epaule_avd.publish(cmd_epaule_avd);
  pub_cmd_epaule_arg.publish(cmd_epaule_arg);
  pub_cmd_epaule_ard.publish(cmd_epaule_ard);
  pub_cmd_coude_avg.publish(cmd_coude_avg);
  pub_cmd_coude_avd.publish(cmd_coude_avd);
  pub_cmd_coude_arg.publish(cmd_coude_arg);
  pub_cmd_coude_ard.publish(cmd_coude_ard);

  pub_pid_aile_avg.publish(pid_aile_av);
  pub_pid_aile_avd.publish(pid_aile_av);
  pub_pid_aile_arg.publish(pid_aile_ar);
  pub_pid_aile_ard.publish(pid_aile_ar);
  pub_pid_epaule_avg.publish(pid_epaule_av);
  pub_pid_epaule_avd.publish(pid_epaule_av);
  pub_pid_epaule_arg.publish(pid_epaule_ar);
  pub_pid_epaule_ard.publish(pid_epaule_ar);
  pub_pid_coude_avg.publish(pid_coude_av);
  pub_pid_coude_avd.publish(pid_coude_av);
  pub_pid_coude_arg.publish(pid_coude_ar);
  pub_pid_coude_ard.publish(pid_coude_ar);

  ROS_INFO("communication beagles ok");
}

void Manette::controllersOff()
{

  double_param.name = "p";
  double_param.value = 0.0;

  pid_off.doubles.push_back(double_param) ;
  
  pub_pid_aile_avg.publish(pid_off);
  pub_pid_aile_avd.publish(pid_off);
  pub_pid_aile_arg.publish(pid_off);
  pub_pid_aile_ard.publish(pid_off);
  pub_pid_epaule_avg.publish(pid_off);
  pub_pid_epaule_avd.publish(pid_off);
  pub_pid_epaule_arg.publish(pid_off);
  pub_pid_epaule_ard.publish(pid_off);
  pub_pid_coude_avg.publish(pid_off);
  pub_pid_coude_avd.publish(pid_off);
  pub_pid_coude_arg.publish(pid_off);
  pub_pid_coude_ard.publish(pid_off);

  ROS_INFO("Controllers OFF.");
}

void Manette::controllersOn()
{
  double_param.name = "p";
  double_param.value = 250.0;
  pid_aile_av.doubles.push_back(double_param) ;
  pid_epaule_av.doubles.push_back(double_param) ;

  double_param.value = 300.0;
  pid_aile_ar.doubles.push_back(double_param) ;
  pid_epaule_ar.doubles.push_back(double_param) ;

  double_param.value = 900.0;
  pid_coude_av.doubles.push_back(double_param) ;

  double_param.value = 1000.0;
  pid_coude_ar.doubles.push_back(double_param) ;          
                                                 
  pub_pid_aile_avg.publish(pid_aile_av);
  pub_pid_aile_avd.publish(pid_aile_av);
  pub_pid_aile_arg.publish(pid_aile_ar);
  pub_pid_aile_ard.publish(pid_aile_ar);
  pub_pid_epaule_avg.publish(pid_epaule_av);
  pub_pid_epaule_avd.publish(pid_epaule_av);
  pub_pid_epaule_arg.publish(pid_epaule_ar);
  pub_pid_epaule_ard.publish(pid_epaule_ar);
  pub_pid_coude_avg.publish(pid_coude_av);
  pub_pid_coude_avd.publish(pid_coude_av);
  pub_pid_coude_arg.publish(pid_coude_ar);
  pub_pid_coude_ard.publish(pid_coude_ar);

  ROS_INFO("Controllers ON.");
}

void Manette::moveAllPosition(double* targetPos, double nb_boucles)
{
  float increment[12];

  increment[0] = (targetPos[0] - cmd_aile_avg.data) / nb_boucles ;
  increment[1] = (targetPos[1] - cmd_epaule_avg.data) / nb_boucles ;
  increment[2] = (targetPos[2] - cmd_coude_avg.data) / nb_boucles ;
  increment[3] = (targetPos[3] - cmd_aile_avd.data) / nb_boucles ;
  increment[4] = (targetPos[4] - cmd_epaule_avd.data) / nb_boucles ;
  increment[5] = (targetPos[5] - cmd_coude_avd.data) / nb_boucles ;
  increment[6] = (targetPos[6] - cmd_aile_arg.data) / nb_boucles ;
  increment[7] = (targetPos[7] - cmd_epaule_arg.data) / nb_boucles ;
  increment[8] = (targetPos[8] - cmd_coude_arg.data) / nb_boucles ;
  increment[9] = (targetPos[9] - cmd_aile_ard.data) / nb_boucles ;
  increment[10] = (targetPos[10] - cmd_epaule_ard.data) / nb_boucles ;
  increment[11] = (targetPos[11] - cmd_coude_ard.data) / nb_boucles ;

  for(int i=1; i<=nb_boucles; i++)
  {
    if(!ros::ok()) break;
    cmd_aile_avg.data = limite_aile(cmd_aile_avg.data + increment[0]) ;
    pub_cmd_aile_avg.publish(cmd_aile_avg);

    cmd_epaule_avg.data = limite_epaule(cmd_epaule_avg.data + increment[1]) ;
    pub_cmd_epaule_avg.publish(cmd_epaule_avg);

    cmd_coude_avg.data = limite_coude(cmd_coude_avg.data + increment[2]) ;
    pub_cmd_coude_avg.publish(cmd_coude_avg);

    cmd_aile_avd.data = limite_aile(cmd_aile_avd.data + increment[3]) ;
    pub_cmd_aile_avd.publish(cmd_aile_avd);

    cmd_epaule_avd.data = limite_epaule(cmd_epaule_avd.data + increment[4]) ;
    pub_cmd_epaule_avd.publish(cmd_epaule_avd);

    cmd_coude_avd.data = limite_coude(cmd_coude_avd.data + increment[5]) ;
    pub_cmd_coude_avd.publish(cmd_coude_avd);

    cmd_aile_arg.data = limite_aile(cmd_aile_arg.data + increment[6]) ;
    pub_cmd_aile_arg.publish(cmd_aile_arg);

    cmd_epaule_arg.data = limite_epaule(cmd_epaule_arg.data + increment[7]) ;
    pub_cmd_epaule_arg.publish(cmd_epaule_arg);

    cmd_coude_arg.data = limite_coude(cmd_coude_arg.data + increment[8]) ;
    pub_cmd_coude_arg.publish(cmd_coude_arg);

    cmd_aile_ard.data = limite_aile(cmd_aile_ard.data + increment[9]) ;
    pub_cmd_aile_ard.publish(cmd_aile_ard);

    cmd_epaule_ard.data = limite_epaule(cmd_epaule_ard.data + increment[10]) ;
    pub_cmd_epaule_ard.publish(cmd_epaule_ard);

    cmd_coude_ard.data = limite_coude(cmd_coude_ard.data + increment[11]) ;
    pub_cmd_coude_ard.publish(cmd_coude_ard);


    usleep(1000);
  }
}

void Manette::stand()
{   
  //les ailes à 0°, les épaules à PI/4 et les coudes à PI/2
  double pos[12] = {0.0, 0.7269, 1.57792, 0.0, 0.7269, 1.57792, 
                    0.0, 0.7269, 1.57792, 0.0, 0.7269, 1.57792};
  moveAllPosition(pos, 800);
}

void Manette::sit()
{   
  //les ailes à 0°, les épaules à PI/8 et les coudes à PI/4
  double pos[12] = {0.0, 0.7269, 1.47792, 0.0, 0.7269, 1.47792, 
                    0.0, 0.3927, 0.7854, 0.0, 0.3927, 0.7854};
  moveAllPosition(pos, 2000);
}

void Manette::reversesit()
{   
  //les ailes à 0°, les épaules à PI/8 et les coudes à PI/4
  double pos[12] = {0.0, 0.4854, 0.77, 0.0, 0.4854, 0.77, 
                      0.0, 0.8854, 1.77, 0.0, 0.8854, 1.77};
  moveAllPosition(pos, 2000);
}

void Manette::down()
{   
  //les ailes à 0°, les épaules et les coudes à leurs limites basses
  double pos[12] = {0.0, 0.0, 0.55, 0.0, 0.0, 0.55, 
                    0.0, 0.0, 0.55, 0.0, 0.0, 0.55};
  moveAllPosition(pos, 2000);
}

void Manette::twerk()
{   
  //les ailes à 0°, les épaules à PI/8 et les coudes à PI/4
  double pos[12] = {0.0, 0.4854, 0.77, 0.0, 0.4854, 0.77, 
                      0.0, 0.8854, 1.77, 0.0, 0.8854, 1.77};
  moveAllPosition(pos, 2000);
  for(int i=1; i<=12; i++)
  {
    
    double pos1[12] = {0.0, 0.4854, 0.77, 0.0, 0.4854, 0.77, 
                      0.0, 0.8854, 1.77, 0.0, 0.8854, 1.77};
    moveAllPosition(pos, 200);
    double pos2[12] = {0.0, 0.4854, 0.77, 0.0, 0.4854, 0.77, 
                       0.0, 0.6354, 1.47, 0.0, 0.6354, 1.47};
    moveAllPosition(pos2, 200);
  }
}

void Manette::left()
{   
  //les ailes à 0°, les épaules à PI/4 et les coudes à PI/2
  double pos[12] = {0.0, 0.7269, 1.22792, 0.0, 0.7269, 1.57792, 
                    0.0, 0.7269, 1.22792, 0.0, 0.7269, 1.57792};
  moveAllPosition(pos, 1000);
}

void Manette::right()
{   
  //les ailes à 0°, les épaules à PI/4 et les coudes à PI/2
  double pos[12] = {0.0, 0.7269, 1.57792, 0.0, 0.7269, 1.22792, 
                    0.0, 0.7269, 1.57792, 0.0, 0.7269, 1.22792};
  moveAllPosition(pos, 1000);
}


void Manette::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==1 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    down();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==1 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    reversesit();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==1 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    sit();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    stand();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==1 &&
    joy->buttons[B_]==1 &&
    joy->buttons[X_]==1 &&
    joy->buttons[Y_]==1 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    twerk();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==1 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    left();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==1 &&
    joy->buttons[start_]==0
    )
  {
    right();
  }

  else if (
    joy->axes[LT_] < 0 &&
    joy->axes[RT_] > 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    controllersOff();
  }

  else if (
    joy->axes[LT_] > 0 &&
    joy->axes[RT_] < 0 &&
    joy->buttons[A_]==0 &&
    joy->buttons[B_]==0 &&
    joy->buttons[X_]==0 &&
    joy->buttons[Y_]==0 &&
    joy->buttons[LB_]==0 &&
    joy->buttons[RB_]==0 &&
    joy->buttons[start_]==0
    )
  {
    controllersOn();
  }
}

void Manette::rightCallback(const popi_robot::Potards::ConstPtr& mes)
{
  cmd_aile_avd.data=mes->aile_av;
  cmd_aile_ard.data=mes->aile_ar;
  cmd_coude_avd.data=mes->coude_av;
  cmd_coude_ard.data=mes->coude_ar;
}

void Manette::leftCallback(const popi_robot::Potards::ConstPtr& mes)
{
  cmd_aile_avg.data=mes->aile_av;
  cmd_aile_arg.data=mes->aile_ar;
  cmd_coude_avg.data=mes->coude_av;
  cmd_coude_arg.data=mes->coude_ar;
}

void Manette::rightEpCallback(const popi_robot::RouesCodeuses::ConstPtr& mes)
{
  cmd_epaule_avd.data=mes->epaule_av;
  cmd_epaule_ard.data=mes->epaule_ar;
}

void Manette::leftEpCallback(const popi_robot::RouesCodeuses::ConstPtr& mes)
{
  cmd_epaule_avg.data=mes->epaule_av;
  cmd_epaule_arg.data=mes->epaule_ar;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "manette");
  Manette manette;

  ros::spin();
}
