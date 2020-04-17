#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <xpp_msgs/RobotStateJoint.h>


class JointPoseTranslator
{
public:
	JointPoseTranslator();

private:
	void StateCallback(const xpp_msgs::RobotStateJoint& msg);
  
	std_msgs::Float64 cmd_aile_avd, cmd_aile_avg, cmd_aile_ard, cmd_aile_arg;
  std_msgs::Float64 cmd_epaule_avd, cmd_epaule_avg, cmd_epaule_ard, cmd_epaule_arg;
  std_msgs::Float64 cmd_coude_avd, cmd_coude_avg, cmd_coude_ard, cmd_coude_arg;
	
  ros::NodeHandle nh_;

  ros::Subscriber joint_sub ;

 	ros::Publisher pub_aile_avd, pub_aile_avg, pub_aile_ard, pub_aile_arg;
	ros::Publisher pub_epaule_avd, pub_epaule_avg, pub_epaule_ard, pub_epaule_arg;
	ros::Publisher pub_coude_avd, pub_coude_avg, pub_coude_ard, pub_coude_arg;
};

JointPoseTranslator::JointPoseTranslator()
{
	joint_sub = nh_.subscribe("xpp/joint_popi_des", 1, &JointPoseTranslator::StateCallback, this);

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
}

void JointPoseTranslator::StateCallback(const xpp_msgs::RobotStateJoint& msg)
{
  
	cmd_aile_avg.data = msg.joint_state.position.at(0);
  	cmd_epaule_avg.data = msg.joint_state.position.at(1);
  	cmd_coude_avg.data = msg.joint_state.position.at(2);

  	cmd_aile_avd.data = msg.joint_state.position.at(3);
  	cmd_epaule_avd.data = msg.joint_state.position.at(4);
  	cmd_coude_avd.data = msg.joint_state.position.at(5);

  	cmd_aile_arg.data = msg.joint_state.position.at(6);
  	cmd_epaule_arg.data = msg.joint_state.position.at(7);
  	cmd_coude_arg.data = msg.joint_state.position.at(8);

  	cmd_aile_ard.data = msg.joint_state.position.at(9);
  	cmd_epaule_ard.data = msg.joint_state.position.at(10);
  	cmd_coude_ard.data = msg.joint_state.position.at(11);


  	pub_aile_avg.publish(cmd_aile_avg);
  	pub_epaule_avg.publish(cmd_epaule_avg);
  	pub_coude_avg.publish(cmd_coude_avg);

  	pub_aile_avd.publish(cmd_aile_avd);
  	pub_epaule_avd.publish(cmd_epaule_avd);
  	pub_coude_avd.publish(cmd_coude_avd);

  	pub_aile_arg.publish(cmd_aile_arg);
  	pub_epaule_arg.publish(cmd_epaule_arg);
  	pub_coude_arg.publish(cmd_coude_arg);

  	pub_aile_ard.publish(cmd_aile_ard);
  	pub_epaule_ard.publish(cmd_epaule_ard);
  	pub_coude_ard.publish(cmd_coude_ard);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_pose_translator");
  JointPoseTranslator joint_pose_translator;

  ros::spin();
}