#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>


class ur5_kinematics {

private:

  ros::NodeHandle nh;
  ros::Subscriber sub_js; //subscribes to /ur5_env/joint_states topic published by ambf client side

  KDL::ChainFkSolverPos_recursive* kdl_fk_pos;
  KDL::ChainIkSolverPos_NR* kdl_ik_pos;
  KDL::ChainIkSolverVel_pinv* _kdl_ik_vel;

  KDL::Chain chain;

  
public:

  ur5_kinematics(ros::NodeHandle& nh);
  ~ur5_kinematics();

  void callback_js(const sensor_msgs::JointState& js);
};

