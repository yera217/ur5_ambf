#include <ur5_rcm/ur5_kinematics.hpp>
#include <cmath>


ur5_kinematics::ur5_kinematics (ros::NodeHandle& nh) : nh(nh), kdl_fk_pos(NULL) { 

  //*** Initializing needed subscribers
  sub_js = nh.subscribe("/ur5_env/joint_states", 10, &ur5_kinematics::callback_js, this);
  
  //*** Initializing UR5 chain using DH parameters
  //ur5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, 0.089159, 0+M_PI)));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(-0.425, 0, 0, 0)));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(-0.39225, 0, 0, 0)));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, 0.10915, 0)));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, -M_PI/2, 0.09465, 0)));
  //chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, 0.0823, 0)));
  //tool
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, 0.11085, M_PI/2)));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, 0.416, M_PI/2)));
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, M_PI/2, 0.009, 0)));
  
  kdl_fk_pos = new KDL::ChainFkSolverPos_recursive( chain );
  //_kdl_ik_vel = new KDL::ChainIkSolverVel_pinv(chain);
  //kdl_ik_pos = new KDL::ChainIkSolverPos_NR( chain, *kdl_fk_pos, *_kdl_ik_vel);
  
}

//ur5_kinematics::~ur5_kinematics(){if (kdl_ik_pos) delete kdl_ik_pos; if (kdl_fk_pos) delete kdl_fk_pos;
						       //if (_kdl_ik_vel) delete _kdl_ik_vel;}
ur5_kinematics::~ur5_kinematics(){if (kdl_fk_pos) delete kdl_fk_pos;}

						       
void ur5_kinematics::callback_js(const sensor_msgs::JointState& js){

  KDL::JntArray q_in(js.position.size() );
  for (int i=0; i<q_in.rows(); i++) {
    q_in(i)=js.position[i];
  }
  
  //*** KDL FK solver
  KDL::Frame kdl_frame_fk;
  kdl_fk_pos -> JntToCart( q_in, kdl_frame_fk);


  std::cout << kdl_frame_fk(0,3) << " " << kdl_frame_fk(1,3) << " " << kdl_frame_fk(2,3) << std::endl;


  //*** KDL IK solver
  //KDL::JntArray kdl_q_out( js.position.size() );
  //kdl_ik_pos -> CartToJnt( q_in, kdl_frame_fk, kdl_q_out );
  
}
