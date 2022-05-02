#include <ur5_rcm/ur5_kinematics.hpp>


int main( int argc, char** argv ) {


  ros::init(argc,argv, "ur5_test");
  ros::NodeHandle nh;

  ur5_kinematics ur5_test(nh);

  ros::spin();

  return 0;

}
