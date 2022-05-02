# Import the Client from ambf_comm package                                                                         \
                                                                                                                    
from ambf_client import Client
#from obj_control_gui import ObjectGUI                                                                             \
                                                                                                                    
import rospy
from sensor_msgs.msg import JointState
import time

# Create a instance of the client                                                                                  \
                                                                                                                    
_client = Client("ur5_2")

# Connect the client which in turn creates callable objects from ROS topics                                        \
                                                                                                                    
# and initiates a shared pool of threads for bidrectional communication                                            \
                                                                                                                    
_client.connect()
#time.sleep(0.3)                                                                                                    

# You can print the names of objects found                                                                         \
                                                                                                                    
#print(_client.get_obj_names())                                                                                    \
                                                                                                                    

elbow_obj = _client.get_obj_handle('/ambf/env/ur5_robot/base_link_inertia')
time.sleep(0.3)

elbow_obj.set_joint_pos(1, 0.0);
elbow_obj.set_joint_pos(2, 0.0);
elbow_obj.set_joint_pos(3, 0.0);
elbow_obj.set_joint_vel(4, 0.0);
elbow_obj.set_joint_vel(5, 0.0);
elbow_obj.set_joint_vel(6, 0.0);                                                                                   


time.sleep(10000000)
_client.clean_up()
