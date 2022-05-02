# Import the Client from ambf_comm package                                                                         
from ambf_client import Client

import rospy
from sensor_msgs.msg import JointState
import time




# Create a instance of the client                                                                                  
_client = Client("ur5_ambf")
_client.connect()

# Base link object handle
base_obj = _client.get_obj_handle('/ambf/env/ur5/base_link')
tool_obj = _client.get_obj_handle('/ambf/env/ur5/wrist_2_link')
#time.sleep(0.3)

# Printing joint names
joint_names = base_obj.get_joint_names()
print(joint_names)



pub = rospy.Publisher('ur5_env/joint_states', JointState, queue_size=10)
rate = rospy.Rate(10) 


while not rospy.is_shutdown():
    
    # Setting joint values
    base_obj.set_joint_pos(0, 1.0)
    base_obj.set_joint_pos(1, 1.0)
    base_obj.set_joint_pos(2, 1.0)
    base_obj.set_joint_pos(3, 1.0)
    base_obj.set_joint_pos(4, 1.0)
    base_obj.set_joint_pos(5, 1.0)
    #base_obj.set_joint_pos(6, 0.0)
    base_obj.set_joint_pos(7, 1.0)
    base_obj.set_joint_pos(8, 1.0)
    base_obj.set_joint_pos(9, 1.0)
    base_obj.set_joint_pos(10, 1.0)


    # Getting joint values
    j1=base_obj.get_joint_pos(0)
    j2=base_obj.get_joint_pos(1)
    j3=base_obj.get_joint_pos(2)
    j4=base_obj.get_joint_pos(3)
    j5=base_obj.get_joint_pos(4)
    #j6=base_obj.get_joint_pos(5)
    #j7=base_obj.get_joint_pos(6)
    #j8=base_obj.get_joint_pos(7)

    # Publishing current joint values
    js = JointState(position=[j1,j2,j3,j4,j5])
    pub.publish(js)

    
    print("Current true FK of the tool: ", tool_obj.get_pos(), tool_obj.get_rpy())
    rate.sleep()


_client.clean_up()

