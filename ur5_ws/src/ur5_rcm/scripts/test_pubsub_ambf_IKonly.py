#!/usr/bin/env python 

# Import the Client from ambf_comm package                                                                         
from ambf_client import Client

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import time




# Create a instance of the client
_client = Client("ur5_ambf")
_client.connect()



class ur5_ambf():
    def __init__(self):

        self.pub = rospy.Publisher('/ur5_env/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/ur5_env/joint_vels", Float64MultiArray, self.callback_ambf)
        rate = rospy.Rate(10)
        
        # Base link object handle
        self.base_obj = _client.get_obj_handle('/ambf/env/ur5/base_link')
        self.tool_obj = _client.get_obj_handle('/ambf/env/ur5/wrist_3_link')
        time.sleep(0.3)                                               

        print("Current true FK of the tool: ", self.tool_obj.get_pos(), self.tool_obj.get_rpy())

        # Printing joint names
        joint_names = self.base_obj.get_joint_names()
        children_names = self.base_obj.get_children_names()
        print(joint_names, children_names)


    def callback_ambf(this,cmd_vel):
        # Setting joint values
        print ("***")
        this.base_obj.set_joint_vel(0, cmd_vel.data[0])
        this.base_obj.set_joint_vel(1, cmd_vel.data[1])
        this.base_obj.set_joint_vel(2, cmd_vel.data[2])
        this.base_obj.set_joint_vel(3, cmd_vel.data[3])
        this.base_obj.set_joint_vel(4, cmd_vel.data[4])
        this.base_obj.set_joint_vel(5, cmd_vel.data[5])
        
        # Getting joint values
        j1=this.base_obj.get_joint_pos(0)
        j2=this.base_obj.get_joint_pos(1)
        j3=this.base_obj.get_joint_pos(2)
        j4=this.base_obj.get_joint_pos(3)
        j5=this.base_obj.get_joint_pos(4)
        j6=this.base_obj.get_joint_pos(5)

        # Publishing current joint values
        js = JointState(position=[j1,j2,j3,j4,j5,j6])
        this.pub.publish(js)

        print("Current true FK of the tool: ", this.tool_obj.get_pos(), this.tool_obj.get_rpy())
        #print("Current joint states: ", js)

ur5_ambf_ = ur5_ambf()
rospy.spin()

_client.clean_up()
