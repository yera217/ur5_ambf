# Import the Client from ambf_comm package
from ambf_client import Client
#from obj_control_gui import ObjectGUI
import rospy
from sensor_msgs.msg import JointState
import time

# Create a instance of the client
_client = Client("ur5")

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bidrectional communication 
_client.connect()
time.sleep(0.3)

# You can print the names of objects found
#print(_client.get_obj_names())

elbow_obj = _client.get_obj_handle('/ambf/env/ur5/base_link')
tool_obj = _client.get_obj_handle('/ambf/env/ur5/wrist_3_link')
#time.sleep(0.3)

joint_names = elbow_obj.get_joint_names()
print(joint_names)

j1=elbow_obj.get_joint_pos(0)
j2=elbow_obj.get_joint_pos(1)
j3=elbow_obj.get_joint_pos(2)
j4=elbow_obj.get_joint_pos(3)
j5=elbow_obj.get_joint_pos(4)
#j6=elbow_obj.get_joint_pos(5)
#j7=elbow_obj.get_joint_pos(6)
#j8=elbow_obj.get_joint_pos(7)

#time.sleep(0.3)
print("Current joint values: ",j1,j2,j3,j4,j5)

#print("Current true FK of the tool: ", tool_obj.get_pos())

pub = rospy.Publisher('ur5_env/joint_states', JointState, queue_size=10)
#rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
    j1=elbow_obj.get_joint_pos(0)
    j2=elbow_obj.get_joint_pos(1)
    j3=elbow_obj.get_joint_pos(2)
    j4=elbow_obj.get_joint_pos(3)
    j5=elbow_obj.get_joint_pos(4)
    #j6=elbow_obj.get_joint_pos(5)
    #j7=elbow_obj.get_joint_pos(6)
    #j8=elbow_obj.get_joint_pos(7)
    js = JointState(position=[j1,j2,j3,j4,j5])
    pub.publish(js)
    print("Current true FK of the tool: ", tool_obj.get_pos())
    rate.sleep()

    
#_client.clean_up()




#print(elbow_obj.object_type)



#cur_pos=elbow_obj.get_pos()
#print(cur_pos)

#obj_gui = ObjectGUI("ur5", None, None, None, None, None)



#joint_names = elbow_obj.get_joint_names()
#num_joints = elbow_obj.get_num_joints() # Get the number of joints of this object
#children_names = elbow_obj.get_children_names() # Get a list of children names belonging to this obj

#print(joint_names)
#print(num_joints)
#print(children_names)

#elbow_obj.set_pos(2, 0.2, 0) 
#time.sleep(2)

#elbow_obj.set_joint_pos(0, 0.5)
#time.sleep(2)