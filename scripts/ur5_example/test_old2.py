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

elbow_obj = _client.get_obj_handle('/ambf/env/ur5_robot/base_link_inertia')
tool_obj = _client.get_obj_handle('/ambf/env/ur5_robot/wrist_3_link')
#time.sleep(0.3)                                                                                                    

joint_names = elbow_obj.get_joint_names()
print(joint_names)

j1=elbow_obj.get_joint_pos(0)
j2=elbow_obj.get_joint_pos(1)
j3=elbow_obj.get_joint_pos(2)
j4=elbow_obj.get_joint_pos(3)
j5=elbow_obj.get_joint_pos(4)
#j6=elbow_obj.get_joint_pos(5)                                                                                      
#time.sleep(0.3)                                                                                                    
print("Current joint values: ",j1,j2,j3,j4,j5) #,j6                                                                 

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
    js = JointState(position=[j1,j2,j3,j4,j5])  #,j6                                                                
    pub.publish(js)
    print("Current true FK of the tool: ", tool_obj.get_pos())
    rate.sleep()
