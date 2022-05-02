# Import the Client from ambf_comm package                                                                         
from ambf_client import Client 
import rospy
from sensor_msgs.msg import JointState
import time

# Create a instance of the client                                                                                  
_client = Client("ur5_2")

# Connect the client which in turn creates callable objects from ROS topics                                         
# and initiates a shared pool of threads for bidrectional communication                                             
_client.connect()
time.sleep(0.3)

# You can print the names of objects found                                                                          
#print(_client.get_obj_names())                                                                                     

base_obj = _client.get_obj_handle('/ambf/env/ur5/base_link')
time.sleep(0.3)

pos=base_obj.get_pos()
rpy=base_obj.get_rpy()

print(pos, "and\n",  rpy)

time.sleep(10000000)
_client.clean_up()

