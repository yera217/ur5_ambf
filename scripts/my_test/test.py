# Import the Client from ambf_comm package
from ambf_client import Client
import time

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bidrectional communication 
_client.connect()

# You can print the names of objects found
print(_client.get_obj_names())

# Lets say from the list of printed names, we want to get the 
# handle to an object names "Torus"
torus_obj = _client.get_obj_handle('Torus')

# Now you can use the torus_obj to set and get its position, rotation,
# Pose etc. If the object has joints, you can also control them
# in either position control mode or open loop effort mode. You can even mix and
# match the joints commands 
torus_obj.set_pos(0, 0, 0) # Set the XYZ Pos in obj's World frame
torus_obj.set_rpy(1.5, 0.7, .0) # Set the Fixed RPY in World frame
time.sleep(5) # Sleep for a while to see the effect of the command before moving on

# Other methods to control the obj position include
# torus_obj.set_pose(pose_cmd) # Where pose_cmd is of type Geometry_msgs/Pose
# torus_obj.set_rot(quaterion) # Where quaternion is a list in the order of [qx, qy, qz, qw]
# Finally all the position control params can be controlled in a single method call
# torus_obj.pose_command(px, py, pz, roll, pitch, yaw, *jnt_cmds)

# We can just as easily get the pose information of the obj
cur_pos = torus_obj.get_pos() # xyx position in World frame
cur_rot = torus_obj.get_rot() # Quaternion in World frame
cur_rpy = torus_obj.get_rpy() # Fixed RPY in World frame

# Similarly you can directly control the wrench acting on the obj by
torus_obj.set_force(5, -5, 10) # Set the force in the World frame
torus_obj.set_torque(0, 0, 0.8) # Set the torque in the World frame
time.sleep(5) # Sleep for a while to see the effect of the command before moving on

# Similarly you can directly control the wrench acting on the obj by
# The key difference is that it's the user's job to update the forces
# and torques in a loop otherwise the wrench in cleared after an internal
# watchdog timer expires if a new command is not set. This is for safety
# reasons where a user shouldn't set a wrench and then forget about it.
for i in range(0, 5000):
    torus_obj.set_force(5, -5, 10) # Set the force in the World frame
    torus_obj.set_torque(0, 0, 0.8) # Set the torque in the World frame
    time.sleep(0.001) # Sleep for a while to see the effect of the command before moving on

# We can get the number of children and joints connected to this body as
num_joints = torus_obj.get_num_joints() # Get the number of joints of this object
children_names = torus_obj.get_children_names() # Get a list of children names belonging to this obj

print(num_joints)
print(children_names)

# If the obj has some joints, we can control them as follows
if num_joints > 1:
    torus_obj.set_joint_pos(0, 0.5) # The the joints at idx 0 to 0.5 Radian
    torus_obj.set_joint_effort(1, 5) # Set the effort of joint at idx 1 to 5 Nm
    time.sleep(2) # Sleep for a while to see the effect of the command before moving on


# Lastly to cleanup
_client.clean_up()
