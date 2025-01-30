#!/usr/bin/env python 

# Import the Client from ambf_comm package                                                                         
from ambf_client import Client
from utils.obj_control_gui import ObjectGUI

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import pyOpt

from std_msgs.msg import Float64MultiArray
from PyKDL import Chain, Segment, Joint, Frame, Rotation, Vector, JntArray, Jacobian
from PyKDL import ChainFkSolverPos_recursive, ChainJntToJacSolver
import kdl_parser_py.urdf as kdl_parser

import numpy as np
from numpy import linalg as LA



# =============================================================================
# Standard Python modules
# =============================================================================
import os, sys, time, math


# =============================================================================
# Extension modules
# =============================================================================
from pyOpt import Optimization
from pyOpt import SLSQP


class TargetController:
	def __init__(self, gui_handle, target_handle):
		self.counter = 0
		self.GUI = gui_handle
		self.target = target_handle

	def update_target_pose(self):
		gui = self.GUI
		gui.App.update()
		self.target.set_pos(gui.x, gui.y, gui.z)
		self.target.set_rpy(gui.ro, gui.pi, gui.ya)

	def run(self):
			self.update_target_pose()


class TargetObj():
	def __init__(self, target_obj, init_xyz, init_rpy):
		self.handle = target_obj
		self.gui = ObjectGUI('target_pos', init_xyz, init_rpy, 0.2, 1, 0.000001)
		self.controller = TargetController(self.gui, self.handle)


class ur5_rcm():
	def __init__(self, target):

		self.target = target

		# Base link object handle
		self.base_obj = _client.get_obj_handle('/ambf/env/ur5/base_link')
		self.trocar_obj = _client.get_obj_handle('/ambf/env/Torus')
		self.tool_obj = _client.get_obj_handle('/ambf/env/ur5/toolgripper1link')
		time.sleep(0.3)

		# Printing joint names
		joint_names = self.base_obj.get_joint_names()
		children_names = self.base_obj.get_children_names()
		print(joint_names, children_names)

		# Parse urdf robot description parameter to KDL chain
		flag,self.tree = kdl_parser.treeFromParam("/robot_description")
		chain = self.tree.getChain("base_link", "wrist_3_link")

		d6=0.1107
		d7=0.416
		a8=0.009

		chain.addSegment ( Segment(Joint(getattr(Joint, 'None')), Frame(Vector(0,d6,0)) ) )
		chain.addSegment ( Segment(Joint(Joint.RotX), Frame( Rotation.RPY(0, 0, 0), Vector(d7,0,0)) ) )
		chain.addSegment ( Segment(Joint(Joint.RotZ), Frame( Rotation.RPY(math.pi, 0, 0), Vector(0,0,0)) ) )
		chain.addSegment ( Segment(Joint(Joint.RotY), Frame( Vector(a8,0,0))) )


		# Initializing KDL FK and Jac solvers for the tool tip
		self.fk_frame = ChainFkSolverPos_recursive( chain );
		self.J_tip = ChainJntToJacSolver( chain )

		self.cmd_vel = Float64MultiArray()
		self.cmd_vel.data=[0,0,0,0,0,0,0,0,0]

	def run(self):
		while not rospy.is_shutdown():

			# Setting x_des target cartesian position [x y z]
			xyz_target=self.target.handle.get_pos()
			rpy_target=self.target.handle.get_rpy()
			x_des = np.array( [xyz_target.x, xyz_target.y, xyz_target.z, rpy_target[0], rpy_target[1], rpy_target[2] ])
			
			# Setting RCM xyz coordinates                                                                                      
			xyz_RCM = np.array( [0.5, 0.5, 0.05] )

			# Setting joint values
			print ("***")
			self.base_obj.set_joint_vel(0, self.cmd_vel.data[0])
			self.base_obj.set_joint_vel(1, self.cmd_vel.data[1])
			self.base_obj.set_joint_vel(2, self.cmd_vel.data[2])
			self.base_obj.set_joint_vel(3, self.cmd_vel.data[3])
			self.base_obj.set_joint_vel(4, self.cmd_vel.data[4])
			self.base_obj.set_joint_vel(5, self.cmd_vel.data[5])
			self.base_obj.set_joint_vel(7, self.cmd_vel.data[6])
			self.base_obj.set_joint_vel(8, self.cmd_vel.data[7])
			self.base_obj.set_joint_vel(9, self.cmd_vel.data[8])

			print ("CMD VEL*******", self.cmd_vel)
			
			# Getting joint values
			j1=self.base_obj.get_joint_pos(0)
			j2=self.base_obj.get_joint_pos(1)
			j3=self.base_obj.get_joint_pos(2)
			j4=self.base_obj.get_joint_pos(3)
			j5=self.base_obj.get_joint_pos(4)
			j6=self.base_obj.get_joint_pos(5)
			j7=self.base_obj.get_joint_pos(7)
			j8=self.base_obj.get_joint_pos(8)
			j9=self.base_obj.get_joint_pos(9)
			
			# Updating current joint state values
			js = JointState(position=[j1,j2,j3,j4,j5,j6,j7,j8,j9])

			print("Current true FK of the tool tip: ", self.tool_obj.get_pos(), self.tool_obj.get_rpy())
			print("Current true FK of the trocar: ", self.trocar_obj.get_pos(), self.trocar_obj.get_rpy())    
			print("Current joint states: ", js)
		


			# Transform ROS JntState to KDL JntArray
			q_in = JntArray(len(js.position))
			for i in range(q_in.rows()):
				q_in[i]=js.position[i]

			# Transform ROS JntState to KDL JntArray of UR5 only (for J_cl)
			q_ur5 = JntArray(6)
			for i in range(6):
				q_ur5[i]=js.position[i]
				

			# Invoke FK solver
			kdl_fk_frame=Frame()
			self.fk_frame.JntToCart(q_in, kdl_fk_frame)
			
			# Invoke Jac solver
			kdl_J_tip=Jacobian(q_in.rows())
			self.J_tip.JntToJac(q_in, kdl_J_tip)


			

			# Converting from KDl types to numpy
			rpy_fk=kdl_fk_frame.M.GetRPY()
			x_curr=np.array( [ kdl_fk_frame.p.x(), kdl_fk_frame.p.y(), kdl_fk_frame.p.z(), rpy_fk[0], rpy_fk[1], rpy_fk[2] ])
			print("x_curr:" ,x_curr)
			print("x_des:", x_des)
			
			J_tip=np.zeros((6,q_in.rows()))
			for i in range(6):
				for j in range(q_in.rows()):
					J_tip[i,j]=kdl_J_tip[i,j]


					
			### Finding closest point on the shaft

			# Defining chain that needed to calculate J at the closest to RCM point on the shaft
			chain_cl = self.tree.getChain("base_link", "wrist_3_link")
			
			d6=0.1107
			chain_cl.addSegment ( Segment(Joint(getattr(Joint, 'None')), Frame(Vector(0,d6,0)) ) )

			# Position xyz of the tool base (tb)
			fk_frame_cl = ChainFkSolverPos_recursive( chain_cl );
			kdl_fk_frame_tb=Frame()
			fk_frame_cl.JntToCart(q_ur5, kdl_fk_frame_tb)
			xyz_tb = np.array( [kdl_fk_frame_tb.p.x(), kdl_fk_frame_tb.p.y(), kdl_fk_frame_tb.p.z()] )
			print("Tool base location", xyz_tb)

			# Tool tip (tt) xyz coordinates
			xyz_tt = x_curr[0:3]

			lamda = np.dot( (xyz_RCM-xyz_tb), (xyz_tt-xyz_tb) ) / np.dot( (xyz_tt-xyz_tb), (xyz_tt-xyz_tb) )
			lamda = max(0, min(lamda,1))

			# Finding xyz coordinates of the point on the shaft closest to RCM
			xyz_cl = xyz_tb+lamda*(xyz_tt-xyz_tb)
			print("Closest points location", xyz_cl)
			
			# Calculating Jacobian at the closest point
			dist_shaft_cl = LA.norm(xyz_cl-xyz_tb)
			print("Distance along the shaft to RCM",dist_shaft_cl)
			#chain_cl.addSegment ( Segment(Joint(Joint.RotX), Frame(Rotation.RPY(math.pi/2, math.pi, math.pi/2),Vector(dist_shaft_cl,0,0)) ) )
			chain_cl.addSegment ( Segment(Joint(getattr(Joint, 'None')), Frame(Vector(dist_shaft_cl,0,0)) ) )	
			J_cl_solver = ChainJntToJacSolver( chain_cl )
			fk_frame_cl = ChainFkSolverPos_recursive( chain_cl );
			kdl_J_cl=Jacobian(6)
			J_cl_solver.JntToJac(q_ur5, kdl_J_cl)
			J_cl=np.zeros((3,6))
			for i in range(3):
				for j in range(6):
					J_cl[i,j]=kdl_J_cl[i,j]


			print ("DISTANCE from shaft TO RCM",LA.norm( xyz_cl-xyz_RCM))

					
			# Objective function for pyOpt                                                                             
			def ik(qdot):
				#qdot=np.array(qdot)
				f = LA.norm( np.matmul(J_tip,qdot/10.0)  - (x_des -x_curr) )
				
				epsilon = 0.01
				g=[0.0]*1
				g[0] = LA.norm( [ xyz_cl[0]- xyz_RCM[0] + helper(J_cl,qdot/10.0, 0), xyz_cl[1] - xyz_RCM[1] + helper(J_cl,qdot/10.0, 1), xyz_cl[2] - xyz_RCM[2] + helper(J_cl,qdot/10.0, 2) ] ) - epsilon
				
				fail=0
				return f,g,fail

			def helper(J,qdot,n):
				res=0.0
				for j in range(6):
					res+=J[n,j]*qdot[j]
				return res

			opt_prob = pyOpt.Optimization('IK velocity',ik)
			opt_prob.addObj('f')
			opt_prob.addVarGroup('qdot', q_in.rows(), 'c', lower=-1, upper=1, value=0)
			opt_prob.addConGroup('g',1,'i')
			#opt_prob.addCon('g','i')
			#print opt_prob

			slsqp = pyOpt.SLSQP()

			[_, sol, _] = slsqp(opt_prob)
			self.cmd_vel.data=sol

			
			self.target.controller.run()
			rate.sleep()



# Create a instance of the client
_client = Client("ur5_rcm")
_client.connect()
time.sleep(0.5)


rate = rospy.Rate(120)


### Target tool position GUI control initialization
init_xyz = [0.55, 0.55, 0.0]
init_rpy = [0.0,  0.0, 0.0]
target_handle = _client.get_obj_handle('/ambf/env/Target')	
target_obj = TargetObj(target_handle, init_xyz, init_rpy)




UR5_RCM = ur5_rcm(target_obj)
UR5_RCM.run()

_client.clean_up()

