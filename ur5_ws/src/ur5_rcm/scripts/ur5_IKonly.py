#!/usr/bin/env python
import rospy
import pyOpt
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from PyKDL import Chain, Segment, Joint, Frame, Vector, JntArray, Jacobian
from PyKDL import ChainFkSolverPos_recursive, ChainJntToJacSolver
import kdl_parser_py.urdf as kdl_parser

import numpy as np
from numpy import linalg as LA



# =============================================================================
# Standard Python modules
# =============================================================================
import os, sys, time

# =============================================================================
# External Python modules
# =============================================================================
from numpy.linalg import norm

# =============================================================================
# Extension modules
# =============================================================================
from pyOpt import Optimization
from pyOpt import NSGA2
from pyOpt import COBYLA
from pyOpt import SLSQP








### Class for Forward Kinematics calculation
class ur5_fk():
    def __init__(self):

        rospy.Subscriber("/ur5_env/joint_states", JointState , self.callback)
        self.pub = rospy.Publisher("/ur5_env/joint_vels", Float64MultiArray, queue_size=10)
        rate=rospy.Rate(10)
        
        # Parse urdf robot description parameter to KDL chain
        flag,tree = kdl_parser.treeFromParam("/robot_description")
        chain = tree.getChain("base_link", "wrist_3_link")
        
        
        print(chain.getNrOfJoints())

        # Initializing KDL FK and Jac solvers for the tool tip
        self.fk_frame = ChainFkSolverPos_recursive( chain );
        self.J_tip = ChainJntToJacSolver( chain )

        
        # To start two-way communication need to start publishing zero joint velocities
        del_q = Float64MultiArray()
        del_q.data=[0,0,0,0,0,0]
        for i in range(2):
           self.pub.publish(del_q)
           print del_q
           time.sleep(0.1)
        
        
    def callback(this,js):
        print("###")
        # Transform ROS JntState to KDL JntArray
        q_in = JntArray(len(js.position))
        for i in range(q_in.rows()):
            q_in[i]=js.position[i]    

        # Invoke FK solver
        kdl_fk_frame=Frame()
        this.fk_frame.JntToCart(q_in, kdl_fk_frame)
        
        # Invoke Jac solver
        kdl_J_tip=Jacobian(q_in.rows())
        this.J_tip.JntToJac(q_in, kdl_J_tip)

        # Setting test x_des target cartesian position [x y z r p y]
        x_des = np.array([ 0.531, 0.109, 0.342, 2.7, 0.027, 1.628 ])
        
        

        # Converting from KDl types to numpy
        rpy_fk=kdl_fk_frame.M.GetRPY()
        x_curr=np.array( [ kdl_fk_frame.p.x(), kdl_fk_frame.p.y(), kdl_fk_frame.p.z(), rpy_fk[0], rpy_fk[1], rpy_fk[2] ])
        print("x_curr:" ,x_curr)
        print("x_des:", x_des)
        
        J_tip=np.zeros((6,q_in.rows()))
        for i in range(6):
            for j in range(q_in.rows()):
                J_tip[i,j]=kdl_J_tip[i,j]



                
        # Objective function for pyOpt                                                                             
        def ik(qdot):
            #qdot=np.array(qdot)
            f = LA.norm( np.matmul(J_tip,qdot)  - (x_des -x_curr) )
            #epsilon = 0.001
            g=qdot-1000
            #g=[0.0]*3
            #g[0] =   abs(xyz_cl[0]- xyz_RCM[0] + helper(J_cl,qdot,0)) -epsilon
            #g[1] =   abs(xyz_cl[1] - xyz_RCM[1] + helper(J_cl,qdot,1)) -epsilon
            #g[2] =   abs(xyz_cl[2] - xyz_RCM[2] + helper(J_cl,qdot,2)) -epsilon
            
            #g[0] = LA.norm( xyz_cl - xyz_RCM + qdot[0] ) -100

            #g = xyz_cl - xyz_RCM + np.matmul( J_cl, qdot[0] )
            #g = qdot-1000
            
            fail=0
            return f,g,fail

        def helper(J,qdot,n):
            res=0.0
            for j in range(6):
                res+=J[n,j]*qdot[j]
            return res
        
        #print("FK position: ", x_fk[0], x_fk[1], x_fk[2], x_fk[3], x_fk[4], x_fk[5], )
        #print("J_tip", kdl_J_tip[0,0],kdl_J_tip[4,4])

        opt_prob = pyOpt.Optimization('IK velocity',ik)
        opt_prob.addObj('f')
        opt_prob.addVarGroup('qdot', q_in.rows(), 'c', lower=-1, upper=1, value=0)
        opt_prob.addConGroup('g',q_in.rows(),'i')
        #opt_prob.addCon('g','i')
        #print opt_prob

        #slsqp = pyOpt.COBYLA()
        slsqp = pyOpt.SLSQP()
        #slsqp.setOption('IPRINT', -1)

        [_, sol, _] = slsqp(opt_prob)
        del_q = Float64MultiArray()
        del_q.data=sol
        #print(sol)
        this.pub.publish(del_q)

        

rospy.init_node('ur5_rcm')
ur5=ur5_fk()
rospy.spin()
