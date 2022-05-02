#!/usr/bin/env python2
import rospy
import pyOpt
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from PyKDL import Chain, Segment, Joint, Frame, JntArray, Jacobian
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
from pyOpt import SLSQP







### Class for Forward Kinematics calculation
class ur5_fk():
    def __init__(self):

        rospy.Subscriber("/ur5_env/joint_states", JointState , self.callback)
        self.pub = rospy.Publisher("/ur5_env/joint_vels", Float64MultiArray, queue_size=10)
        rate=rospy.Rate(10)
        
        # Parse urdf robot description parameter to KDL chain
        flag,tree = kdl_parser.treeFromParam("/robot_description")
        chain = tree.getChain("base_link", "wrist_2_link")
        
        d6=0.1107
        d7=0.416
        a8=0.009
        #chain.addSegment( Segment( Joint(Joint.RotY), Frame().DH(0, -math.pi/2, d6, -math.pi/2) ) )


        #chain.addSegment( Segment( Joint(Joint.RotY), Frame().DH(0,-math.pi/2,d6,-math.pi/2) ) )


        #chain.addSegment( Segment( Joint(Joint.RotZ), Frame().DH(0,math.pi/2,d7,-math.pi/2) ) )
        #chain.addSegment( Segment( Joint(Joint.RotZ), Frame().DH(a8,0,0,0) ) )
        
        #print(chain.getNrOfJoints(), chain.getSegment(0).getName(),chain.getSegment(1).getName(),chain.getSegment(2).getName(),chain.getSegment(3).getName(),chain.getSegment(4).getName(), chain.getSegment(5).getName())
        print(chain.getNrOfJoints())
        #chain = Chain()
        #s1=Segment(Joint(Joint2.RotZ), Frame.DH(0, math.pi/2, 0.089159, math.pi)
        #addSegment(s1)
        #chain.addSegment(Segment(Joint(Joint.RotZ), Frame.DH(-0.425, 0, 0, 0)))
        #chain.addSegment(Segment(Joint(Joint.RotZ), Frame.DH(-0.39225, 0, 0, 0)))
        #chain.addSegment(Segment(Joint(Joint.RotZ), Frame.DH(0, math.pi/2, 0.10915, 0)))
        #chain.addSegment(Segment(Joint(Joint.RotZ), Frame.DH(0, -math.pi/2, 0.09465, 0)))

        # Initializing KDL FK and Jac solvers
        self.fk_frame = ChainFkSolverPos_recursive( chain );
        self.J_tip = ChainJntToJacSolver( chain )

        del_q = Float64MultiArray()
        del_q.data=[0,0,0,0,0]
        for i in range(2):
           self.pub.publish(del_q)
           print del_q
           time.sleep(0.1)
        #rospy.spin()
        
        
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
        x_des = np.array([ 0.2, 0.3, -0.5, -0.5, -0.5, 0.3 ])


        # Converting from KDl types to numpy
        rpy_fk=kdl_fk_frame.M.GetRPY()
        x_curr=np.array( [ kdl_fk_frame.p.x(), kdl_fk_frame.p.y(), kdl_fk_frame.p.z(), rpy_fk[0], rpy_fk[1], rpy_fk[2] ])
        print("x_curr:" ,x_curr)
        print("x_des:", x_des)
        #J_tip=np.array( [[kdl_J_tip[0,0],kdl_J_tip[0,1],kdl_J_tip[0,2],kdl_J_tip[0,3],kdl_J_tip[0,4],kdl_J_tip[0,5]],
         #                [kdl_J_tip[1,0],kdl_J_tip[1,1],kdl_J_tip[1,2],kdl_J_tip[1,3],kdl_J_tip[1,4],kdl_J_tip[1,5]],
          #               [kdl_J_tip[2,0],kdl_J_tip[2,1],kdl_J_tip[2,2],kdl_J_tip[2,3],kdl_J_tip[2,4],kdl_J_tip[2,5]],
           #              [kdl_J_tip[3,0],kdl_J_tip[3,1],kdl_J_tip[3,2],kdl_J_tip[3,3],kdl_J_tip[3,4],kdl_J_tip[3,5]],
            #             [kdl_J_tip[4,0],kdl_J_tip[4,1],kdl_J_tip[4,2],kdl_J_tip[4,3],kdl_J_tip[4,4],kdl_J_tip[4,5]],
             #            [kdl_J_tip[5,0],kdl_J_tip[5,1],kdl_J_tip[5,2],kdl_J_tip[5,3],kdl_J_tip[5,4],kdl_J_tip[5,5]]] )
        #print(kdl_J_tip)
        #J_tip=np.array( [[kdl_J_tip[0,0],kdl_J_tip[0,1],kdl_J_tip[0,2]],
        #                 [kdl_J_tip[1,0],kdl_J_tip[1,1],kdl_J_tip[1,2]],
        #                 [kdl_J_tip[2,0],kdl_J_tip[2,1],kdl_J_tip[2,2]],
        #                 [kdl_J_tip[3,0],kdl_J_tip[3,1],kdl_J_tip[3,2]],
        #                 [kdl_J_tip[4,0],kdl_J_tip[4,1],kdl_J_tip[4,2]],
        #                 [kdl_J_tip[5,0],kdl_J_tip[5,1],kdl_J_tip[5,2]]] )

        #J_tip=np.array( [[kdl_J_tip[0,0]],                                        
        #                 [kdl_J_tip[1,0]],                                         
        #                 [kdl_J_tip[2,0]],                                         
        #                 [kdl_J_tip[3,0]],                                         
        #                 [kdl_J_tip[4,0]],                                         
        #                 [kdl_J_tip[5,0]] ])

        #J_tip=np.array( [[kdl_J_tip[0,0],kdl_J_tip[0,1]],                                         
        #                 [kdl_J_tip[1,0],kdl_J_tip[1,1]],                                         
        #                 [kdl_J_tip[2,0],kdl_J_tip[2,1]],                                         
        #                 [kdl_J_tip[3,0],kdl_J_tip[3,1]],                                         
        #                 [kdl_J_tip[4,0],kdl_J_tip[4,1]],                                       
        #                 [kdl_J_tip[5,0],kdl_J_tip[5,1]] ])

        J_tip=np.zeros((6,q_in.rows()))
        for i in range(5):
            for j in range(q_in.rows()-1):
                J_tip[i,j]=kdl_J_tip[i,j]
        #print(J_tip)
        
        # Objective function for pyOpt                                                                             
        def ik(qdot):
            #qdot=np.array(qdot)
            f = LA.norm( np.matmul(J_tip,qdot)  - (x_des -x_curr) )
            g = qdot-1000
            fail=0
            return f,g,fail

        
        #print("FK position: ", x_fk[0], x_fk[1], x_fk[2], x_fk[3], x_fk[4], x_fk[5], )
        #print("J_tip", kdl_J_tip[0,0],kdl_J_tip[4,4])

        opt_prob = pyOpt.Optimization('IK velocity',ik)
        opt_prob.addObj('f')
        opt_prob.addVarGroup('qdot', q_in.rows(), 'c', lower=-100, upper=100, value=0)
        opt_prob.addConGroup('g',q_in.rows(),'i')

        #print opt_prob

        slsqp = pyOpt.SLSQP()
        slsqp.setOption('IPRINT', -1)

        [_, sol, _] = slsqp(opt_prob,sens_type='FD')
        del_q = Float64MultiArray()
        del_q.data=sol
        #print(sol)
        this.pub.publish(del_q)

        

rospy.init_node('ur5_rcm')
ur5=ur5_fk()
rospy.spin()
