#!/usr/bin/env python

import numpy
import random
import sys
import math
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########

        tree = []
        path = []
        none_point = RRTBranch(None,None)
        start_point = RRTBranch(none_point, numpy.array(self.q_current))

        tx = ee_goal.translation.x
        ty = ee_goal.translation.y
        tz = ee_goal.translation.z
        trans = tf.transformations.translation_matrix([tx, ty, tz])
        rx = ee_goal.rotation.x
        ry = ee_goal.rotation.y
        rz = ee_goal.rotation.z
        rw = ee_goal.rotation.w
        rot = tf.transformations.quaternion_matrix([rx, ry, rz, rw])
        T = numpy.dot(trans,rot)
        q_goal = numpy.array(self.IK(T))
        end_point = RRTBranch(none_point,q_goal)
        tree.append(start_point)

        while(True):
            random_q = self.random_number_generator()
            min_dist = 1000000000000000
            position = 0

            for i in range(len(tree)):
                dist = self.distance_between_two_points(tree[i].q, random_q)
                if dist < min_dist:
                    min_dist = dist
                    position = i

            h = self.is_segment_valid(tree[position].q, random_q)

            if h == True:
                vect = self.vector_between_two_points(tree[position].q,random_q)
                newp = numpy.add(tree[position].q, numpy.multiply(vect,0.1)) 
                tree.append(RRTBranch(tree[position], newp))
		print("Added point to tree")

            	v = self.is_segment_valid(tree[-1].q, end_point.q)

           	if v == True:
			end_point.parent = tree[-1]
                	tree.append(end_point)        #RRTBranch(tree[-1], end_point.q))
			tree = numpy.array(tree)
			print("Found end")
                	break

		else:
			print("Cant see goal, moving to next point")

	    else:
		print("Collision, going to next point")

	route = self.construct_tree_path(end_point, path, none_point)
	#routefinal  =  route[::-1]
	print (route)

	randomv = JointTrajectory()
	randomv.joint_names = self.joint_names
	lisst = []
	for i in range(1,len(route)):
		nodes = JointTrajectoryPoint()
		#lisst.append(nodes)
		nodes.positions = route[i]
		randomv.points.append(nodes)
	#randomv.points = lisst
	#randomv=list(randomv)
	print (randomv)						#print(x.tolist())
	self.pub.publish(randomv)


	


            

                




        ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid

    ''' MY NEW FUNCTIONS START HERE'''

    def random_number_generator(self):                                   
        new_random_point = numpy.zeros(self.num_joints)
        #print(new_random_point)
        for i in range(self.num_joints):
            r = random.uniform(-3.14159,3.14159)
            #print(r)
            new_random_point[i]+=r
            #print(new_random_point)
        return new_random_point


    def distance_between_two_points(self,a,b):
        add=0
        for i in range(self.num_joints):
            add = add + (b[i] - a[i])**2
            dist = numpy.sqrt(add)
        return dist


    def vector_between_two_points(self,a,b):
        vector=[]
        for i in range(self.num_joints):
            vector.append(b[i]-a[i])
        return numpy.array(vector)

            

    def is_segment_valid(self,a,b):
	j = 0.01
	vecto = self.vector_between_two_points(a,b)
	vecto1 = numpy.add(a, numpy.multiply(vecto, 0.01))
	while j < 1:
            vecto1 = numpy.add(vecto1, numpy.multiply(vecto, 0.01))
            torf = self.is_state_valid(vecto1)
	    if torf == True:
                return True
		j += 0.01
            else: 
                return False
		break

    def construct_tree_path(self, end_point, path, none_point):
	path.append(end_point.q)						## list()
	if end_point.parent == none_point.parent:
		return path[::-1]						###
	else:
		return self.construct_tree_path(end_point.parent, path, none_point)


	







'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

