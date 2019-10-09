The goal of this project is to implement the RRT algorithm for sampling-based motion planning. We will make use of the MoveIt! software framework to help with some needed auxiliary calls, but the motion planning algorithm will be implemented as part of the homework.

The assignment4 folder will contain an assignment4 package, a robot simulator package, a cartesian control package, a forward kinematics package, a motion planning package, kuka robot description package and a urdf_package. Do not forget to catkin_make your workspace once you check out the assignment.

For this project, you will also have to install some other packages:

To install MoveIt!, use the following command:
sudo apt-get install ros-kinetic-moveit

Due to an inconsistency in the assimp package included with Ubuntu 16.04, you also need to run the following fix:
sudo apt-get update
sudo apt-get install python-pip
sudo -H pip install pyassimp --upgrade

Robot simulators:
For this assignment, we provide the Kuka LWR robot URDF. To start of the robot simulator, use one the following command:
roslaunch motion_planning kuka_lwr.launch

This will start a simulated robot, load the URDF to the parameter server, and start RVIZ with a proper configuration file. You will also see a familiar ring-and-arrows control, which you can drag to set the goal of the motion planner.

Starter code:
The starter code we have provided you does the following:

Reads the robot URDF from the parameter server and loads information about the joints of the robot.
Subscribes to the "/joint_states" topic in order to receive information about the current joint values for the robot (same as for the previous assignment)
Subscribes to the '/motion_planning_goal' topic in order to listen for commands for the motion planner to execute. 
Subscribes to the '/obstacle' topic in order to listen for which obstacle is currently being published in RVIZ.
Creates a publisher for the '/joint_trajectory' topic which will publish the commands for the robot to execute after motion planning.
Has a function to check if the state of the robot is valid.
Has a function which will perform inverse kinematics for you.

RRT Code:
The "motion_planning" function in the starter code. This function is a callback for the /motion_planning_goal topic of the data type "geometry_msgs/Transform". Each time the callback is invoked, you must plan and execute an appropriate joint trajectory using an RRT planner. Some more details:

The start point of motion planning is a goal expressed in joint space. The goal you receive is in end-effector space so you must perform IK on it to obtain a goal configuration in joint space. Our starter code provides you with a function that performs collision-aware IK. (Incidentally, this is done by using a MoveIt! service, but knowledge of this is not required for implementation.) 

Once you have a goal in joint space, implement the RRT algorithm as discussed in class and presented on the handout. The length of the branches you add to your tree should be around 0.1.

You will need a call that verifies if a specific set of joint values produces a "valid" (collision-free) configuration. Our starter code also provides a function to do that.

Note that, for the KUKA arm, each joint range is between -PI and PI. You will need to sample random joint values inside this entire range if the planner is to succeed with the more complicated obstacles.

After planning an appropriate path to the goal, you must publish it as a trajectory. You will publish the result on the topic "/joint_trajectory" of the type "trajectory_msgs/JointTrajectory". Use "rosmsg show JointTrajectory" to see what this message type looks like. The message you publish should be populated as follows:
the joint_names field should contain the names of your joints, in the same order as the joint values that make up your trajectory.
the points[] field should contain a list of waypoints in joint space. Each entry in this list is of the type "JointTrajectoryPoint". In each of these entries, you must only fill in the "positions" field with the joint values at that specific waypoint. Leave all the other fields (e.g.  velocities, accelerations, etc.) empty.
Note that, nominally, a trajectory should also contain timing information, as discussed in class. However, to simplify the assignment, we only ask you to fill in the position component of the trajectory.
Note that your path must always start from the current robot configuration. The starter code subscribes to the "/joint_states" topic and saves the starting configuration in the member variable self.q_current.
You must "shortcut" the path returned by the RRT. Do that by directly connecting any two waypoints in your path that can be connected via a collision-free straight line segment in joint space. 

After shortcutting, you must re-sample your trajectory in joint space. For each segment, add a number of internal samples equally spaced apart inside the segment. The number of internal samples for each segment should be the smallest possible such that no consecutive points on the path are farther than 0.5 to each other.
If you do not re-sample your trajectory, the robot will move a little bit and then immediately stop. Note that the number of internal samples you will add could be different for each segment. 

To run the starter code: 
rosrun assignment4 mp.py

Testing your node:
When you right-click on the rings-and-arrows control and select "Move Arm" your node will receive a goal to motion plan to the current position of the control. If your node does all its work correctly and publishes a valid trajectory, the arm will move to the goal.
You can add obstacles to the scene by right-clicking the rings-and-arrows control and selecting "Obstacles". You will have multiple choice of obstacles.
Your node must produce collision-free movement. Make sure to visually inspect your paths to make sure the robot does not go through the obstacle.

To run the Autograder, navigate to the assignment4 package and then use the commands: 
bash grade_assignment4.sh

