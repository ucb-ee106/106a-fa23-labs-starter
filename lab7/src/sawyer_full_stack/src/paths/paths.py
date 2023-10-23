#!/usr/bin/env python

"""
Starter script for lab1. 
Author: Chris Correa
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

from utils.utils import *

try:
    import rospy
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
except:
    pass

class MotionPath:
    def __init__(self, limb, kin, ik_solver, trajectory):
        """
        Parameters
        ----------
        limb : :obj:`baxter_interface.Limb` or :obj:`intera_interface.Limb`
        kin : :obj:`baxter_pykdl.baxter_kinematics` or :obj:`sawyer_pykdl.sawyer_kinematics`
            must be the same arm as limb
        trajectory: Trajectory object (see trajectories.py)
        total_time : float
            number of seconds you wish the trajectory to run for
        """
        self.limb = limb
        self.kin = kin
        self.ik_solver = ik_solver
        self.trajectory = trajectory
        self.previous_computed_ik = get_joint_positions(self.limb)

    def to_robot_trajectory(self, num_waypoints=300, jointspace=True, extra_points = 10):
        """
        Parameters
        ----------
        num_waypoints : float
            how many points in the :obj:`moveit_msgs.msg.RobotTrajectory`
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  
        """
        traj = JointTrajectory()
        traj.joint_names = self.limb.joint_names()
        points = []
        for t in np.linspace(0, self.trajectory.total_time, num=num_waypoints):
            point = self.trajectory_point(t, jointspace)
            points.append(point)

        # We want to make a final point at the end of the trajectory so that the 
        # controller has time to converge to the final point.
        for i in range(1, extra_points+1):
            extra_point = self.trajectory_point(self.trajectory.total_time, jointspace)
            extra_point.time_from_start = rospy.Duration.from_sec(self.trajectory.total_time + i)
            points.append(extra_point)

        traj.points = points
        traj.header.frame_id = 'base'
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def ik_service_client(self, final_pose, *args, **kwargs):
        """IK solver from Lab 5

        Args:
            final_pose (List[7]): xyz, quat for final position

        Returns:
            np.ndarray: joint angles for the robot
        """
        service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
        ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        header = Header(stamp=rospy.Time.now(), frame_id='base')

        # Create a PoseStamped and specify header (specifying a header is very important!)
        pose_stamped = PoseStamped()
        pose_stamped.header = header

        # Set end effector position
        pose_stamped.pose.position.x = final_pose[0]
        pose_stamped.pose.position.y = final_pose[1]
        pose_stamped.pose.position.z = final_pose[2]
        
        # Set end effector quaternion
        pose_stamped.pose.orientation.x = final_pose[3]
        pose_stamped.pose.orientation.y = final_pose[4]
        pose_stamped.pose.orientation.z = final_pose[5]
        pose_stamped.pose.orientation.w = final_pose[6]

        # Add desired pose for inverse kinematics
        ik_request.pose_stamp.append(pose_stamped)
        # Request inverse kinematics from base to "right_hand" link
        ik_request.tip_names.append('right_hand')

        try:
            rospy.wait_for_service(service_name, 5.0)
            response = ik_service_proxy(ik_request)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return

        # Check if result valid, and type of seed ultimately used to get solution
        if (response.result_type[0] > 0):
            # rospy.loginfo("SUCCESS!")
            # Format solution into Limb API-compatible dictionary
            return np.array(response.joints[0].position)
            
        return None

    def trajectory_point(self, t, jointspace):
        """
        takes a discrete point in time, and puts the position, velocity, and
        acceleration into a ROS JointTrajectoryPoint() to be put into a 
        RobotTrajectory.  
        
        Parameters
        ----------
        t : float
        jointspace : bool
            What kind of trajectory.  Joint space points are 7x' and describe the
            angle of each arm.  Workspace points are 3x', and describe the x,y,z
            position of the end effector.  

        Returns
        -------
        :obj:`trajectory_msgs.msg.JointTrajectoryPoint`


        joint_names: [left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2]
        points: 
        - 
        positions: [-0.11520713 -1.01663718 -1.13026189  1.91170776  0.5837694   1.05630898  -0.70543966]

        """
        point = JointTrajectoryPoint()
        delta_t = .01
        if jointspace:
            theta_t_2 = self.ik_service_client(self.trajectory.target_pose(max(t-2*delta_t, 0)))            
            theta_t_1 = self.ik_service_client(self.trajectory.target_pose(max(t-delta_t, 0)))
            theta_t   = self.ik_service_client(self.trajectory.target_pose(t))
            
            # we said you shouldn't simply take a finite difference when creating
            # the path, why do you think we're doing that here?
            point.positions = theta_t
            point.velocities = (theta_t - theta_t_1) / delta_t
            point.accelerations = (theta_t - 2*theta_t_1 + theta_t_2) / (delta_t**2)
            self.previous_computed_ik = theta_t
        else:
            point.positions = self.trajectory.target_pose(t)
            point.velocities = self.trajectory.target_velocity(t)
        point.time_from_start = rospy.Duration.from_sec(t)
        return point
