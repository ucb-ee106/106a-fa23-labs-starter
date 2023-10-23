#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import tf
import sys
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from trajectory import plan_curved_trajectory

#Define the method which contains the main functionality of the node.
def controller(waypoint):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - goal_frame: the tf frame of the target AR tag
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  pub = ## TODO: what topic should we publish to? how?
  tfBuffer = ## TODO: initialize a buffer
  tfListener = ## TODO: initialize a tf listener
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz
  # you can also use the rate to calculate your dt, but you don't have to

  # All in the form [x, y]
  Kp = np.diag([2, 0.8]) ## TODO: You may need to tune these values for your turtlebot
  Kd = np.diag([-0.5, 0.5]) ## TODO: You may need to tune these values for your turtlebot
  Ki = np.diag([-0.1, 0.1]) ## TODO: You may need to tune these values for your turtlebot

  prev_time = ## TODO: initialize your time, what rospy function would be helpful here?
  integ = ## TODO: initialize an empty np array -- make sure to keep your sizes consistent
  derivative = ## TODO: initialize an empty np array 
  previous_error = ## TODO: initialize an empty np array 

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      trans_odom_to_base_link = ## TODO: create a transform between odom to base link

      (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion(
        [trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y,
            trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])


      waypoint_trans = ## TODO: initialize a PoseStamped
      waypoint_trans.pose.position.x = ## TODO: what value would you use here?
      waypoint_trans.pose.position.y = ## TODO: what value would you use here?
      waypoint_trans.pose.position.z = ## TODO: what value would you use here?  # Assuming the waypoint is on the ground

      quat = quaternion_from_euler() ## TODO: what would be the inputs to this function (there are 3)
      waypoint_trans.pose.orientation.x = ## TODO: what value would you use here?
      waypoint_trans.pose.orientation.y = ## TODO: what value would you use here?
      waypoint_trans.pose.orientation.z = ## TODO: what value would you use here?
      waypoint_trans.pose.orientation.w = ## TODO: what value would you use here?

      # Use the transform to compute the waypoint's pose in the base_link frame
      waypoint_in_base_link = do_transform_pose() ## TODO: what would be the inputs to this function (there are 2)
      (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [waypoint_in_base_link.pose.orientation.x, waypoint_in_base_link.pose.orientation.y,
            waypoint_in_base_link.pose.orientation.z, waypoint_in_base_link.pose.orientation.w])


      curr_time = ## TODO: get your time, what rospy function would be helpful here?

      # some debug output below
      print(f"Current: {trans_odom_to_base_link.transform.translation.x}, {trans_odom_to_base_link.transform.translation.y}, {baselink_yaw  }")
      print(f"Target: {waypoint}")

      # Process trans to get your state error
      # Generate a control command to send to the robot
      
      error = ## TODO: what are two values that we can use for this np.array, and what are the dimensions
      
      # proportional term
      proportional = np.dot(Kp, error).squeeze()
      
      # integral term
      dt = ## TODO: quick operation to determine dt
      integ += ## TODO: integral is summing up error over time, so what would we expect to add on to our integral term tracker here?
      integral = np.dot(Ki, integ).squeeze()

      # dervative term
      error_deriv = ## TODO: quick operation to determine dt
      derivative = np.dot(Kd, error_deriv).squeeze()

      msg = Twist()
      msg.linear.x = proportional[0] + derivative[0] + integral[0] 
      msg.angular.z = proportional[1] + derivative[1] + integral[1] 

      control_command = msg

      #################################### end your code ###############

      previous_error = ## TODO
      prev_time = ## TODO
      pub.publish(control_command)

      if ... : ##TODO: what is our stopping condition/how do we know to go to the next waypoint?
        return

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


def planning_callback(msg):
  try:
    trajectory = plan_curved_trajectory() ## TODO: What is the tuple input to this function?

    ## TODO: write a loop to loop over our waypoints and call the controller function on each waypoint

  except rospy.ROSInterruptException:
    pass
      

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  rospy.Subscriber() ## TODO: what are we subscribing to here?
  
  rospy.spin()