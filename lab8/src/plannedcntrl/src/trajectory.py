import math
import tf2_ros
import rospy
import tf
import numpy as np

import matplotlib.pyplot as plt

def plot_trajectory(waypoints):
    # Extracting x, y and theta values from the waypoints
    x_vals = [point[0] for point in waypoints]
    y_vals = [point[1] for point in waypoints]
    theta_vals = [point[2] for point in waypoints]
    
    # Plotting the trajectory
    plt.figure(figsize=(10, 6))
    plt.plot(x_vals, y_vals, '-o', label='Trajectory')
    
    # Plotting the start and end points
    plt.scatter(x_vals[0], y_vals[0], color='green', s=100, zorder=5, label='Start')
    plt.scatter(x_vals[-1], y_vals[-1], color='red', s=100, zorder=5, label='End')
    
    # Plotting orientation arrows along the trajectory
    for x, y, theta in waypoints:
        plt.arrow(x, y, 0.05 * np.cos(theta), 0.05 * np.sin(theta), head_width=0.01, head_length=0.005, fc='blue', ec='blue')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

def bezier_curve(p0, p1, p2, p3, t):
    """Calculate a point on a cubic Bezier curve defined by p0, p1, p2, and p3 at parameter t."""
    return ## TODO

def generate_bezier_waypoints(x1, y1, theta1, x2, y2, theta2, offset=1.0, num_points=10):
    # 1. Calculate direction vector based on yaw
    direction_start = np.array([np.cos(theta1), np.sin(theta1)])
    direction_end = np.array([-np.cos(theta2), -np.sin(theta2)])  # Opposite direction for the end point

    # 2. Determine control points based on yaw and offset
    control1 = np.array([x1, y1]) + offset * direction_start
    control2 = np.array([x2, y2]) + offset * direction_end

    # 3. Sample points along the Bezier curve
    t_values = np.linspace(0, 1, num_points)
    waypoints = [bezier_curve(np.array([x1, y1]), control1, control2, np.array([x2, y2]), t) for t in t_values]

    # 4. Determine orientation at each point
    thetas = []
    for i in range(len(waypoints) - 1):
        dx = waypoints[i + 1][0] - waypoints[i][0]
        dy = waypoints[i + 1][1] - waypoints[i][1]
        thetas.append(np.arctan2(dy, dx))
    thetas.append(thetas[-1])  # Repeat last orientation for the last waypoint

    waypoints_with_theta = [(waypoints[i][0], waypoints[i][1], thetas[i]) for i in range(len(waypoints))]

    return waypoints_with_theta

def plan_curved_trajectory(target_position):
    """
    Plan a curved trajectory for a Roomba-type robot from current_position to target_position using a Bezier curve.
    
    Parameters:
    - target_position: A tuple (x, y) representing in the robot base frame.
    
    Returns:
    - A list of waypoints [(x, y, theta), ...] where type can be 'rotate' or 'move' and value is the amount to rotate in radians or move in meters.
    """
    tfBuffer = ## TODO: initialize a buffer
    tfListener = ## TODO: initialize a transform listener
    while not rospy.is_shutdown():
        try:
            trans = ## TODO: apply a lookup transform between our world frame and turtlebot frame
            print(trans)
            break
        except:
            pass
    x1, y1 = trans.transform.translation.x, trans.transform.translation.y
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [trans.transform.rotation.x, trans.transform.rotation.y,
            trans.transform.rotation.z, trans.transform.rotation.w])
    
    x2 = ## TODO: how would you get x2 from our target position? remember this is relative to x1 
    y2 = ## TODO: how would you get x2 from our target position? remember this is relative to x1 

    waypoints = generate_bezier_waypoints(x1, y1, yaw, x2, y2, yaw, offset=0.2, num_points=10)
    plot_trajectory(waypoints)

    return waypoints

if __name__ == '__main__':
    rospy.init_node('turtlebot_controller', anonymous=True)
    # trajectory = plan_curved_trajectory()

    # For testing
    trajectory = generate_bezier_waypoints(0.0, 0.0, np.pi/2, 0.2, 0.2, np.pi/2, offset=0.2, num_points=100)
    plot_trajectory(trajectory)