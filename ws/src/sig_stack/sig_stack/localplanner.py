import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from sig_stack.utils import yaw_from_quaternion
from sig_stack.utils import quaternion_from_yaw
import numpy as np

class LocalPlanner(Node):
  def __init__(self):
    super().__init__('local_planner')

    # Publishes the chosen target waypoint to the pure pursuit controller
    self.publisher_ = self.create_publisher(Odometry, 'drive_to', 10)

    # Subscribes to the car's current pose (position + orientation) from localization
    self.subscription_ = self.create_subscription(Odometry, 'odom', self.pose_callback, 10)

    # Subscribes to the global planner's ordered list of waypoints along the race line
    self.global_planner_subscription_ = self.create_subscription(PoseArray, 'global_planner', self.global_planner_callback, 10)

    # Tracks the car's current position in the map frame
    self.currentX = 0.0
    self.currentY = 0.0
    self.currentVelocity = 0.0 #speed we are currently
    self.currentYaw = 0.0  # Heading angle in radians

    # Stores the most global path as a list of Pose objects
    #list
    self.gPath = None
    #numpy xy
    self.gPath_xy = None
    #numpy yaw
    self.gPath_yaw = None

    ##################
    #MPPI PARAMETERS

    #Number of trajectories per look
    self.K = 50
    #Size of Horizon per look (number of timesteps)
    self.T = 20
    #Seconds per timestep
    self.dt = .1
    #temp (controls how much we favor low cost trajectories)
    self.lamda = 1.0
    #std (standard deviation) of speed of noise
    self.sigma_v = .5
    #Std of yaw change rate noise
    self.sigma_w = .5
    #speed limits (what are my unites? m/s?)
    self.v_min, self.v_max = 0.0, 3.0
    #yaw change rate limits (What are my units? rad/s)
    self.w_min, self.w_max = -np.pi/2, np.pi/2
    #Cost weight for gPath distace
    self.w_dist = 1.0
    #Cost weight for heading error
    self.w_heading = .5
    #Cost weight for yaw rate effort
    self.w_ctrl = .1
    #steps ahead to project the output waypoint
    self.lookahead_step = 5

    #nominal control sequence
    self.U = np.zeros((self.T, 2))
    self.U[:, 0] = 1.0

  def global_planner_callback(self, msg):
    # Update the stored global path whenever the global planner publishes a new one
    self.gPath = msg.poses
    self.gPath_xy = np.array([(i.position.x, i.position.y) for i in msg.poses])
    self.gPath_yaw = np.array([yaw_from_quaternion(i.orientation.w, i.orientation.x, i.orientation.y, i.orientation.z) for i in msg.poses])


  def mppi_trajectory_generation(self):
    sample = np.random.randn(self.K, self.T, 2)
    sample[:, :, 0] *= self.sigma_v
    sample[:, :, 1] *= self.sigma_w

    perturbed = sample + self.U

    # Initialize K copies of current state
    x = np.full(self.K, self.currentX)
    y = np.full(self.K, self.currentY)
    theta = np.full(self.K, self.currentYaw)
    S = np.zeros(self.K)

    for t in range(self.T):
        v_t = perturbed[:, t, 0]   # (K,) speed for each sample at step t
        w_t = perturbed[:, t, 1]   # (K,) yaw rate for each sample at step t
        
        # unicycle dynamics — update x, y, theta
        x = x + v_t * np.cos(theta) * self.dt
        y = y + v_t * np.sin(theta) * self.dt
        theta = theta + w_t * self.dt

        # distance cost — dx, dy, min squared dist
        dx = x[:, np.newaxis] - self.gPath_xy[:, 0]
        dy = y[:, np.newaxis] - self.gPath_xy[:, 1]
        nearest = (dx**2 + dy**2).argmin(axis=1)
        dist = (dx**2 + dy**2).min(axis=1)

        heading = np.arctan2(np.sin(theta - self.gPath_yaw[nearest]), np.cos(theta - self.gPath_yaw[nearest]))

        ctrl = w_t ** 2
        
        S += self.w_dist * dist + self.w_heading * (heading ** 2) + self.w_ctrl * ctrl

    w = np.exp(-(S -S.min())/ self.lamda)
    w /= w.sum()
    self.U += (w[:, np.newaxis, np.newaxis] * sample).sum(axis=0)
    self.U[:, 0] = np.clip(self.U[:, 0], self.v_min, self.v_max)
    self.U[:, 1] = np.clip(self.U[:, 1], self.w_min, self.w_max)

    speed = self.U[0,0]

    self.U[:-1] = self.U[1:]

    # Initialize K copies of current state
    x = self.currentX
    y = self.currentY
    theta = self.currentYaw

    for t in range(self.lookahead_step):
        v_t = self.U[t, 0]   # (K,) speed for each sample at step t
        w_t = self.U[t, 1]   # (K,) yaw rate for each sample at step t
        
        # unicycle dynamics — update x, y, theta
        x = x + v_t * np.cos(theta) * self.dt
        y = y + v_t * np.sin(theta) * self.dt
        theta = theta + w_t * self.dt

    return (x, y, theta, speed)

  def location_callback(self):
    waypoint = self.mppi_trajectory_generation()

    # Pack the chosen waypoint into an Odometry message and publish it
    # so the pure pursuit controller knows where to steer toward
    msg = Odometry()
    msg.pose.pose.position.x = waypoint[0]
    msg.pose.pose.position.y = waypoint[1]

    q = quaternion_from_yaw(waypoint[2])
    msg.pose.pose.orientation.w = q[0]
    msg.pose.pose.orientation.x = q[1]
    msg.pose.pose.orientation.y = q[2]
    msg.pose.pose.orientation.z = q[3]
    msg.twist.twist.linear.x = waypoint[3]

    self.publisher_.publish(msg)

  def pose_callback(self, msg):
    # Update the car's current position and heading whenever odom publishes
    self.currentX = msg.pose.pose.position.x
    self.currentY = msg.pose.pose.position.y
    self.currentVelocity = msg.twist.twist.linear.x
    q = msg.pose.pose.orientation
    self.currentYaw = yaw_from_quaternion(q.w, q.x, q.y, q.z)

    # only run if we have received waypoints from the global planner (empty list is falsy)
    if self.gPath:
      self.location_callback()

def main(args=None):
  # Initialize the ROS 2 runtime
  rclpy.init(args=args)

  # Create and spin the local planner node, which processes callbacks until shutdown
  local_planner = LocalPlanner()
  rclpy.spin(local_planner)

  # Clean up on exit
  local_planner.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
