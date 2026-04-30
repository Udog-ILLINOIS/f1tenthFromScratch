import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from sig_stack.utils import yaw_from_quaternion

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

    # Defines an annular (ring-shaped) look-ahead zone around the car.
    # Only waypoints within [minRadius, maxRadius] meters are considered as targets.
    self.minRadius = 0.25  # Too close — car may already be past this point
    self.maxRadius = 1.5   # Too far — pure pursuit loses accuracy at large look-ahead

    # Maximum allowed heading difference (in radians) between the car and a candidate waypoint.
    # Filters out waypoints that require a sharp turn, preferring smooth forward progress.
    self.maxAngle = math.pi / 4  # 45 degrees

    # Stores the most global path as a list of Pose objects
    self.poses = []

  def global_planner_callback(self, msg):
    # Update the stored global path whenever the global planner publishes a new one
    self.poses = msg.poses

  def location_callback(self):
    # Filter the global path down to waypoints inside the look-ahead annulus
    poseList = []
    #this works but will iterate through the entire lobal line
    for pose in self.poses:
      # Compute squared distance from the car to this waypoint
      r = pow(pose.position.x - self.currentX, 2) + pow(pose.position.y - self.currentY, 2)
      if r <= self.maxRadius and r >= self.minRadius:
        poseList.append(pose)

    # Default to the first candidate in case none pass the heading filter below
    #if all waypoints reqire steep turns we defualt to [0]
    waypoint = poseList[0]

    # Among the filtered candidates, find the first one whose heading
    # is close enough to the car's current heading. This prefers waypoints
    # that don't require a sharp turn, helping the car stay on the race line
    # and avoiding the planner cutting corners or reversing direction.
    for pose in poseList:
      new_yaw = yaw_from_quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
      if abs(new_yaw - self.currentYaw) < self.maxAngle:
        waypoint = pose
        break

    # Pack the chosen waypoint into an Odometry message and publish it
    # so the pure pursuit controller knows where to steer toward
    msg = Odometry()
    msg.pose.pose.position.x = waypoint.position.x
    msg.pose.pose.position.y = waypoint.position.y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation = waypoint.orientation
    msg.twist.twist.linear.x = 1 #This is terrible. Just a placeholder NOTE: THIS NEEDS TO BE FIXED

    self.publisher_.publish(msg)

  def pose_callback(self, msg):
    # Update the car's current position and heading whenever odom publishes
    self.currentX = msg.pose.pose.position.x
    self.currentY = msg.pose.pose.position.y
    self.currentVelocity = msg.twist.twist.linear.x
    q = msg.pose.pose.orientation
    self.currentYaw = yaw_from_quaternion(q.w, q.x, q.y, q.z)

    # only run if we have received waypoints from the global planner (empty list is falsy)
    if self.poses:
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
