from dataclasses import dataclass
from typing import List, Tuple
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray

WHEELBASE = 0.57

MIN_STEERING = -0.5
MAX_STEERING = 0.5

MIN_VELOCITY = 0.5
MAX_VELOCITY = 1.0

MAPPING_FREQUENCY = 10
STEP_DURATION_S = 6

Pose = Tuple[float, float, float]
Command = Tuple[float, float]

@dataclass
class StampedPose:
    time_ns: float
    pose: Pose

@dataclass
class DriveStep:
    start_time_ns: float
    initial_pose: StampedPose
    command: Command
    planned_poses: List[StampedPose]
    ground_truth_poses: List[StampedPose]


class WishDrive(Node):
    def __init__(self):
        super().__init__("wish_drive_node")

        self.ground_truth_sub = self.create_subscription(
            Odometry,
            "icp_odom",
            self.ground_truth_callback,
            10
        )

        self.command_pub = self.create_publisher(
            AckermannDriveStamped,
            "drive",
            10
        )
        self.visualize_step_pub = self.create_publisher(
            MarkerArray,
            "planned_path",
            10
        )

        self.step_timer = self.create_timer(
            1.0 / 40.0,
            self.step_timer_callback
        )

        self.step: DriveStep | None = None
        self.last_pose: StampedPose | None = None

        self.get_logger().info("Wish drive is initialized")
    

    def start_new_step(self):
        if self.last_pose is None:
            self.get_logger().error("Cannot start a step without an initial pose received")
            return 

        command = self.generate_random_command()
        planned_poses = self.compute_planned_poses(self.last_pose, command)

        self.step = DriveStep(
            start_time_ns=self.get_clock().now().nanoseconds,
            initial_pose=self.last_pose,
            command=self.generate_random_command(),
            planned_poses=planned_poses,
            ground_truth_poses=[]
        )


    def generate_random_command(self) -> Command:
        speed = np.random.uniform(MIN_VELOCITY, MAX_VELOCITY)
        steering = np.random.uniform(MIN_STEERING, MAX_STEERING)

        return speed, steering


    def compute_planned_poses(self, initial_pose: StampedPose, command: Command) -> List[StampedPose]:
        dt = 1.0 / MAPPING_FREQUENCY
        speed, steering = command

        stamped_poses = []
        current_pose = initial_pose
        for _ in range(int(STEP_DURATION_S * MAPPING_FREQUENCY)):
            x, y, theta = current_pose.pose

            x += speed * np.cos(theta) * dt
            y += speed * np.sin(theta) * dt
            theta += (speed / WHEELBASE) * np.tan(steering) * dt

            current_pose = StampedPose(
                time_ns=current_pose.time_ns + dt * 1e9,
                pose=(x, y, theta)
            )
            stamped_poses.append(current_pose)

        return stamped_poses


    def ground_truth_callback(self, odom: Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        theta = odom.pose.pose.orientation.z

        stamp_ns = odom.header.stamp.nanosec + odom.header.stamp.sec * 1e9
        stamped_pose = StampedPose(
            time_ns=stamp_ns,
            pose=(x, y, theta)
        )

        self.last_pose = stamped_pose

        # No step is currently running
        if self.step is None:
            return

        self.step.ground_truth_poses.append(stamped_pose)


    def step_timer_callback(self):
        if self.step is None:
            return

        current_time_ns = self.get_clock().now().nanoseconds
        if current_time_ns - self.step.start_time_ns > STEP_DURATION_S * 1e9:
            self.export_step_data()
            self.step = None
            return
        
        # TODO: Publish command

    
    def export_step_data(self):
        if self.step is None:
            return
        
        self.get_logger().info(f"Step data exported")

    
    def visualize_step(self, step: DriveStep):
        if step is None:
            return
        
        planned_poses_msg = MarkerArray()
        planned_poses_msg.markers = []

        for planned_pose in step.planned_poses:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "planned_path"
            marker.id = len(planned_poses_msg.markers)
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = planned_pose.pose[0]
            marker.pose.position.y = planned_pose.pose[1]
            marker.pose.orientation.z = planned_pose.pose[2]
            marker.scale.x = 0.1
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            planned_poses_msg.markers.append(marker)
        
        self.visualize_step_pub.publish(planned_poses_msg)


def main(args=None):
    rclpy.init(args=args)

    wish_drive_node = WishDrive()

    rclpy.spin(wish_drive_node)

    wish_drive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()