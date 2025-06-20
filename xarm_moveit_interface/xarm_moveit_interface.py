#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, RobotCommander, moveit_commander
import yaml
import os

class XArmMoveItInterface(Node):
    def __init__(self):
        super().__init__('xarm_moveit_interface')
        moveit_commander.roscpp_initialize([])
        self.robot = RobotCommander()
        self.group = MoveGroupCommander("xarm7")
        self.get_logger().info("xArm MoveIt interface initialized")

    def save_current_pose(self, filename="saved_pose.yaml"):
        pose = self.group.get_current_pose().pose
        with open(filename, "w") as f:
            yaml.dump({
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z
                },
                "orientation": {
                    "x": pose.orientation.x,
                    "y": pose.orientation.y,
                    "z": pose.orientation.z,
                    "w": pose.orientation.w
                }
            }, f)
        self.get_logger().info(f"Current pose saved to {filename}")

    def move_to_saved_pose(self, filename="saved_pose.yaml"):
        if not os.path.exists(filename):
            self.get_logger().error(f"File {filename} does not exist")
            return False
        with open(filename, "r") as f:
            data = yaml.safe_load(f)

        pose = Pose()
        pose.position.x = data["position"]["x"]
        pose.position.y = data["position"]["y"]
        pose.position.z = data["position"]["z"]
        pose.orientation.x = data["orientation"]["x"]
        pose.orientation.y = data["orientation"]["y"]
        pose.orientation.z = data["orientation"]["z"]
        pose.orientation.w = data["orientation"]["w"]

        self.group.set_pose_target(pose)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if success:
            self.get_logger().info("Moved to saved pose.")
        else:
            self.get_logger().error("Failed to move to pose.")
        return success

    def move_to_named_target(self, name):
        self.group.set_named_target(name)
        success = self.group.go(wait=True)
        self.group.stop()
        if success:
            self.get_logger().info(f"Moved to named target: {name}")
        else:
            self.get_logger().error(f"Failed to move to named target: {name}")
        return success


def main():
    rclpy.init()
    node = XArmMoveItInterface()

    import sys
    if len(sys.argv) == 2 and sys.argv[1] == 'save':
        node.save_current_pose()
    elif len(sys.argv) == 2 and sys.argv[1] == 'load':
        node.move_to_saved_pose()
    elif len(sys.argv) == 3 and sys.argv[1] == 'named':
        node.move_to_named_target(sys.argv[2])
    else:
        print("Usage:")
        print("  ros2 run xarm_moveit_interface xarm_moveit_interface save        # Save current pose")
        print("  ros2 run xarm_moveit_interface xarm_moveit_interface load        # Move to saved pose")
        print("  ros2 run xarm_moveit_interface xarm_moveit_interface named <name># Move to named target")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
