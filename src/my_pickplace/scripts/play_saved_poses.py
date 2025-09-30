#!/usr/bin/env python3
import os
import sys
import rospy
import yaml
import moveit_commander
from geometry_msgs.msg import Pose

POSE_DIR = os.path.expanduser("~/simur3_ws/poses")

def load_pose(filename):
    """從 YAML 檔讀取 Pose"""
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
    return pose

def main():
    if len(sys.argv) < 2:
        print("用法: rosrun my_pickplace play_pose.py <pose編號>")
        return

    target_num = int(sys.argv[1])
    pose_file = os.path.join(POSE_DIR, f"pose{target_num}.yaml")

    if not os.path.exists(pose_file):
        print(f"找不到 {pose_file}")
        return

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("play_pose", anonymous=True)

    arm = moveit_commander.MoveGroupCommander("manipulator")
    arm.set_max_velocity_scaling_factor(0.3)
    arm.set_max_acceleration_scaling_factor(0.3)

    pose = load_pose(pose_file)
    rospy.loginfo(f"Moving to pose{target_num}")

    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose)
    arm.go(wait=True)

    arm.stop()
    arm.clear_pose_targets()
    rospy.loginfo("Done.")

if __name__ == "__main__":
    main()

