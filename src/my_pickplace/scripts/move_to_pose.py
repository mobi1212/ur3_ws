#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def main():
    # 初始化 MoveIt Commander 與 ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur3_move_to_pose", anonymous=True)

    # 控制 UR3 的 move group
    arm = moveit_commander.MoveGroupCommander("manipulator")

    # 建立一個目標姿態 Pose
    target_pose = Pose()
    target_pose.position.x = 0.3   # X座標 (公尺)
    target_pose.position.y = 0.0   # Y座標 (公尺)
    target_pose.position.z = 0.2   # Z座標 (公尺)
    target_pose.orientation.w = 1.0  # 朝向 (簡單設為無旋轉)

    # 設定目標並執行
    arm.set_pose_target(target_pose)
    success = arm.go(wait=True)

    # 停止並清除目標
    arm.stop()
    arm.clear_pose_targets()

    if success:
        rospy.loginfo("Move successful!")
    else:
        rospy.logwarn("Move failed!")

if __name__ == "__main__":
    main()
