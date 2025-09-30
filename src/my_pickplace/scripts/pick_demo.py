#!/usr/bin/env python3
import sys, copy, rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped

NODE = "pick_demo"
ARM_GROUP = "manipulator"
TABLE_NAME = "table"
BOX_NAME   = "part"

# 桌子 & 方塊參數 (相對 world)
TABLE_L, TABLE_W, TABLE_H = 1.00, 0.60, 0.04
TABLE_TOP_Z = 0.00
BOX_SIZE = 0.04
BOX_X, BOX_Y = 0.35, 0.00

APPROACH_Z = 0.10
LIFT_Z     = 0.12

def add_table_and_box(scene, base_frame):
    # 桌子
    table = PoseStamped()
    table.header.frame_id = base_frame
    table.pose.orientation.w = 1.0
    table.pose.position.x = 0.20
    table.pose.position.y = 0.00
    table.pose.position.z = TABLE_TOP_Z - TABLE_H/2.0
    scene.add_box(TABLE_NAME, table, size=(TABLE_L, TABLE_W, TABLE_H))

    # 方塊
    box = PoseStamped()
    box.header.frame_id = base_frame
    box.pose.orientation.w = 1.0
    box.pose.position.x = BOX_X
    box.pose.position.y = BOX_Y
    box.pose.position.z = TABLE_TOP_Z + BOX_SIZE/2.0
    scene.add_box(BOX_NAME, box, size=(BOX_SIZE, BOX_SIZE, BOX_SIZE))

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(NODE)

    scene = moveit_commander.PlanningSceneInterface()
    arm   = moveit_commander.MoveGroupCommander(ARM_GROUP)
    robot = moveit_commander.RobotCommander()

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    arm.set_planning_time(10.0)
    arm.set_num_planning_attempts(20)

    base_frame = arm.get_planning_frame()   # 在模擬裡通常是 "world"
    eef_link   = arm.get_end_effector_link() or "tool0"
    rospy.loginfo("planning_frame=%s, eef_link=%s", base_frame, eef_link)

    rospy.sleep(1.0)  # 等 scene 初始化
    add_table_and_box(scene, base_frame)
    rospy.sleep(1.0)
    rospy.loginfo("Added table & box into scene.")

    # 用當前 orientation，避免奇怪翻腕
    current_ori = arm.get_current_pose().pose.orientation
    def with_ori(p):
        pose = Pose()
        pose.position = copy.deepcopy(p.position)
        pose.orientation = current_ori
        return pose

    # target
    target = Pose()
    target.position.x = BOX_X
    target.position.y = BOX_Y
    target.position.z = TABLE_TOP_Z + BOX_SIZE/2.0

    # 1) approach
    approach = copy.deepcopy(target)
    approach.position.z += APPROACH_Z
    arm.set_pose_target(with_ori(approach))
    if arm.go(wait=True):
        rospy.loginfo("Step1: Approach success.")
    else:
        rospy.logwarn("Step1: Approach failed."); return
    arm.stop(); arm.clear_pose_targets()

    # 2) descend
    arm.set_pose_target(with_ori(target))
    if arm.go(wait=True):
        rospy.loginfo("Step2: Descend success.")
    else:
        rospy.logwarn("Step2: Descend failed, skip grasp."); return
    arm.stop(); arm.clear_pose_targets()

    # 3) attach (模擬夾取)
    scene.attach_box(eef_link, BOX_NAME, touch_links=robot.get_link_names())
    rospy.loginfo("Step3: Attached box.")

    # 4) lift
    lift = copy.deepcopy(target)
    lift.position.z += LIFT_Z
    arm.set_pose_target(with_ori(lift))
    if arm.go(wait=True):
        rospy.loginfo("Step4: Lift success.")
    else:
        rospy.logwarn("Step4: Lift failed.")
    arm.stop(); arm.clear_pose_targets()

    rospy.loginfo("Pick demo done.")
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()

