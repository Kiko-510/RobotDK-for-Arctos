#!/usr/bin/env python3
import sys
import math
import rospy
import moveit_commander
import geometry_msgs.msg

VELOCITY_SCALING = 0.1
ACCELERATION_SCALING = 0.1

# ==============================
# Trajectory configuration
# ==============================
NUM_CIRCLES = 10
POINTS_PER_CIRCLE = 36

CENTER_X = 0.000
CENTER_Y = -0.45
CENTER_Z0 = 0.3

RADIUS0 = 0.05
DZ_PER_CIRCLE = 0.0004
DR_PER_CIRCLE = 0.0002

QX = 0.0
QY = 0.0
QZ = 0.0
QW = 1.0

RETURN_HOME = False


def generate_waypoints():
    waypoints = []

    for k in range(NUM_CIRCLES):
        radius = RADIUS0 + k * DR_PER_CIRCLE
        center_x = CENTER_X
        center_y = CENTER_Y
        center_z = CENTER_Z0 + k * DZ_PER_CIRCLE

        # 在 x-y 平面画圆，当前层 z 固定
        for i in range(POINTS_PER_CIRCLE + 1):
            theta = 2.0 * math.pi * i / POINTS_PER_CIRCLE

            x = center_x + radius * math.cos(theta)
            y = center_y + radius * math.sin(theta)
            z = center_z

            waypoints.append((x, y, z, QX, QY, QZ, QW))

    return waypoints


WAYPOINTS = generate_waypoints()


def make_pose(x, y, z, qx, qy, qz, qw):
    p = geometry_msgs.msg.Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def run_trajectory():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("arctos_trajectory_runner", anonymous=True)

    group = moveit_commander.MoveGroupCommander("arm")
    group.set_max_velocity_scaling_factor(VELOCITY_SCALING)
    group.set_max_acceleration_scaling_factor(ACCELERATION_SCALING)
    group.set_planning_time(5.0)
    group.allow_replanning(True)

    rospy.loginfo("RViz trajectory test started")
    rospy.loginfo(f"Planning frame: {group.get_planning_frame()}")
    rospy.loginfo(f"End effector: {group.get_end_effector_link()}")
    rospy.loginfo(f"Total waypoints: {len(WAYPOINTS)}")

    for i, wp in enumerate(WAYPOINTS):
        rospy.loginfo(
            f"Waypoint {i+1}/{len(WAYPOINTS)}: "
            f"x={wp[0]:.4f}, y={wp[1]:.4f}, z={wp[2]:.4f}"
        )

        target = make_pose(*wp)
        group.set_start_state_to_current_state()
        group.set_pose_target(target)

        success = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        if not success:
            rospy.logerr("MoveIt planning/execution failed.")
            return

        rospy.sleep(0.2)

    rospy.loginfo("Done.")


if __name__ == "__main__":
    try:
        run_trajectory()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nAborted by user.")