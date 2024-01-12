#!/usr/bin/env python
"""
Chen Peng
"""
import rospy
import math
import sys
import geometry_msgs.msg
import moveit_commander
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Float64MultiArray


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = math.dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = math.fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= math.cos(tolerance / 2.0)

    return True


class MoveitServoZ1(object):
    def __init__(self, rate=30):
        super(MoveitServoZ1, self).__init__()
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_servo", anonymous=True, log_level=rospy.INFO)
        self.namespace = rospy.get_namespace()
        self.rate = rospy.Rate(rate)
        rospy.loginfo(rospy.get_name() + " Start")
        rospy.loginfo(rospy.get_name() + " Namespace: " + self.namespace)
        self.create_move_group()
        self.define_subscribers()
        rospy.spin()

    def define_subscribers(self):
        # command dependent on the servo configure file
        moveit_servo_command_topic = "/z1_joint_group_position_controller/command"
        rospy.Subscriber(
            moveit_servo_command_topic,
            Float64MultiArray,
            self.servo_command_callback,
        )

    def servo_command_callback(self, command: Float64MultiArray):
        # execute the joint command from servo
        # TODO: make sure this is preemptable
        # self.go_to_joint_state(command.data)

        # Calling ``stop()`` ensures that there is no residual movement
        # self.move_group.stop()

        # current_joints = self.move_group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, tolerance)
        self.move_group.go(command.data, wait=False)
        rospy.loginfo("go to joint states: " + str(command.data))

    def create_move_group(self):
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## This interface can be used to plan and execute motions:
        ## Make sure this is consistent with servo config
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Getting Basic Information
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("[MoveGroup] Planning frame: %s" % planning_frame)
        eef_link = move_group.get_end_effector_link()
        print("[MoveGroup] End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        print("[MoveGroup] Available Planning Groups:", robot.get_group_names())
        # print("[MoveGroup] Printing robot state")
        # print(robot.get_current_state())

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # Set vel and acc scale
        self.move_group.set_max_acceleration_scaling_factor(1)
        self.move_group.set_max_velocity_scaling_factor(1)


if __name__ == "__main__":
    MoveitServoZ1()
