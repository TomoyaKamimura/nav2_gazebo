"""
Sample python program for Gazebo simulated turtlebot3 in ROS2
Author : Tomoya Kamimura
Date : 2025/01/02
"""

import random

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException


def main(args=None):
    # initialize program
    rclpy.init(args=args)

    # create node for logging
    node = rclpy.create_node("nav2_gazebo")
    node.get_logger().info("nav2_gazebo initializing...")
    try:

        # set initial pose of tb3
        # position is (x, y) = (0.0, 0.0)
        navigator = BasicNavigator()
        init_pose = PoseStamped()
        init_pose.header.frame_id = "map"
        init_pose.header.stamp = navigator.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.z = 0.0
        init_pose.pose.orientation.w = 1.0
        navigator.setInitialPose(init_pose)
        navigator.waitUntilNav2Active()

        node.get_logger().info(
            "nav2_gazebo Initialized, use CTRL + C for exit program\n"
        )

        # start navigation loop
        task_count = 0
        while True:
            msg = "Task {} : Started".format(task_count)
            node.get_logger().info(msg)

            # generate random position and set it to tb3
            # range : x (-2.0 ~ 2.0), y (-2.0 ~ 2.0)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = random.uniform(-2.0, 2.0)
            goal_pose.pose.position.y = random.uniform(-2.0, 2.0)

            navigator.goToPose(goal_pose)

            # wait until the move to goal is complete
            index = 0
            while not navigator.isTaskComplete():
                index += 1
                # get feedback and show logs
                feedback = navigator.getFeedback()
                if feedback and index % 10 == 0:
                    # check timeout of navigation
                    # timeout : 30 secs
                    if Duration.from_msg(feedback.navigation_time) > Duration(
                        seconds=30
                    ):
                        msg = "Task {} : Timeouted".format(task_count)
                        node.get_logger().warning(msg)
                        navigator.cancelTask()

            # show result of navigating
            result = navigator.getResult()
            if result is TaskResult.SUCCEEDED:
                msg = "Task {} : Completed\n".format(task_count)
                node.get_logger().info(msg)
            else:
                msg = "Task {} : Incompleted\n.".format(task_count)
                node.get_logger().warning(msg)
            task_count += 1

    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("nav2_gazebo ended")


if __name__ == "__main__":
    main()
