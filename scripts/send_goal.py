#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from transforms3d.euler import euler2quat
from time import sleep
from math import pi


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(
            String,
            '/goal_request',
            self.goal_request_callback,
            10
        )
        self.goals_completed_pub = self.create_publisher(Bool, '/goals_completed', 10)
        self.goal_positions = {
            "tavolo": [0.71, -0.9, 0.0],
            "dispensa": [-0.18, -0.09, 0.0],
            "frigorifero": [1.9, -0.45, 0.0],
            "base": [1.85, 0.68, 0.0]
        }

        self.goal_orientations = {
            "tavolo": [0.0, 0.0, -0.7, 0.71],
            "dispensa": [0.0, 0.0, 1.0, 0.03],
            "frigorifero": [0.0, 0.0, -0.12, 1.0],
            "base":[0.0, 0.0, -0.71, 0.7]
        }

        self.goal_queue = []
        self.goal_active = False
        self.has_sent_any_goal = False

    def goal_request_callback(self, msg):
        nome_goal = msg.data.strip()
        self.get_logger().info(f"Richiesto goal: {nome_goal}")
        if nome_goal in self.goal_positions:
            self.goal_queue.append(nome_goal)
            if not self.goal_active:
                self.send_next_goal()
        else:
            self.get_logger().warn(f"Goal '{nome_goal}' non riconosciuto.")

    def send_next_goal(self):
        if not self.goal_queue:
            self.get_logger().info("Nessun goal in coda.")
            self.goal_active = False
            if self.has_sent_any_goal:
                msg = Bool()
                msg.data = True
                self.goals_completed_pub.publish(msg)
            return

        goal_name = self.goal_queue.pop(0)
        position = self.goal_positions[goal_name]
        orientation = self.goal_orientations[goal_name]
        self.has_sent_any_goal = True

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        qx, qy, qz, qw = orientation
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Inviando goal a: {goal_name}")
        self.goal_active = True

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rifiutato dal server Nav2.')
            self.goal_active = False
            self.send_next_goal()
            return

        self.get_logger().info('Goal accettato. Aspettando risultato...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status

        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("Goal raggiunto.")
            sleep(3)
        else:
            self.get_logger().warn(f"Goal fallito con status: {status}")

        self.goal_active = False
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

