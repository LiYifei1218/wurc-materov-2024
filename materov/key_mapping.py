# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs import Joy
from geometry_msgs import Twist


class KeyMapping(Node):

    def __init__(self):
        super().__init__('key_mapping') # name of the node
        self.subscription = self.create_subscription(
            Joy, # message type of the topic
            'joy', # name of the topic
            self.listener_callback, # callback, executes when there's an update in topic
            10)
        
        self.subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Twist, # command message type
            'com_vel',
            10
        )
        self.get_logger().info("Key mapping initialized!!")

    def listener_callback(self, msg):
        self.get_logger().info('Joystick data received.')

        # Create a Twist message
        twist = Twist()

        # Map joystick axes to linear and angular velocities
        twist.linear.x = msg.axes[1]  # Forward/Backward
        twist.linear.y = msg.axes[0]  # Left/Right
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = msg.axes[3]  # Rotation

        # Publish the Twist message
        self.command_publisher.publish(twist)
        self.get_logger().info(f'Published Twist: {twist}')




def main(args=None):
    rclpy.init(args=args)

    key_mapping = KeyMapping()

    rclpy.spin(key_mapping)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_mapping.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
