#!/usr/bin/env python
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from pan_tilt_msgs.msg import PanTiltCmdDeg

delta_value = 2
pan_tilt_yaw = 0.0
pan_tilt_pitch = 0.0

publisher = 0

class pan_tilt_control_node(Node):
    def __init__(self):
        super().__init__('pan_tilt_control_node')
        self.publisher_ = self.create_publisher(PanTiltCmdDeg, 'pan_tilt_cmd_deg', 10)
        self.joy_subscriber_ = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, data):
        global pan_tilt_yaw, pan_tilt_pitch

        # button Y
        if data.buttons[0]==1:
          pan_tilt_pitch -= delta_value
          # rospy.loginfo("up")

        # button B
        if data.buttons[1]==1:
          pan_tilt_yaw -= delta_value
          # rospy.loginfo("right")

        # button A
        if data.buttons[2]==1:
          pan_tilt_pitch += delta_value
          # rospy.loginfo("down")

        # button X
        if data.buttons[3]==1:
          pan_tilt_yaw += delta_value
          # rospy.loginfo("left")

        # button RB
        if data.buttons[5]==1:
          pan_tilt_pitch = 0
          pan_tilt_yaw = 0
          # rospy.loginfo("reset")

        if(pan_tilt_pitch > 60):
          pan_tilt_pitch = 60
        if(pan_tilt_pitch < -60):
          pan_tilt_pitch = -60

        if(pan_tilt_yaw > 60):
          pan_tilt_yaw = 60
        if(pan_tilt_yaw < -60):
          pan_tilt_yaw = -60

        command = PanTiltCmdDeg()
        command.speed = 20
        command.yaw = pan_tilt_yaw * 1.0
        command.pitch = pan_tilt_pitch * 1.0

        self.publisher_.publish(command)



def main(args=None):
  rclpy.init(args=args)
  node = pan_tilt_control_node()
  try:
      node.get_logger().info("PanTilt Control Start")
      rclpy.spin(node)
  except KeyboardInterrupt:
      node.get_logger().info("Shutting down the node")

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
