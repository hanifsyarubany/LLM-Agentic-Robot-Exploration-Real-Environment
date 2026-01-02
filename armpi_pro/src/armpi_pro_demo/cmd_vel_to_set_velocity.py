#!/usr/bin/env python3
# coding: utf-8

import sys
import math
import rospy
from geometry_msgs.msg import Twist
from chassis_control.msg import SetVelocity


class CmdVelToSetVelocity(object):
    def __init__(self):
        # Parameters: tune these to match your sim speeds
        # Max chassis linear speed (Hiwonder units, mm/s)
        self.max_linear_mm_s = rospy.get_param('~max_linear_mm_s', 100.0)
        # Expected max |linear| from /cmd_vel in m/s (from your navigation stack)
        self.max_cmd_lin = rospy.get_param('~max_cmd_lin', 0.5)

        # Max yaw rate command for Hiwonder (dx)
        self.max_yaw_cmd = rospy.get_param('~max_yaw_cmd', 0.8)
        # Expected max |angular.z| from /cmd_vel in rad/s
        self.max_cmd_wz = rospy.get_param('~max_cmd_wz', 1.0)

        self.last_heading_deg = 90.0  # default "forward"

        self.pub = rospy.Publisher(
            '/chassis_control/set_velocity',
            SetVelocity,
            queue_size=1
        )
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("cmd_vel_to_set_velocity node started.")

    def cmd_vel_cb(self, msg: Twist):
        # /cmd_vel in base_link frame:
        # linear.x [m/s] forward, linear.y [m/s] left, angular.z [rad/s] +CCW
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        # ---- Translational part: map (vx, vy) -> (speed_mm_s, heading_deg) ----
        speed_m = math.hypot(vx, vy)  # magnitude in m/s

        if speed_m < 1e-3:
            # Essentially no translation
            linear_mm_s = 0.0
        else:
            # Hiwonder convention:
            # 0° = right, 90° = forward, 180° = left, 270° = back
            # ROS base_link: x forward, y left
            # A mapping that satisfies all four cardinal directions:
            # heading = atan2(vx, -vy)
            heading_rad = math.atan2(vx, -vy)
            heading_deg = math.degrees(heading_rad)
            if heading_deg < 0.0:
                heading_deg += 360.0
            self.last_heading_deg = heading_deg

            # Scale linear speed so that |/cmd_vel| = max_cmd_lin -> max_linear_mm_s
            if self.max_cmd_lin > 0:
                scale_lin = self.max_linear_mm_s / self.max_cmd_lin
                linear_mm_s = speed_m * scale_lin
            else:
                # fallback: direct m/s -> mm/s then clamp
                linear_mm_s = speed_m * 1000.0

            # clamp to safe range
            if linear_mm_s > self.max_linear_mm_s:
                linear_mm_s = self.max_linear_mm_s

        # ---- Rotational part: map angular.z -> dx ----
        if abs(wz) < 1e-3:
            dx = 0.0
        else:
            if self.max_cmd_wz > 0:
                # Positive angular.z in ROS = CCW (left)
                # Docs say positive dx = clockwise, so we flip sign.
                scale_yaw = self.max_yaw_cmd / self.max_cmd_wz
                dx = -wz * scale_yaw
            else:
                dx = 0.0

            # clamp
            if dx > self.max_yaw_cmd:
                dx = self.max_yaw_cmd
            elif dx < -self.max_yaw_cmd:
                dx = -self.max_yaw_cmd

        # If everything is basically zero, send a clean stop
        if speed_m < 1e-3 and abs(wz) < 1e-3:
            linear_mm_s = 0.0
            dx = 0.0

        # Publish as positional arguments (same style as Hiwonder example)
        # SetVelocity fields are (linear_mm_s, heading_deg, dx)
        self.pub.publish(linear_mm_s, self.last_heading_deg, dx)

    def on_shutdown(self):
        rospy.loginfo("Shutting down cmd_vel_to_set_velocity, sending stop...")
        try:
            self.pub.publish(0.0, self.last_heading_deg, 0.0)
            rospy.sleep(0.1)
        except rospy.ROSException:
            pass


if __name__ == '__main__':
    if sys.version_info[0] < 3:
        print("Please run this node with Python3!")
        sys.exit(0)

    rospy.init_node('cmd_vel_to_set_velocity', log_level=rospy.DEBUG)
    node = CmdVelToSetVelocity()
    rospy.spin()
