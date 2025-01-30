#!/usr/bin/env python3
import sys

import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    's': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
    '\x1b[B' : (0, -1, 0, 0), #Down Arrow
    '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
    '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
}

def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == '\x1b':
            additional_chars = sys.stdin.read(2)
            key += additional_chars
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    settings = saveTerminalSettings()
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    pub_twist = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
    pub_pose = node.create_publisher(geometry_msgs.msg.Pose, '/drone_pose', qos_profile)
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

    arm_toggle = False

    speed = 0.5
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if key == ' ':
                arm_toggle = not arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                print(f"Arm toggle is now: {arm_toggle}")

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.z = th
            pub_twist.publish(twist)

            # Publish drone's pose
            pose = geometry_msgs.msg.Pose()
            pose.position.x = x_val
            pose.position.y = y_val
            pose.position.z = z_val
            pose.orientation.z = yaw_val  # Set yaw or use a proper quaternion for orientation
            pub_pose.publish(pose)

            print("Twist -> X:", twist.linear.x, "Y:", twist.linear.y, "Z:", twist.linear.z, "Yaw:", twist.angular.z)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        pub_twist.publish(twist)

        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()
