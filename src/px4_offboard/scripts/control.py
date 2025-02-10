#!/usr/bin/env python3
import sys
import rclpy
import geometry_msgs.msg
import std_msgs.msg
import termios
import tty
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher per il movimento e l'arming
        self.pub_twist = self.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
        self.arm_pub = self.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

        # Subscriber per la posizione dell'ArUco
        self.aruco_sub = self.create_subscription(
            geometry_msgs.msg.Pose,
            '/aruco_pose',
            self.aruco_callback,
            qos_profile
        )

        self.arm_toggle = False
        self.aruco_detected = False
        self.aruco_position = None
        self.speed = 0.5

    def aruco_callback(self, msg):
        """ Callback quando il marker ArUco viene rilevato. """
        self.get_logger().info(f"ArUco trovato a: X={msg.position.x}, Y={msg.position.y}, Z={msg.position.z}")
        self.aruco_detected = True
        self.aruco_position = msg

    def move_to_aruco(self):
        """ Muove il drone verso il marker ArUco e si ferma a 1 metro di distanza. """
        if not self.aruco_detected or not self.aruco_position:
            self.get_logger().info("⚠️ Nessun ArUco rilevato, attendo...")
            return

        target_x = self.aruco_position.position.x
        target_y = self.aruco_position.position.y

        # Calcola la posizione finale a 1 metro dal marker
        offset_x = -1.0 if target_x > 0 else 1.0
        offset_y = -1.0 if target_y > 0 else 1.0

        target_x += offset_x
        target_y += offset_y

        self.get_logger().info(f"Spostamento verso: X={target_x}, Y={target_y}")

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = float(target_x * self.speed)
        twist.linear.y = float(target_y * self.speed)

        self.pub_twist.publish(twist)
        time.sleep(3)  # Attendi 3 secondi per il movimento

        # Ferma il drone
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self.pub_twist.publish(twist)
        self.get_logger().info("✅ Drone posizionato a 1 metro dal marker.")

    def keyboard_control(self):
        """ Controllo manuale con la tastiera. """
        msg = """
        Controllo drone con tastiera:
        W: Salire | S: Scendere | A: Yaw Sinistra | D: Yaw Destra
        ↑ : Avanti | ↓ : Indietro | ← : Sinistra | → : Destra
        SPACE: Arma/Disarma il drone
        CTRL+C: Esci
        """
        print(msg)

        moveBindings = {
            'w': (0.0, 0.0, 1.0, 0.0),  # Z+ (Up)
            's': (0.0, 0.0, -1.0, 0.0), # Z- (Down)
            'a': (0.0, 0.0, 0.0, -1.0), # Yaw Left
            'd': (0.0, 0.0, 0.0, 1.0),  # Yaw Right
            '\x1b[A': (0.0, 1.0, 0.0, 0.0),  # Forward
            '\x1b[B': (0.0, -1.0, 0.0, 0.0), # Backward
            '\x1b[C': (-1.0, 0.0, 0.0, 0.0), # Right
            '\x1b[D': (1.0, 0.0, 0.0, 0.0),  # Left
        }

        settings = saveTerminalSettings()
        try:
            while True:
                key = getKey(settings)

                if key in moveBindings:
                    x, y, z, th = moveBindings[key]
                else:
                    x, y, z, th = 0.0, 0.0, 0.0, 0.0  # Stop movement when no key is pressed
                    if key == '\x03':  # CTRL+C per uscire
                        break

                if key == ' ':
                    self.arm_toggle = not self.arm_toggle
                    arm_msg = std_msgs.msg.Bool()
                    arm_msg.data = self.arm_toggle
                    self.arm_pub.publish(arm_msg)
                    print(f"🚀 Drone {'ARMED' if self.arm_toggle else 'DISARMED'}")

                    if self.arm_toggle:
                        self.get_logger().info("🔍 Cercando ArUco marker...")
                        time.sleep(2)
                        self.move_to_aruco()

                twist = geometry_msgs.msg.Twist()
                twist.linear.x = float(x * self.speed)
                twist.linear.y = float(y * self.speed)
                twist.linear.z = float(z * self.speed)
                twist.angular.z = float(th)
                self.pub_twist.publish(twist)

        except Exception as e:
            print(f"Error: {e}")

        finally:
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            self.pub_twist.publish(twist)
            restoreTerminalSettings(settings)
            rclpy.shutdown()

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':  
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
    rclpy.init()
    drone = DroneControl()
    drone.keyboard_control()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
