from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        self.get_logger().info("Inizializzazione del nodo DroneControl.")

        # Sottoscrizione al topic per i comandi del drone
        self.command_sub = self.create_subscription(
            Float32,
            'drone_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Sottoscritto al topic 'drone_command'.")

        # Sottoscrizione alla distanza del marker
        self.distance_sub = self.create_subscription(
            Float32,
            'marker_distance',
            self.distance_callback,
            10
        )
        self.get_logger().info("Sottoscritto al topic 'marker_distance'.")

        # Sottoscrizione alla posizione del drone
        self.drone_pose_sub = self.create_subscription(
            Pose,
            '/drone/pose',
            self.drone_pose_callback,
            10
        )
        self.get_logger().info("Sottoscritto al topic '/drone/pose'.")

        # Sottoscrizione alla posizione del parallelepipedo
        self.cube_pose_sub = self.create_subscription(
            Pose,
            '/parallelepiped/pose',
            self.cube_pose_callback,
            10
        )
        self.get_logger().info("Sottoscritto al topic '/parallelepiped/pose'.")

        # Publisher per i comandi di movimento del drone
        self.velocity_pub = self.create_publisher(Twist, 'drone/cmd_vel', 10)
        self.get_logger().info("Publisher creato per 'drone/cmd_vel'.")

        # Stato per il controllo della distanza
        self.target_reached = False
        self.get_logger().info("Nodo di controllo drone avviato.")

    def command_callback(self, msg):
        self.get_logger().info(f"Comando ricevuto: {msg.data}.")
        if msg.data > 0:
            self.move_drone_towards_object()

    def distance_callback(self, msg):
        self.get_logger().info(f"Distanza dal marker: {msg.data} metri.")
        if msg.data <= 1.0:
            self.target_reached = True
            self.stop_drone()
        else:
            self.target_reached = False
            self.move_drone_towards_object()

    def drone_pose_callback(self, pose):
        self.get_logger().info(f"Posizione del drone: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}.")

    def cube_pose_callback(self, pose):
        self.get_logger().info(f"Posizione del parallelepipedo: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}.")

    def move_drone_towards_object(self):
        if self.target_reached:
            self.get_logger().info("Il drone è già vicino al marker. Non è necessario avvicinarsi ulteriormente.")
            return

        self.get_logger().info("Inizio del movimento del drone verso l'oggetto.")
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.5  # Velocità in avanti
        velocity_msg.linear.y = 0.0
        velocity_msg.linear.z = 0.0
        velocity_msg.angular.x = 0.0
        velocity_msg.angular.y = 0.0
        velocity_msg.angular.z = 0.0

        self.velocity_pub.publish(velocity_msg)
        self.get_logger().info("Comando di movimento inviato al drone.")

    def stop_drone(self):
        # Ferma il drone
        self.get_logger().info("Il drone ha raggiunto il marker. Arresto.")
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.velocity_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    control_node = DroneControl()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
