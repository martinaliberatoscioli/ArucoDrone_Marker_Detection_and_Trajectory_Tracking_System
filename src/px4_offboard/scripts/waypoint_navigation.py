import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, OffboardControlMode, VehicleCommand
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Publisher per i comandi di traiettoria
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Timer per inviare i comandi periodicamente
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Definizione dei waypoints [x, y, z] in metri
        self.waypoints = [
            [0.0, 0.0, -2.0],  # Decollo a 2 metri di altezza
            [2.0, 0.0, -2.0],  # Spostamento in X
            [2.0, 2.0, -2.0],  # Spostamento in Y
            [0.0, 2.0, -2.0],  # Ritorno
            [0.0, 0.0, -2.0],  # Centro
            [0.0, 0.0, -0.2],  # Atterraggio
        ]
        
        self.current_waypoint = 0
        self.offboard_started = False

    def timer_callback(self):
        """ Invia periodicamente i comandi di controllo al drone. """
        if not self.offboard_started:
            self.start_offboard_mode()
        
        if self.current_waypoint < len(self.waypoints):
            self.publish_waypoint(self.waypoints[self.current_waypoint])

    def publish_waypoint(self, waypoint):
        """ Pubblica un waypoint su /fmu/in/trajectory_setpoint """
        msg = TrajectorySetpoint()
        msg.position = [waypoint[0], waypoint[1], -abs(waypoint[2])]  # Assicura il frame NED corretto
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = 0.0  # Angolo di direzione (0 significa rivolto a nord)

        self.trajectory_publisher.publish(msg)
        self.get_logger().info(f"📍 Waypoint inviato: {waypoint}")

        # Controlla se il drone è vicino al waypoint
        self.check_waypoint_reached(waypoint)

    def check_waypoint_reached(self, waypoint):
        """ Controlla se il drone ha raggiunto il waypoint. """
        time.sleep(3)  # Tempo stimato per raggiungere il waypoint
        self.current_waypoint += 1

    def start_offboard_mode(self):
        """ Attiva la modalità Offboard e arma il drone. """
        self.get_logger().info("🔧 Attivazione Offboard Mode...")

        # Imposta il drone in modalità Offboard
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True

        # Invia il messaggio più volte per assicurarsi che venga accettato da PX4
        for _ in range(10):
            self.control_mode_publisher.publish(offboard_msg)
            time.sleep(0.1)  # Aspetta tra un invio e l'altro

        # Arma il drone
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

        time.sleep(1)
        self.offboard_started = True
        self.get_logger().info("🚀 Modalità Offboard attivata e drone armato.")

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        """ Invia comandi al veicolo (es. arm, disarm). """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("✋ Interruzione del volo.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
