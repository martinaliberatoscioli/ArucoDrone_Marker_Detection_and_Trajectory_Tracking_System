#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
 
 
class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode and keeping it in place."""
 
    def __init__(self) -> None:
        super().__init__('offboard_control')
 
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
 
        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
 
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
 
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = None
        self.vehicle_status = None
        self.target_position = (2.5, 2.0, -2.0)  # Destinazione desiderata (X, Y, Z)
        self.position_reached = False  # Flag per segnalare il raggiungimento della posizione
 
        # Timer per inviare comandi periodicamente
        self.timer = self.create_timer(0.1, self.timer_callback)
 
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Aggiorna la posizione attuale del drone."""
        self.vehicle_local_position = vehicle_local_position
 
    def vehicle_status_callback(self, vehicle_status):
        """Aggiorna lo stato del drone."""
        self.vehicle_status = vehicle_status
 
    def arm(self):
        """Invia il comando di armamento."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Drone armato!')
 
    def engage_offboard_mode(self):
        """Attiva la modalità OFFBOARD."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Passaggio alla modalità OFFBOARD in corso...")
 
    def publish_offboard_control_heartbeat_signal(self):
        """Invia il segnale per mantenere il drone in modalità OFFBOARD."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
 
    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Invia il setpoint di posizione al drone."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # 90 gradi (verso la direzione desiderata)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Setpoint inviato: {[x, y, z]}")
 
    def publish_vehicle_command(self, command, **params) -> None:
        """Pubblica un comando per il veicolo."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
 
    def timer_callback(self) -> None:
        """Loop principale per il controllo del drone."""
        self.publish_offboard_control_heartbeat_signal()
 
        if self.offboard_setpoint_counter >= 10:
            if self.vehicle_status is not None and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.engage_offboard_mode()
            
            self.arm()
 
        # Controlla se il drone è in OFFBOARD e aggiorna la posizione
        if (
            self.vehicle_status is not None and
            self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.vehicle_local_position is not None
        ):
            # Se il drone non ha ancora raggiunto la posizione, continua a inviare il setpoint
            if not self.position_reached:
                self.publish_position_setpoint(*self.target_position)
 
                # Controllo se il drone è vicino alla destinazione
                if (
                    abs(self.vehicle_local_position.x - self.target_position[0]) < 0.1 and
                    abs(self.vehicle_local_position.y - self.target_position[1]) < 0.1 and
                    abs(self.vehicle_local_position.z - self.target_position[2]) < 0.1
                ):
                    self.get_logger().info("Posizione raggiunta!")
                    self.position_reached = True
 
            # Se il drone ha raggiunto la posizione, continua a inviare lo stesso setpoint per mantenerlo stabile
            else:
                self.publish_position_setpoint(*self.target_position)
                self.get_logger().info("Mantenendo la posizione!")
 
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
 
 
def main(args=None) -> None:
    print('🚀 Avvio nodo di controllo offboard...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Errore: {e}")
 
