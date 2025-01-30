import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")

import matplotlib
matplotlib.use('TkAgg')  # Usa un backend compatibile con GUI

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry  # Messaggio corretto per PX4
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import pandas as pd
import os

class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')

        # QoS compatibile con PX4 (Best Effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile)
        self.subscription  # Evita il warning per variabile inutilizzata

        self.get_logger().info("Sottoscritto al topic /fmu/out/vehicle_odometry con QoS Best Effort")

        # File CSV
        self.csv_file = os.path.expanduser("~/odometry_data.csv")
        self.data = []
        self.start_time = None  # Per normalizzare il tempo

    def odometry_callback(self, msg):
        timestamp = msg.timestamp * 1e-6  # Converti da microsecondi a secondi

        # Se è il primo messaggio, usa il timestamp come riferimento
        if self.start_time is None:
            self.start_time = timestamp

        # Normalizza il tempo rispetto al primo campione
        relative_time = timestamp - self.start_time

        pos_x, pos_y, pos_z = msg.position
        vel_x, vel_y, vel_z = msg.velocity

        self.data.append([relative_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z])

        self.get_logger().info(f"Time={relative_time:.3f}, Pos=({pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}), Vel=({vel_x:.3f}, {vel_y:.3f}, {vel_z:.3f})")

    def save_to_csv(self):
        if self.data:
            df = pd.DataFrame(self.data, columns=["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])
            df.to_csv(self.csv_file, index=False)
            self.get_logger().info(f"Dati salvati in {self.csv_file}")
            self.plot_data(df)

    def plot_data(self, df):
        plt.figure(figsize=(12, 6))

        # Normalizza l'asse del tempo per partire da zero
        df["time"] = df["time"] - df["time"].min()

        plt.subplot(2, 1, 1)
        plt.plot(df["time"], df["pos_x"], label="X Position")
        plt.plot(df["time"], df["pos_y"], label="Y Position")
        plt.plot(df["time"], df["pos_z"], label="Z Position")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
        plt.title("Drone Position Over Time")

        plt.subplot(2, 1, 2)
        plt.plot(df["time"], df["vel_x"], label="X Velocity")
        plt.plot(df["time"], df["vel_y"], label="Y Velocity")
        plt.plot(df["time"], df["vel_z"], label="Z Velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.title("Drone Velocity Over Time")

        plt.tight_layout()
        plt.savefig(os.path.expanduser("~/odometry_plot.png"))
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arresto... Salvataggio dati...")
        node.save_to_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
