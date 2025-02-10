import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")
 
import matplotlib
matplotlib.use('TkAgg')
 
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import pandas as pd
import os
 
class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
 
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
 
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile)
 
        self.get_logger().info("Sottoscritto ai topic /fmu/out/vehicle_odometry e /fmu/out/vehicle_status")
 
        self.armed = False
        self.csv_file = os.path.expanduser("~/odometry_data.csv")
        self.data = []
        self.start_time = None
 
        plt.ion()
        self.fig, self.axs = plt.subplots(2, 1, figsize=(12, 6))
        self.line1, = self.axs[0].plot([], [], label="X Position", linewidth=1.5)
        self.line2, = self.axs[0].plot([], [], label="Y Position", linewidth=1.5)
        self.line3, = self.axs[0].plot([], [], label="Z Position", linewidth=1.5)
 
        self.line4, = self.axs[1].plot([], [], label="X Velocity", linewidth=1.5)
        self.line5, = self.axs[1].plot([], [], label="Y Velocity", linewidth=1.5)
        self.line6, = self.axs[1].plot([], [], label="Z Velocity", linewidth=1.5)
 
        self.axs[0].set_xlabel("Time (s)")
        self.axs[0].set_ylabel("Position (m)")
        self.axs[0].legend()
        self.axs[0].grid(True)
        self.axs[0].set_title("Drone Position Over Time")
 
        self.axs[1].set_xlabel("Time (s)")
        self.axs[1].set_ylabel("Velocity (m/s)")
        self.axs[1].legend()
        self.axs[1].grid(True)
        self.axs[1].set_title("Drone Velocity Over Time")
 
    def status_callback(self, msg):
        self.armed = msg.arming_state == 2
        if self.armed:
            self.get_logger().info("Drone ARMATO, inizio registrazione dati.")
        else:
            self.get_logger().info("Drone DISARMATO, in attesa...")
 
    def odometry_callback(self, msg):
        if not self.armed:
            return
 
        timestamp = msg.timestamp * 1e-9
 
        if self.start_time is None:
            self.start_time = timestamp
 
        relative_time = timestamp - self.start_time  
 
        pos_x, pos_y, pos_z = msg.position
        vel_x, vel_y, vel_z = msg.velocity
 
        self.data.append([relative_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z])
 
        # 🟢 LOG: Controlliamo che i dati continuano a essere aggiornati
        self.get_logger().info(f"Time={relative_time:.3f} s, Pos=({pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}), Vel=({vel_x:.3f}, {vel_y:.3f}, {vel_z:.3f})")
 
        self.update_plot()
 
    def update_plot(self):
        if len(self.data) > 1:
            df = pd.DataFrame(self.data, columns=["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])
 
            # Mantieni solo gli ultimi 200 punti per migliorare le prestazioni
            df = df.iloc[-200:]
 
            self.line1.set_data(df["time"], df["pos_x"])
            self.line2.set_data(df["time"], df["pos_y"])
            self.line3.set_data(df["time"], df["pos_z"])
            self.line4.set_data(df["time"], df["vel_x"])
            self.line5.set_data(df["time"], df["vel_y"])
            self.line6.set_data(df["time"], df["vel_z"])
 
            # 🟢 LOG: Controlliamo se l'asse X si aggiorna
            print(f"Aggiornamento plot: Min time={df['time'].min()}, Max time={df['time'].max()}")
 
            self.axs[0].relim()
            self.axs[0].autoscale_view()
            self.axs[1].relim()
            self.axs[1].autoscale_view()
 
            plt.draw()
            plt.pause(0.001)  # Aggiorna più velocemente
 
    def save_to_csv(self):
        if self.data:
            df = pd.DataFrame(self.data, columns=["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])
            df["time"] = df["time"] - df["time"].min()
            df.to_csv(self.csv_file, index=False)
            self.get_logger().info(f"Dati salvati in {self.csv_file}")
 
    def plot_final_data(self):
        df = pd.DataFrame(self.data, columns=["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z"])
 
        plt.figure(figsize=(12, 6))
        plt.subplot(2, 1, 1)
        plt.plot(df["time"], df["pos_x"], label="X Position", linewidth=1.5)
        plt.plot(df["time"], df["pos_y"], label="Y Position", linewidth=1.5)
        plt.plot(df["time"], df["pos_z"], label="Z Position", linewidth=1.5)
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.legend()
        plt.grid(True)
        plt.title("Drone Position Over Time")
 
        plt.subplot(2, 1, 2)
        plt.plot(df["time"], df["vel_x"], label="X Velocity", linewidth=1.5)
        plt.plot(df["time"], df["vel_y"], label="Y Velocity", linewidth=1.5)
        plt.plot(df["time"], df["vel_z"], label="Z Velocity", linewidth=1.5)
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.grid(True)
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
        node.plot_final_data()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 
