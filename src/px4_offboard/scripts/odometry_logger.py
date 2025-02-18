#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", category=UserWarning, module="matplotlib")
 
import matplotlib
matplotlib.use('TkAgg')
 
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
import pandas as pd
import os
import time
 
class OdometryLogger(Node):
    def __init__(self):
        super().__init__('odometry_logger')
 
        # Configurazione QoS compatibile con PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
 
        # Sottoscrizioni con QoS corretto
        self.subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
 
        self.status_subscription = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
 
        self.get_logger().info("Sottoscritto ai topic: /fmu/out/vehicle_odometry e /fmu/out/vehicle_status")
 
        self.armed = False
        self.csv_file = os.path.expanduser("~/odometry_data.csv")
        self.data = []
        self.start_time = None
        self.last_update_time = time.time()  # Per ridurre la frequenza dei log
 
        # Inizializzazione della visualizzazione con 3 subplot (posizione, velocità, quaternioni)
        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(12, 9))
        
        # Posizione
        self.line1, = self.axs[0].plot([], [], label="X Position", linewidth=1.5)
        self.line2, = self.axs[0].plot([], [], label="Y Position", linewidth=1.5)
        self.line3, = self.axs[0].plot([], [], label="Z Position", linewidth=1.5)
 
        # Velocità
        self.line4, = self.axs[1].plot([], [], label="X Velocity", linewidth=1.5)
        self.line5, = self.axs[1].plot([], [], label="Y Velocity", linewidth=1.5)
        self.line6, = self.axs[1].plot([], [], label="Z Velocity", linewidth=1.5)
 
        # Quaternioni
        self.line7, = self.axs[2].plot([], [], label="q_w", linewidth=1.5)
        self.line8, = self.axs[2].plot([], [], label="q_x", linewidth=1.5)
        self.line9, = self.axs[2].plot([], [], label="q_y", linewidth=1.5)
        self.line10, = self.axs[2].plot([], [], label="q_z", linewidth=1.5)
 
        # Configura grafici
        for ax, title, ylabel in zip(self.axs,
                                     ["Drone Position Over Time", "Drone Velocity Over Time", "Drone Orientation (Quaternions)"],
                                     ["Position (m)", "Velocity (m/s)", "Quaternion Value"]):
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True)
            ax.set_title(title)
 
    def status_callback(self, msg):
        """Verifica se il drone è armato."""
        self.armed = msg.arming_state == 2
        if self.armed:
            self.get_logger().info("Drone ARMATO, inizio registrazione dati.")
        else:
            self.get_logger().info("Drone DISARMATO, in attesa...")
 
    def odometry_callback(self, msg):
        """Registra i dati di odometria e aggiorna il grafico."""
        if not self.armed:
            return  # Non registrare dati se il drone non è armato
 
        timestamp = msg.timestamp * 1e-9  # Converti in secondi
 
        if self.start_time is None:
            self.start_time = timestamp
 
        relative_time = timestamp - self.start_time  
 
        try:
            pos_x, pos_y, pos_z = msg.position
            vel_x, vel_y, vel_z = msg.velocity
            q_w, q_x, q_y, q_z = msg.q  # 🔹 Ottenere i quaternioni
 
            self.data.append([relative_time, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, q_w, q_x, q_y, q_z])
 
            # Riduzione della frequenza dei log
            if time.time() - self.last_update_time > 1.0:
                self.get_logger().info(
                    f"Time={relative_time:.3f} s | Pos=({pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}) | "
                    f"Vel=({vel_x:.3f}, {vel_y:.3f}, {vel_z:.3f}) | q=({q_w:.3f}, {q_x:.3f}, {q_y:.3f}, {q_z:.3f})"
                )
                self.last_update_time = time.time()
 
            self.update_plot()
 
        except AttributeError:
            self.get_logger().error("Errore: dati di odometria non validi.")
 
    def update_plot(self):
        """Aggiorna il grafico in tempo reale."""
        if len(self.data) > 1:
            df = pd.DataFrame(self.data, columns=["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "q_w", "q_x", "q_y", "q_z"])
            df = df.iloc[-200:]  # Mantieni solo gli ultimi 200 punti per efficienza
 
            # Posizione
            self.line1.set_data(df["time"], df["pos_x"])
            self.line2.set_data(df["time"], df["pos_y"])
            self.line3.set_data(df["time"], df["pos_z"])
            
            # Velocità
            self.line4.set_data(df["time"], df["vel_x"])
            self.line5.set_data(df["time"], df["vel_y"])
            self.line6.set_data(df["time"], df["vel_z"])
 
            # Quaternioni
            self.line7.set_data(df["time"], df["q_w"])
            self.line8.set_data(df["time"], df["q_x"])
            self.line9.set_data(df["time"], df["q_y"])
            self.line10.set_data(df["time"], df["q_z"])
 
            for ax in self.axs:
                ax.relim()
                ax.autoscale_view()
 
            plt.draw()
            plt.pause(0.1)  
 
    def save_to_csv(self):
        """Salva i dati in un file CSV."""
        if self.data:
            df = pd.DataFrame(self.data, columns=["time", "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "q_w", "q_x", "q_y", "q_z"])
            df["time"] = df["time"] - df["time"].min()
            df.to_csv(self.csv_file, index=False)
            self.get_logger().info(f"Dati salvati in {self.csv_file}")
 
def main():
    rclpy.init()
    node = OdometryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arresto... Salvataggio dati...")
        node.save_to_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 
 
