#!/usr/bin/env python3

import os
from std_msgs.msg import String, Float32MultiArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
import math
from rclpy.clock import Clock

class ParallelepipedAndMarkerDetector(Node):
    def __init__(self):
        super().__init__('parallelepiped_marker_detector')
        self.get_logger().info("Nodo di rilevamento (parallelepipedo + ArUco) avviato!")

        # Sottoscrizione al topic della camera
        self.image_sub = self.create_subscription(
            Image, '/rgbd_camera/image', self.image_callback, 10)

        # Publisher per l'immagine processata
        self.image_pub = self.create_publisher(Image, "/processed_camera/image", 10)

        # Publisher per i risultati di rilevamento (log testuale)
        self.detection_pub = self.create_publisher(String, 'detection_results', 10)

        # Publisher per la posizione dell'ArUco
        self.aruco_pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)

        self.bridge = CvBridge()
        self.drone_position = None

        # ID dell'ArUco target
        self.target_marker_id = 42

        # Percorso dell'immagine di Bologna
        self.bologna_image_path = "/home/martina/DRONE_2/src/px4_offboard/scripts/Bologna.jpg"
       
        # Carica l'immagine di Bologna in memoria
        self.bologna_image = self.load_image(self.bologna_image_path)

    def image_callback(self, msg):
        """Riceve un frame dalla telecamera e rileva l'ArUco Marker."""
        self.get_logger().info("Ricevuto un frame dalla telecamera.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect_objects(cv_image)
        except Exception as e:
            self.get_logger().error(f"Errore nell'elaborazione dell'immagine: {e}")

    def detect_objects(self, image):
        """Rileva il parallelepipedo rosso e l'ArUco marker."""
        detection_result = "Nessun oggetto trovato."

        # Rilevamento parallelepipedo rosso
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f"Trovati {len(contours)} contorni rossi.")

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Soglia minima
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                detection_result = "Parallelepipedo rilevato!"

        # Rilevamento ArUco
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            detection_result += " e ArUco trovato!"

            # Controlla se l'ArUco rilevato è quello target (42)
            if self.target_marker_id in ids:
                self.get_logger().info(f"ArUco ID {self.target_marker_id} rilevato! Pubblico immagine di Bologna.")
                self.publish_bologna_image()
            else:
                self.publish_camera_feed(image)

            # Calcola la posizione dell'ArUco come media dei corners
            marker_position = self.calculate_marker_position(corners)

            # Pubblica la posizione dell'ArUco (PoseStamped)
            if marker_position:
                self.publish_aruco_pose(marker_position)

        else:
            self.get_logger().info("Nessun ArUco rilevato. Pubblico solo il feed della telecamera.")
            self.publish_camera_feed(image)

        # Mostra il feed della telecamera
        cv2.imshow("Rilevamento", image)
        cv2.waitKey(1)

        # Pubblica il risultato testuale
        self.detection_pub.publish(String(data=detection_result))
        self.get_logger().info(f"Risultato rilevamento: {detection_result}")

    def publish_aruco_pose(self, marker_position):
        """Pubblica la posizione dell'ArUco come PoseStamped."""
        if marker_position is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"  # o "camera_link", a seconda del tuo frame
            pose_msg.pose.position.x = float(marker_position[0])
            pose_msg.pose.position.y = float(marker_position[1])
            pose_msg.pose.position.z = float(marker_position[2])
            
            self.aruco_pose_pub.publish(pose_msg)
            self.get_logger().info(f"Posizione ArUco pubblicata: {marker_position}")
        else:
            self.get_logger().warn("Impossibile pubblicare Pose: marker_position è None!")

    def calculate_marker_position(self, corners):
        """Ritorna (x, y, 0) come posizione media del marker."""
        if corners and len(corners) > 0:
            corner = np.mean(corners[0][0], axis=0)  # media dei corner
            return (corner[0], corner[1], 0.0)
        return None

    def load_image(self, image_path):
        """Carica un'immagine di Bologna."""
        if not os.path.exists(image_path):
            self.get_logger().error(f"Errore: file {image_path} non trovato.")
            return None

        img = cv2.imread(image_path)
        if img is not None:
            self.get_logger().info(f"Immagine caricata: {image_path}")
            return img
        else:
            self.get_logger().error(f"Errore nel caricamento dell'immagine: {image_path} non può essere aperto.")
            return None

    def publish_bologna_image(self):
        """Pubblica l'immagine di Bologna su /processed_camera/image."""
        if self.bologna_image is None:
            self.get_logger().error("Immagine di Bologna non trovata! Verifica il percorso.")
            return
        
        img_msg = self.bridge.cv2_to_imgmsg(self.bologna_image, encoding="bgr8")
        self.image_pub.publish(img_msg)
        self.get_logger().info("Immagine di Bologna pubblicata su /processed_camera/image")

    def publish_camera_feed(self, image):
        """Pubblica il frame della telecamera su /processed_camera/image."""
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = ParallelepipedAndMarkerDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
