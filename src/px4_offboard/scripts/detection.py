from std_msgs.msg import Float32, String, Float32MultiArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import math

class ParallelepipedAndMarkerDetector(Node):
    def __init__(self):
        super().__init__('parallelepiped_marker_detector')
        self.get_logger().info("Inizializzazione del nodo ParallelepipedAndMarkerDetector.")

        # Sottoscrizione al topic della camera
        self.image_sub = self.create_subscription(
            Image, '/rgbd_camera/image', self.image_callback, 10)
        self.get_logger().info("Sottoscritto al topic della camera: /rgbd_camera/image")

        # Sottoscrizione alla posizione del drone
        self.position_sub = self.create_subscription(
            Float32MultiArray, '/drone/position', self.position_callback, 10)
        self.get_logger().info("Sottoscritto al topic della posizione del drone: /drone/position")

        # Publisher per risultati rilevamento
        self.detection_pub = self.create_publisher(String, 'detection_results', 10)
        self.get_logger().info("Publisher creato per detection_results")

        # Publisher per i comandi al drone
        self.command_pub = self.create_publisher(Float32, 'drone_command', 10)
        self.get_logger().info("Publisher creato per drone_command")

        self.bridge = CvBridge()
        self.drone_position = None  # Posizione del drone inizialmente non nota

    def position_callback(self, msg):
        # Aggiorna la posizione del drone
        self.drone_position = msg.data[:3]  # Supponiamo che le coordinate siano nei primi 3 elementi

    def image_callback(self, msg):
        self.get_logger().info("Ricevuto un messaggio immagine.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.detect_objects(cv_image)
        except Exception as e:
            self.get_logger().error(f'Errore durante l\'elaborazione dell\'immagine: {e}')

    def calculate_distance(self, marker_position):
        if self.drone_position is None:
            return None  # Posizione del drone non disponibile

        # Calcola la distanza usando la formula euclidea
        distance = math.sqrt(
            (marker_position[0] - self.drone_position[0]) ** 2 +
            (marker_position[1] - self.drone_position[1]) ** 2 +
            (marker_position[2] - self.drone_position[2]) ** 2
        )
        return distance

    def detect_objects(self, image):
        self.get_logger().info("Inizio della funzione di rilevamento.")
        detection_result = "Nessun oggetto trovato."

        # Rilevazione parallelepipedo rosso
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Maschera per il rosso
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f"Trovati {len(contours)} contorni.")
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Soglia area per contorno parallelepipedo
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                detection_result = "Parallelepipedo trovato!"
                self.publish_drone_command("approach")  # Invio comando per avvicinarsi
                self.get_logger().info(detection_result)

        # **Correzione OpenCV ArUco**
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()

        # Rilevazione dell'ArUco marker
        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            detection_result += " e ArUco marker trovato!"

            # Calcolo della distanza
            marker_position = self.calculate_marker_position(corners)  # Calcola la posizione del marker
            distance = self.calculate_distance(marker_position)

            if distance is not None:
                self.get_logger().info(f"Distanza dal drone all'ArUco marker: {distance:.2f} metri")
                cv2.putText(image, f"Distanza: {distance:.2f} m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                if distance > 1.0:  # Se il drone è più lontano di 1 metro
                    self.publish_drone_command("approach")
                else:
                    self.publish_drone_command("stop")  # Comando per fermarsi

        self.get_logger().info(detection_result)
        cv2.imshow("Image", image)
        cv2.waitKey(1)

        self.detection_pub.publish(String(data=detection_result))
        self.get_logger().info("Risultato della rilevazione pubblicato.")

    def publish_drone_command(self, action):
        command_msg = Float32()
        if action == "approach":
            command_msg.data = 1.0  # Comando per avvicinarsi
        elif action == "stop":
            command_msg.data = 0.0  # Comando per fermarsi
        else:
            command_msg.data = -1.0  # Comando per allontanarsi (esempio)
        
        self.command_pub.publish(command_msg)
        self.get_logger().info(f"Pubblicato comando drone: {command_msg.data}")

    def calculate_marker_position(self, corners):
        # Calcola la posizione del marker in base ai corners
        if corners:
            corner = corners[0][0]  # Prendi il primo marker (se ci sono più marker)
            marker_position = (corner[0][0], corner[0][1], 0)  # Aggiungi la z se necessario
            return marker_position
        return None

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('ParallelepipedAndMarkerDetector').info("Avvio del nodo parallelepiped_marker_detector...")
    detector = ParallelepipedAndMarkerDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
