import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.bridge = CvBridge()
        
        # Crea un publisher per inviare le immagini su /rgbd_camera/image
        self.publisher = self.create_publisher(Image, '/rgbd_camera/image', 10)
        
        # Inizializza la videocamera (se usi una simulazione, questo può essere simulato con immagini statiche)
        self.timer = self.create_timer(0.1, self.publish_image)  # Invio a 10Hz
        
        self.get_logger().info("Nodo Camera Publisher avviato, pubblicando su /rgbd_camera/image")
    
    def publish_image(self):
        # Simula l'acquisizione di un frame dalla videocamera
        ret, frame = cv2.VideoCapture(0).read()  # Usa la webcam reale, se disponibile
        if not ret:
            self.get_logger().warn("Impossibile acquisire il frame dalla videocamera.")
            return
        
        try:
            # Converti il frame OpenCV in messaggio ROS2 Image
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info("Immagine pubblicata su /rgbd_camera/image")
        
        except Exception as e:
            self.get_logger().error(f"Errore durante la conversione dell'immagine: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()