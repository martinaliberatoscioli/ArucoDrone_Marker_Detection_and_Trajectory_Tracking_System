import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.bridge = CvBridge()
        # Iscrizione al topic /rgbd_camera/image
        self.subscription = self.create_subscription(
            Image,
            '/rgbd_camera/image',  # Il topic della videocamera simulata
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Converti il messaggio ROS 2 Image in un array NumPy (OpenCV)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Ora puoi elaborare l'immagine (ad esempio, rilevare l'ArUco marker)
            # Usa OpenCV per visualizzare l'immagine o fare altro
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

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
