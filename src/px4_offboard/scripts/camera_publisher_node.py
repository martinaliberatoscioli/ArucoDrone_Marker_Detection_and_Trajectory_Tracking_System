import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.bridge = CvBridge()

        # Crea un Publisher per pubblicare immagini processate
        self.image_pub = self.create_publisher(Image, "/processed_camera/image", 10)

        # Crea un Subscriber per ricevere immagini
        self.subscription = self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.image_callback,
            10
        )

        # Timer per pubblicare immagini periodicamente (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.latest_frame = None  # Memorizza l'ultima immagine ricevuta

    def image_callback(self, msg):
        try:
            # Converti il messaggio ROS Image in un'immagine OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Visualizza l'immagine
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

            # Salva l'ultima immagine ricevuta
            self.latest_frame = frame

        except Exception as e:
            self.get_logger().error(f"Errore nella conversione dell'immagine: {e}")

    def publish_image(self):
        """Pubblica l'ultima immagine ricevuta periodicamente."""
        if self.latest_frame is not None:
            img_msg = self.bridge.cv2_to_imgmsg(self.latest_frame, encoding="bgr8")
            self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
