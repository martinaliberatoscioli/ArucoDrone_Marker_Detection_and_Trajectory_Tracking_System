import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import image_transport

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.bridge = CvBridge()
        
        # Usa image_transport per pubblicare immagini in modo efficiente
        self.image_pub = image_transport.create_publisher(self, "/rgbd_camera/image_raw", 10)

        # Iscrizione alla telecamera
        self.subscription = self.create_subscription(
            Image,
            '/rgbd_camera/image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Pubblica l'immagine usando image_transport
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)

            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Errore nella conversione dell'immagine: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
