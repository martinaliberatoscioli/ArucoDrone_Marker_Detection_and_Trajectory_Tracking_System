import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CameraTF2Publisher(Node):
    def __init__(self):
        super().__init__('camera_tf2_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_transform)  # Pubblica ogni 0.1 secondi

    def publish_transform(self):
        # Crea il messaggio di trasformazione
        transform = TransformStamped()

        # Imposta l'header (ora, quindi la trasformazione avverrà rispetto al frame 'world')
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'  # Il frame di riferimento globale
        transform.child_frame_id = 'camera_link'  # Il frame della fotocamera

        # Definisci la posizione e l'orientamento della fotocamera rispetto al frame globale
        transform.transform.translation.x = 1.0  # Posizione della fotocamera (modifica secondo le tue esigenze)
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 1.5  # Ad esempio, fotocamera a 1.5 metri sopra il suolo
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # Nessuna rotazione, fotocamera allineata

        # Pubblica la trasformazione
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    camera_tf2_publisher = CameraTF2Publisher()
    rclpy.spin(camera_tf2_publisher)
    camera_tf2_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
