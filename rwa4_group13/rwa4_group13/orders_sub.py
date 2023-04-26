import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage

class rwa4(Node):
    def __init__(self):
        super().__init__('rwa4')
        self.orders_sub = self.create_subscription(
            Order, '/ariac/orders', self.orders_callback, 10)
        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.table1_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/table1_camera/image',
            self.table1_callback, qos_policy)
        
        self.table2_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/table2_camera/image',
            self.table2_callback, qos_policy)
        
        self.orders_sub  # prevent unused variable warning
        self.table1_sub
        self.table2_sub

    def orders_callback(self, msg):
        self.get_logger().info('/ariac/orders info: "%s"' % msg.id)
        # create object with all the msg information

    def table1_callback(self, msg):
        self.get_logger().info('tabel 1 camera: id: "%s"' % msg.tray_poses[0].id)
        self.get_logger().info('tabel 1 camera: position x: "%s"' % msg.tray_poses[0].pose.position.x)
        self.get_logger().info('tabel 1 camera: position y: "%s"' % msg.tray_poses[0].pose.position.y)
        self.get_logger().info('tabel 1 camera: position z: "%s"' % msg.tray_poses[0].pose.position.z)

    def table2_callback(self, msg):
        self.get_logger().info('tabel 2 camera: id: "%s"' % msg.tray_poses[0].id)
        self.get_logger().info('tabel 2 camera: position x: "%s"' % msg.tray_poses[0].pose.position.x)
        self.get_logger().info('tabel 2 camera: position y: "%s"' % msg.tray_poses[0].pose.position.y)
        self.get_logger().info('tabel 2 camera: position z: "%s"' % msg.tray_poses[0].pose.position.z)

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()