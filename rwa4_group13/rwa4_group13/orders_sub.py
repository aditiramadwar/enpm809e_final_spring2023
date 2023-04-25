import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from ariac_msgs.msg import Order

class rwa4(Node):

    def __init__(self):
        super().__init__('rwa4')
        self.orders_sub = self.create_subscription(
            Order, '/ariac/orders', self.orders_callback, 10)
        
        # self.table1_sub = self.create_subscription(
        #     String, '/table1_camera',
        #     self.listener1_callback,
        #     10)
        # self.table2_sub = self.create_subscription(
        #     String,
        #     '/table2_camera',
        #     self.listener2_callback,
        #     10)
        self.orders_sub  # prevent unused variable warning

    '''
    /ariac/orders
    id: MMB30H56
    type: 0
    priority: false
    kitting_task:
    agv_number: 4
    tray_id: 3
    destination: 3
    parts:
    - part:
        color: 2
        type: 10
        quadrant: 3
    - part:
        color: 4
        type: 11
        quadrant: 1
    assembly_task:
    agv_numbers: []
    station: 0
    parts: []
    combined_task:
    station: 0
    parts: []
    '''
    def orders_callback(self, msg):
        self.get_logger().info('I heard /ariac/orders info: "%s"' % msg.id)
        # create object with all the msg information

    # def listener1_callback(self, msg):
    #     self.get_logger().info('I heard tabel 1 camera info: "%s"' % msg.data)

    # def listener2_callback(self, msg):
    #     self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()