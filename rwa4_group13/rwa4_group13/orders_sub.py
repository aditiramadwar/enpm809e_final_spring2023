import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage

class Part():
    def __init__(self, part_color, part_type, part_quadrant) -> None:
        self.color = part_color
        self.type = part_type
        self.quadrant = part_quadrant
    
    def __str__(self) -> str:
        
        return "color : {},\n\t type : {},\n\t quadrant : {}\n\t".format(self.color, self.type, self.quadrant)
class Kitting_Task():
    def __init__(self, agv_number, tray_id, destination, parts) -> None:
        self.agv_number = agv_number
        self.tray_id = tray_id
        self.destination = destination
        self.parts = [Part(part_color = kitting_part.part.color, 
                           part_type = kitting_part.part.type, 
                           part_quadrant = kitting_part.quadrant) 
                           for kitting_part in parts]
    
    def __str__(self) -> str:
        part_string = "".join("part_{} : \n\t{}".format(str(idx), part.__str__()) for idx, part in enumerate(self.parts))
        output = "agv_number : {},\n tray_id : {},\n destination : {},\n parts : \n\t{}".format(self.agv_number, self.tray_id, self.destination, part_string)

        return output
class AriacOrder():

    def __init__(self, order_id, order_type, order_priority, kitting_task) -> None:
        self.id = order_id 
        self.type = order_type
        self.priority = order_priority
        self.kitting_task = Kitting_Task(agv_number = kitting_task.agv_number,
                                         tray_id = kitting_task.tray_id,
                                         destination = kitting_task.destination,
                                         parts = kitting_task.parts
                                         )


    def __str__(self) -> str:
        output = "id : {},\n type : {},\n priority : {},\n kitting_task : {}".format(self.id, self.type, self.priority, self.kitting_task.__str__())

        return output


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
        order = AriacOrder(order_id = msg.id,
                      order_type = msg.type,
                      order_priority = msg.priority,
                      kitting_task = msg.kitting_task
                      )
        self.get_logger().info('/ariac/orders info: "%s"' % order.__str__())
        # create object with all the msg information

    def table1_callback(self, msg):
        # self.get_logger().info('tabel 1 camera: id: "%s"' % msg.tray_poses[0].id)
        # self.get_logger().info('tabel 1 camera: position x: "%s"' % msg.tray_poses[0].pose.position.x)
        # self.get_logger().info('tabel 1 camera: position y: "%s"' % msg.tray_poses[0].pose.position.y)
        # self.get_logger().info('tabel 1 camera: position z: "%s"' % msg.tray_poses[0].pose.position.z)
        pass

    def table2_callback(self, msg):
        # self.get_logger().info('tabel 2 camera: id: "%s"' % msg.tray_poses[0].id)
        # self.get_logger().info('tabel 2 camera: position x: "%s"' % msg.tray_poses[0].pose.position.x)
        # self.get_logger().info('tabel 2 camera: position y: "%s"' % msg.tray_poses[0].pose.position.y)
        # self.get_logger().info('tabel 2 camera: position z: "%s"' % msg.tray_poses[0].pose.position.z)
        pass

def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()