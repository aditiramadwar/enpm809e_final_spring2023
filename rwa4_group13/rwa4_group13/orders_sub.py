import rclpy
from rclpy.node import Node

from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
import PyKDL

class Part():
    def __init__(self, part_color, part_type, part_quadrant) -> None:
        self.color_dict = {0:"Red", 1:"Green", 2 : "Blue", 3: "Orange", 4: "Purple"}
        self.type_dict = {10:"Battery", 11: "Pump", 12:"Sensor", 13:"Regulator"}
        self.color = self.color_dict[part_color]
        self.type = self.type_dict[part_type]
        self.quadrant = part_quadrant
    
    def __str__(self) -> str:
        
        return "\n\t- {} {} \n\t\t- pose : \n\t\t\t- position: [] \n\t\t\t- orientation: []".format(self.color, self.type)
    
        # return "color : {},\n\t type : {},\n\t quadrant : {}\n\t".format(self.color, self.type, self.quadrant)

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
        part_string = "".join("\t{}".format(part.__str__()) for part in self.parts)
        output = "Tray:\n\t- id : {} \n\t- pose:\n\t\t- position: [] \n\t\t- orientation: [] \n Part: {}".format(self.tray_id, part_string)

        # part_string = "".join("part_{} : \n\t{}".format(str(idx), part.__str__()) for idx, part in enumerate(self.parts))
        # output = "agv_number : {},\n tray_id : {},\n destination : {},\n parts : \n\t{}".format(self.agv_number, self.tray_id, self.destination, part_string)

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
        output = "\n----------------------\n--- Order {} ---\n----------------------\n {} \n".format(self.id, self.kitting_task.__str__())
        # output = "----------------------\n--- Order {} ---\n----------------------\n type : {},\n priority : {},\n kitting_task : {}".format(self.id, self.type, self.priority, self.kitting_task.__str__())

        return output

from geometry_msgs.msg import Pose
import PyKDL

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
        
        self.left_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image',
            self.left_bin_callback, qos_policy)
        
        self.right_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image',
            self.right_bin_callback, qos_policy)
        
        self.orders_sub
        self.table1_sub
        self.table2_sub
        self.left_bin_sub
        self.right_bin_sub

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
        if(len(msg.tray_poses) != 0):
            tray_world_frame = Pose()
            tray_world_frame = self._multiply_pose(msg.sensor_pose, msg.tray_poses[0].pose)
        # self.get_logger().info('tray in world frame: position x: "%s"' % tray_world_frame.position.x)

    def table2_callback(self, msg):
        # self.get_logger().info('tabel 2 camera: id: "%s"' % msg.tray_poses[0].id)
        if(len(msg.tray_poses) != 0):
            tray_world_frame = Pose()
            tray_world_frame = self._multiply_pose(msg.sensor_pose, msg.tray_poses[0].pose)
        # self.get_logger().info('tray in world frame: position x: "%s"' % tray_world_frame.position.x)

    def left_bin_callback(self, msg):
        # color, type
        # self.get_logger().info('tabel 2 camera: id: "%s"' % msg.part_poses[0].part)

        if(len(msg.part_poses) != 0):
            part_world_frame = Pose()
            part_world_frame = self._multiply_pose(msg.sensor_pose, msg.part_poses[0].pose)
        # self.get_logger().info('left part in world frame: position x: "%s"' % part_world_frame.position.x)

    def right_bin_callback(self, msg):
        # color, type
        # self.get_logger().info('tabel 2 camera: id: "%s"' % msg.part_poses[0].part)
        if(len(msg.part_poses) != 0):
            part_world_frame = Pose()
            part_world_frame = self._multiply_pose(msg.sensor_pose, msg.part_poses[0].pose)
        # self.get_logger().info('right part in world frame: position x: "%s"' % part_world_frame.position.x)

    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        '''
        Use KDL to multiply two poses together. Function taken from tf_node.py
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose
    
def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()