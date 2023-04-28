import rclpy
from rclpy.node import Node

from ariac_msgs.msg import Order
from ariac_msgs.msg import AdvancedLogicalCameraImage
from geometry_msgs.msg import Pose
import PyKDL

class Part():

    def __init__(self, part_color, part_type, **kwargs) -> None:

        self.color = part_color
        self.type = part_type
        self.quadrant = kwargs["part_quadrant"] if kwargs else None
    
    def __str__(self) -> str:
        
        if self.quadrant:
            return "color : {},\n\t type : {},\n\t quadrant : {}\n\t".format(self.color, self.type, self.quadrant)
        else:
            return "color : {},\n\t type : {}\n\t".format(self.color, self.type)

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

        # part_string = "".join("\t{}".format(part.__str__()) for part in self.parts)
        # output = "Tray:\n\t- id : {} \n\t- pose:\n\t\t- position: [] \n\t\t- orientation: [] \n Part: {}".format(self.tray_id, part_string)

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

class Position():

    def __init__(self, x, y, z) -> None:
        self.x = x
        self.y = y
        self.z = z
    
    def __str__(self) -> str:
        
        return "position: \n\t x : {},\n\t y : {},\n\t z : {}\n\t".format(self.x, self.y, self.z)

class Orientation():

    def __init__(self, x, y, z, w) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def __str__(self) -> str:
        
        return "orientation: \n\t x : {},\n\t y : {},\n\t z : {},\n\t w : {}\n\t".format(self.x, self.y, self.z, self.w)

class ObjectPose():
    
    def __init__(self, object_position, object_orientation) -> None:

        self.position = Position(x = object_position.x,
                                 y = object_position.y,
                                 z = object_position.z)
        
        self.orientation = Orientation(x = object_orientation.x,
                                       y = object_orientation.y,
                                       z = object_orientation.z,
                                       w = object_orientation.w)
    
    def __str__(self) -> str:
        
        return "{}\n{}".format(self.position.__str__(), self.orientation.__str__())

class WorldFrame():

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

class TrayPoses(WorldFrame):

    def __init__(self, tray_poses, sensor_pose) -> None:
        
        self.ids = []
        self.poses = []
        for tray_pose in tray_poses:

            self.ids.append(tray_pose.id)
            tray_world_pose = self._multiply_pose(sensor_pose, tray_pose.pose)
            self.poses.append(ObjectPose(object_position = tray_world_pose.position,
                                        object_orientation = tray_world_pose.orientation))
        
        self.sensor_pose = ObjectPose(object_position = sensor_pose.position,
                                      object_orientation= sensor_pose.orientation)

    def __str__(self) -> str:
        
        # part_string = "".join("part_{} : \n\t{}".format(str(idx), part.__str__()) for idx, part in enumerate(self.parts))
        # output = "agv_number : {},\n tray_id : {},\n destination : {},\n parts : \n\t{}".format(self.agv_number, self.tray_id, self.destination, part_string)
        output = "tray_poses: \n"+"".join("tray_id : {}\n{}".format(str(tray_id), pose.__str__()) for tray_id, pose in zip(self.ids, self.poses)) 

        return output + "\n\t sensor pose: \n{}".format(self.sensor_pose.__str__())


        # return self.sensor_pose.__str__()
    

class PartPoses(WorldFrame):

    def __init__(self, part_poses, sensor_pose) -> None:
        
        self.parts = []
        self.poses = []
        for part_pose in part_poses:

            part_world_pose = self._multiply_pose(sensor_pose, part_pose.pose)
            self.parts.append(Part(part_color = part_pose.part.color,
                                   part_type = part_pose.part.type))
            self.poses.append(ObjectPose(object_position = part_world_pose.position,
                                        object_orientation = part_world_pose.orientation))
        
        self.sensor_pose = ObjectPose(object_position = sensor_pose.position,
                                      object_orientation= sensor_pose.orientation)

    def __str__(self) -> str:
    
        # part_string = "".join("part_{} : \n\t{}".format(str(idx), part.__str__()) for idx, part in enumerate(self.parts))
        # output = "agv_number : {},\n tray_id : {},\n destination : {},\n parts : \n\t{}".format(self.agv_number, self.tray_id, self.destination, part_string)
        output = "part_poses: \n"+"".join("{}/n{}".format(part.__str__(), pose.__str__()) for part, pose in zip(self.parts, self.poses)) + "\n\t".format(self.sensor_pose.__str__())
        # print(output)
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
        
        self.left_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/left_bins_camera/image',
            self.left_bin_callback, qos_policy)
        
        self.right_bin_sub = self.create_subscription(
            AdvancedLogicalCameraImage, '/ariac/sensors/right_bins_camera/image',
            self.right_bin_callback, qos_policy)
        
        # Flags for logging first messages
        self.table1_msg = False
        self.table2_msg = False
        self.left_bin_msg = False
        self.right_bin_msg = False
        self.parse_flag = False

        self.tray1_poses = None
        self.tray2_poses = None
        self.part1_poses = None
        self.part2_poses = None
        self.order = None

        self.color_dict = {0:"Red", 1:"Green", 2 : "Blue", 3: "Orange", 4: "Purple"}
        self.type_dict = {10:"Battery", 11: "Pump", 12:"Sensor", 13:"Regulator"}
        # self.fourhtfunc()
        self.timed_function = self.create_timer(0.1, self.parse_print)

        # aditi: return/store the objects created for printing out everything in the end

    def orders_callback(self, msg):
        self.order = AriacOrder(order_id = msg.id,
                      order_type = msg.type,
                      order_priority = msg.priority,
                      kitting_task = msg.kitting_task
                      )
        # self.parse_orders()
        # self.get_logger().info('/ariac/orders info: "%s"' % self.order.__str__())
        # self.get_logger().info('/ariac/orders info: "%s"' % self.order.parse())

        # enable the flags to log only the first message
        self.table1_msg = True
        self.table2_msg = True
        self.left_bin_msg = True
        self.right_bin_msg = True
        self.parse_flag = True


    def table1_callback(self, msg):
        if self.table1_msg:
            self.tray1_poses = TrayPoses(tray_poses = msg.tray_poses,
                            sensor_pose = msg.sensor_pose)


            # self.get_logger().info('tray in world frame: position x: "%s"' % self.tray1_poses.poses[0].position.x)
            # self.get_logger().info('tray1 pose : "%s"' % self.tray1_poses.poses[0].position.x)
            if self.tray1_poses.poses and self.tray1_poses.sensor_pose:
                self.table1_msg = False

    def table2_callback(self, msg):
        if self.table2_msg:
            self.tray2_poses = TrayPoses(tray_poses = msg.tray_poses,
                            sensor_pose = msg.sensor_pose)
            
            # self.get_logger().info('tray2 pose : "%s"' % self.tray2_poses.__str__())
            if self.tray2_poses.poses and self.tray2_poses.sensor_pose:
                self.table2_msg = False

    def left_bin_callback(self, msg):
        if self.left_bin_msg:
            self.part1_poses = PartPoses(part_poses = msg.part_poses,
                            sensor_pose = msg.sensor_pose)
            # print(part_poses.__str__())
            # self.get_logger().info('part1_poses : "%s"' % self.part1_poses.__str__())
            if self.part1_poses.poses and self.part1_poses.sensor_pose:
                self.part1_msg = False

    def right_bin_callback(self, msg):
        if self.right_bin_msg:
            self.part2_poses = PartPoses(part_poses = msg.part_poses,
                            sensor_pose = msg.sensor_pose)
            # self.get_logger().info('part2_poses : "%s"' % self.part2_poses.__str__())
            if self.part2_poses.poses and self.part2_poses.sensor_pose:
                self.part2_msg = False


    def parse_print(self):

        if (not (self.table1_msg and self.table2_msg and self.left_bin_msg and self.right_bin_msg)) and self.parse_flag:
            # self.get_logger().info('%s' % self.tray1_poses.__str__())
            # self.get_logger().info('tray1 pose : "%s"' % self.tray1_poses.ids)
            # self.get_logger().info('%s' % str(self.table1_msg)+str(self.table2_msg)+str(self.left_bin_msg)+str(self.right_bin_msg))
            output = "\n----------------------\n--- Order {} ---\n----------------------\n".format(self.order.id)
            self.get_logger().info('%s' % output)

            ############ Tray ##########
            tray_id = self.order.kitting_task.tray_id
            cur_tray_pose = None
            # for tray_poses in [self.tray1_poses, self.tray2_poses]:
            #     for tray_id, tray_pose in zip(tray_poses.ids, tray_poses.poses):
            #         if self.order.kitting_task.tray_id == tray_id:
            #             cur_tray_pose = tray_pose
            #             break
            all_tray_ids = [*self.tray1_poses.ids, *self.tray2_poses.ids]
            all_tray_poses = [*self.tray1_poses.poses, *self.tray2_poses.poses]

            for idx, t_id in enumerate(all_tray_ids):
                if t_id == tray_id:
                    cur_tray_pose = all_tray_poses[idx]
                    break
            assert cur_tray_pose is not None, "No trays matched tray ID in Order"
            output = "Tray:\n\t- id : {} \n\t- pose:\n\t\t- position: [{}, {}, {}] \n\t\t- orientation: [{}, {}, {}, {}]".format(tray_id, 
                                                                                                                cur_tray_pose.position.x, 
                                                                                                                cur_tray_pose.position.y, 
                                                                                                                cur_tray_pose.position.z, 
                                                                                                                cur_tray_pose.orientation.x, 
                                                                                                                cur_tray_pose.orientation.y, 
                                                                                                                cur_tray_pose.orientation.z,
                                                                                                                cur_tray_pose.orientation.w)
            self.get_logger().info('%s' % output)

            ######### Parts ##########
            output = "\nPart:\n"
            self.get_logger().info('%s' % output)
            
            # for each part in order
            for cur_part in self.order.kitting_task.parts:
                # for parts in left and right bins
                for part_poses in [self.part1_poses, self.part2_poses]:
                    # for every part in each bin
                    for part, part_pose in zip(part_poses, part_poses.poses):
                        if cur_part.color == part.parts.part_color and cur_part.type == part.parts.part_type:
                            cur_part_pose = part_pose
                            break

                output = "\t- {}\n\t\t- pose:\n\t\t\t- position: [{}, {}, {}] \n\t\t\t- orientation: [{}, {}, {}, {}]".format(self.order.kitting_task.parts.color,
                                                                                                                    cur_part_pose.position.x, 
                                                                                                                    cur_part_pose.position.y, 
                                                                                                                    cur_part_pose.position.z, 
                                                                                                                    cur_part_pose.orientation.x, 
                                                                                                                    cur_part_pose.orientation.y, 
                                                                                                                    cur_part_pose.orientation.z,
                                                                                                                    cur_tray_pose.orientation.w)
                self.get_logger().info('%s' % output)
            
            self.parse_flag = False
        # if self.tray1_poses is not None:
        #     self.get_logger().info('%s' % self.tray1_poses.__str__())
        # else:
        #     self.get_logger().info('%s' % self.tray1_poses)


def main(args=None):
    rclpy.init(args=args)
    my_node = rwa4()
    rclpy.spin(my_node)
    # rwa4.furthfunc()
    print("SDFsfdgsdfg")
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()