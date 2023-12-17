import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf2_msgs.msg import TFMessage
import copy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs
import numpy as np
import math

TIME_PERIOD = 3

class target_to_goal_node(Node):
    def __init__(self):
        super().__init__('target_to_goal_node')

        self.inital_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(TIME_PERIOD, self.timer_callback)

        # self.time_period = TIME_PERIOD
        # self.tmr = self.create_timer(self.time_period, self.callback)

        self.calc_class = calc_relative_pos()

        # initialize
        self.init = False

        # goal pose 
        self.goal_pos = PoseStamped()

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'robot0', rclpy.time.Time())
            position = transform.transform.translation
            rotation = transform.transform.rotation
            # (roll, pitch, yaw) = self.euler_from_quaternion(rotation)
            self.goal_pos.pose.pose.position.x = position.x
            self.goal_pos.pose.pose.position.y = position.y            
            self.goal_pos.pose.pose.rotation.x = rotation.x            
            self.goal_pos.pose.pose.rotation.y = rotation.y            
            self.goal_pos.pose.pose.rotation.z = rotation.z            
            self.goal_pos.pose.pose.rotation.w = rotation.w            

                # for i in range(NUM_ROBOT):
                # # for i in range(1):
                #     self.pid[i].x = np.array([self.get_pos_list[i].x, self.get_pos_list[i].y, self.get_yaw_list[i], self.u_list[i][0], self.u_list[i][1]])
                #     self.pid[i].goal = np.array([self.goal_pos.pose.pose.position.x, self.goal_pos.pose.pose.position.y])
                                
            self.inital_pose_pub.publish(self.goal_pos)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info(f'Exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    class_node = target_to_goal_node()
    rclpy.spin(class_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
