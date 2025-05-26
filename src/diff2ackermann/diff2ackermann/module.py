#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException

class DiffToAckermann(Node):
    def __init__(self):
        super().__init__('diff_to_ackermann_converter')
        
        # Parameters
        self.wheelbase = 0.54838  # Distance between front and rear axles
        self.max_steering_angle = 0.69  # Maximum steering angle in radians
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'Wheelbase set to: {self.wheelbase} meters')
        
        # Subscribe to differential drive commands
        self.diff_sub = self.create_subscription(
            Twist,
            'diff_cmd_vel',
            self.diff_callback,
            10)
            
        # Publish Ackermann drive commands
        self.ack_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)        
        
        self.get_logger().info('Differential Drive to Ackermann converter started')
        
    def update_wheelbase_from_tf(self):
        """Calculate wheelbase from TF transforms between front and rear axles"""
        try:
            # Try to get transforms for front and rear wheel positions
            front_trans = self.tf_buffer.lookup_transform('base_link', 'fr_right_link', rclpy.time.Time())
            rear_trans = self.tf_buffer.lookup_transform('base_link', 're_right_link', rclpy.time.Time())
            
            # Extract positions
            front_x = front_trans.transform.translation.x
            rear_x = rear_trans.transform.translation.x
            
            # Calculate wheelbase as distance between front and rear axles
            new_wheelbase = abs(front_x - rear_x)
            

            self.wheelbase = new_wheelbase
            self.get_logger().info(f'Updated wheelbase from TF: {self.wheelbase} meters')
            
            return True
                
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(f'Could not determine wheelbase from TF: {str(e)}')
            self.get_logger().warning('Using default wheelbase value')
            
            return False
    
        
    def diff_callback(self, msg):
        # Create Ackermann message
        ack_msg = Twist()
        
        # Set speed to the linear x velocity
        ack_msg.linear.x = msg.linear.x
        
        # Convert angular velocity to steering angle
        
        if msg.angular.z == 0.0: # If no angular velocity, set steering angle to 0
            steer = 0.0
        else:
            turning_radius = msg.linear.x / msg.angular.z
            
            # Calculate steering angle based on turning radius and wheelbase
            steer = math.atan(self.wheelbase / turning_radius)
            
        if abs(steer) > self.max_steering_angle:
            self.get_logger().warning(f'Desired yaw rate cannot be achieved, not driving')
            self.get_logger().warning(f'steer={steer}, max_steering_angle={self.max_steering_angle}')
            ack_msg.angular.z = 0.0
            ack_msg.linear.x = 0.0
        else:
            ack_msg.angular.z = steer
        
            
        # Publish the Ackermann message
        self.ack_pub.publish(ack_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DiffToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()