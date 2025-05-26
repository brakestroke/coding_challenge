#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, ParameterType
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist

class TwistPublisherNode(Node):
    """Node that publishes Twist messages based on dynamically configurable parameters."""
    
    def __init__(self):
        super().__init__('diff_vel_publisher')
        
        # Create parameter descriptors with ranges
        velocity_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Linear velocity in m/s',
            floating_point_range=[FloatingPointRange(
                from_value=-2.0,
                to_value=2.0,
                step=0.01
            )]
        )
        
        yaw_rate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Angular velocity (yaw rate) in rad/s',
            floating_point_range=[FloatingPointRange(
                from_value=-2.0,
                to_value=2.0,
                step=0.01
            )]
        )
        
        publish_rate_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Rate at which to publish messages (Hz)',
            floating_point_range=[FloatingPointRange(
                from_value=1.0,
                to_value=50.0,
                step=1.0
            )]
        )
        
        # Declare parameters with descriptors
        self.declare_parameter('velocity', 0.0, velocity_descriptor)
        self.declare_parameter('yaw_rate', 0.0, yaw_rate_descriptor)
        self.declare_parameter('publish_rate', 10.0, publish_rate_descriptor)
        self.declare_parameter('topic', '/diff_cmd_vel')
        
        # Get the initial publishing rate
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(Twist, self.topic, 10)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Twist publisher started. Publishing to: {self.topic}')
        self.get_logger().info('Set velocity and yaw_rate parameters to control the robot')
        self.get_logger().info('You can use "ros2 param set" command or rqt to change parameters')
    
    def parameters_callback(self, params):
        """Callback for parameter changes."""
        for param in params:
            if param.name == 'publish_rate' and param.type_ == ParameterType.PARAMETER_DOUBLE:
                # Update timer if publish_rate changes
                self.publish_rate = param.value
                self.timer.cancel()
                self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)
            elif param.name == 'topic' and param.type_ == ParameterType.PARAMETER_STRING:
                # Update topic if it changes
                self.topic = param.value
                self.pub = self.create_publisher(Twist, self.topic, 10)
        
        result = SetParametersResult()
        result.successful = True
        result.reason = "Parameters updated successfully"
        return result
    
    def timer_callback(self):
        """Publish twist message based on current parameter values."""
        # Get current parameter values
        velocity = self.get_parameter('velocity').value
        yaw_rate = self.get_parameter('yaw_rate').value
        
        # Create and publish Twist message
        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = yaw_rate
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()