#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time

class SimpleJointPublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_publisher')
        
        # Create joint state publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Create timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        
        # Joint names for the waver robot
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint', 
            'back_right_wheel_joint'
        ]
        
        # Initialize joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Time tracking for animation
        self.start_time = time.time()
        
        self.get_logger().info('🤖 Simple Joint Publisher started')
        self.get_logger().info(f'📊 Publishing joint states for: {self.joint_names}')

    def publish_joint_states(self):
        # Create joint state message
        joint_state = JointState()
        
        # Set header
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        # Set joint names
        joint_state.name = self.joint_names
        
        # Create some animated joint positions (simulate wheel rotation)
        current_time = time.time() - self.start_time
        wheel_rotation = math.sin(current_time * 2.0) * 0.5  # Slow rotation
        
        # All wheels rotate together for demo
        self.joint_positions = [wheel_rotation] * 4
        self.joint_velocities = [math.cos(current_time * 2.0)] * 4
        
        joint_state.position = self.joint_positions
        joint_state.velocity = self.joint_velocities
        joint_state.effort = [0.0] * 4  # No effort/torque data
        
        # Publish the joint state
        self.joint_pub.publish(joint_state)
        
        # Log every 50 messages (5 seconds at 10Hz)
        if hasattr(self, 'msg_count'):
            self.msg_count += 1
        else:
            self.msg_count = 1
            
        if self.msg_count % 50 == 0:
            self.get_logger().info(f'📡 Published {self.msg_count} joint state messages')

def main(args=None):
    rclpy.init(args=args)
    
    print("🚀 Starting Simple Joint State Publisher...")
    print("📊 This will publish joint states for the waver robot")
    print("🔗 React app should receive this data via rosbridge")
    
    joint_publisher = SimpleJointPublisher()
    
    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        print("🛑 Shutting down joint publisher...")
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()
        print("✅ Joint publisher stopped")

if __name__ == '__main__':
    main() 