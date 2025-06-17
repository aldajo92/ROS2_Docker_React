#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import random
import math
import time


class RandomDataPublisher(Node):
    def __init__(self):
        super().__init__('random_data_publisher')
        
        # Create publishers for different types of data
        self.temperature_pub = self.create_publisher(Float64, '/sensor_data/temperature', 10)
        self.humidity_pub = self.create_publisher(Float64, '/sensor_data/humidity', 10)
        self.position_x_pub = self.create_publisher(Float64, '/odom/pose/pose/position/x', 10)
        self.position_y_pub = self.create_publisher(Float64, '/odom/pose/pose/position/y', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Additional custom topics for more variety
        self.voltage_pub = self.create_publisher(Float64, '/sensor_data/battery_voltage', 10)
        self.pressure_pub = self.create_publisher(Float64, '/sensor_data/pressure', 10)
        
        # Timer to publish data at regular intervals
        self.timer = self.create_timer(0.1, self.publish_random_data)  # 10 Hz
        
        # Initialize some state variables for more realistic data
        self.time_start = time.time()
        self.position_x = 0.0
        self.position_y = 0.0
        self.temperature_base = 25.0  # Base temperature
        self.humidity_base = 50.0     # Base humidity
        
        self.get_logger().info('Random Data Publisher Node Started')
        self.get_logger().info('Publishing to topics:')
        self.get_logger().info('  - /sensor_data/temperature')
        self.get_logger().info('  - /sensor_data/humidity')
        self.get_logger().info('  - /sensor_data/battery_voltage')
        self.get_logger().info('  - /sensor_data/pressure')
        self.get_logger().info('  - /odom/pose/pose/position/x')
        self.get_logger().info('  - /odom/pose/pose/position/y')
        self.get_logger().info('  - /cmd_vel')

    def publish_random_data(self):
        current_time = time.time() - self.time_start
        
        # Temperature: simulate daily temperature variation with noise
        temperature = Float64()
        temperature.data = self.temperature_base + 5.0 * math.sin(current_time * 0.1) + random.uniform(-2.0, 2.0)
        self.temperature_pub.publish(temperature)
        
        # Humidity: simulate humidity changes with noise
        humidity = Float64()
        humidity.data = max(0.0, min(100.0, self.humidity_base + 10.0 * math.cos(current_time * 0.15) + random.uniform(-5.0, 5.0)))
        self.humidity_pub.publish(humidity)
        
        # Battery voltage: simulate battery discharge with noise
        voltage = Float64()
        voltage.data = 12.0 - (current_time * 0.001) + random.uniform(-0.2, 0.2)  # Slowly decreasing
        self.voltage_pub.publish(voltage)
        
        # Pressure: simulate atmospheric pressure changes
        pressure = Float64()
        pressure.data = 1013.25 + 20.0 * math.sin(current_time * 0.05) + random.uniform(-5.0, 5.0)
        self.pressure_pub.publish(pressure)
        
        # Position: simulate robot movement
        # Update position with some random walk
        self.position_x += random.uniform(-0.1, 0.1)
        self.position_y += random.uniform(-0.1, 0.1)
        
        pos_x = Float64()
        pos_x.data = self.position_x
        self.position_x_pub.publish(pos_x)
        
        pos_y = Float64()
        pos_y.data = self.position_y
        self.position_y_pub.publish(pos_y)
        
        # Cmd_vel: simulate velocity commands
        cmd_vel = Twist()
        cmd_vel.linear.x = 2.0 * math.sin(current_time * 0.3) + random.uniform(-0.5, 0.5)
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = math.cos(current_time * 0.2) + random.uniform(-0.3, 0.3)
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    
    node = RandomDataPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Random Data Publisher Node Shutting Down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 