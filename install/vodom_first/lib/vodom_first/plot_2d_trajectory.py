#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
import threading
import math
from collections import deque
import time

class SimpleVisualizer2D(Node):
    def __init__(self):
        super().__init__('simple_visualizer_2d')
        
        # Data storage
        self.trajectory = deque(maxlen=1000)
        self.current_pose = None
        
        # Stats
        self.frame_count = 0
        self.total_distance = 0.0
        self.last_pose = None
        
        # ROS2 subscriptions
        self.pose2d_sub = self.create_subscription(
            Pose2D,
            'vo_pose_2d',
            self.pose2d_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            'vo_path_uliXD',
            self.path_callback,
            10
        )
        
        # Timer for periodic updates
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('Simple 2D Visualizer initialized (console output only)')

    def pose2d_callback(self, msg):
        """Update current pose"""
        try:
            # Validate incoming data
            if (math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.theta) or
                math.isinf(msg.x) or math.isinf(msg.y) or math.isinf(msg.theta)):
                self.get_logger().warn(f"Invalid pose data received: x={msg.x}, y={msg.y}, theta={msg.theta}")
                return
            
            self.current_pose = msg
            self.trajectory.append((msg.x, msg.y, msg.theta))
            
            # Calculate distance
            if self.last_pose is not None:
                dx = msg.x - self.last_pose[0]
                dy = msg.y - self.last_pose[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if not math.isnan(distance) and not math.isinf(distance) and distance < 100.0:
                    self.total_distance += distance
            
            self.last_pose = (msg.x, msg.y)
            self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f"Error in pose2d_callback: {e}")

    def path_callback(self, msg):
        """Handle complete path updates"""
        try:
            if len(msg.poses) > 0:
                self.trajectory.clear()
                self.total_distance = 0.0
                
                prev_x, prev_y = None, None
                for pose_stamped in msg.poses:
                    x = pose_stamped.pose.position.x
                    y = pose_stamped.pose.position.y
                    
                    if (math.isnan(x) or math.isnan(y) or
                        math.isinf(x) or math.isinf(y)):
                        continue
                    
                    # Extract yaw from quaternion
                    q = pose_stamped.pose.orientation
                    try:
                        yaw = 2 * math.atan2(q.z, q.w)
                        if math.isnan(yaw) or math.isinf(yaw):
                            yaw = 0.0
                    except (ValueError, ZeroDivisionError):
                        yaw = 0.0
                    
                    self.trajectory.append((x, y, yaw))
                    
                    if prev_x is not None:
                        dx = x - prev_x
                        dy = y - prev_y
                        distance = math.sqrt(dx*dx + dy*dy)
                        
                        if not math.isnan(distance) and not math.isinf(distance) and distance < 100.0:
                            self.total_distance += distance
                    
                    prev_x, prev_y = x, y
                
                self.frame_count = len(self.trajectory)
                
        except Exception as e:
            self.get_logger().error(f"Error in path_callback: {e}")

    def print_status(self):
        """Print status to console"""
        if self.current_pose:
            self.get_logger().info(
                f"Frames: {self.frame_count} | "
                f"Points: {len(self.trajectory)} | "
                f"Distance: {self.total_distance:.2f}m | "
                f"Position: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}) | "
                f"Heading: {math.degrees(self.current_pose.theta):.1f}°"
            )
        else:
            self.get_logger().info(f"Waiting for pose data... ({self.frame_count} frames)")

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = SimpleVisualizer2D()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()