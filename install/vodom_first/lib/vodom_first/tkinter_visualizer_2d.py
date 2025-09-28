#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path
import tkinter as tk
from tkinter import ttk
import threading
import math
from collections import deque
import time

class TkinterVisualizer2D(Node):
    def __init__(self):
        super().__init__('tkinter_visualizer_2d')
        
        # Tkinter setup
        self.root = tk.Tk()
        self.root.title('2D Visual Odometry - Real-time Visualization')
        self.root.geometry('1000x700')
        self.root.configure(bg='black')
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Canvas for drawing
        self.canvas = tk.Canvas(main_frame, bg='black', width=800, height=600)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Info panel
        info_frame = ttk.Frame(main_frame)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        
        # Info labels
        ttk.Label(info_frame, text="Visual Odometry Info", font=('Arial', 12, 'bold')).pack(pady=(0, 10))
        
        self.info_vars = {
            'frames': tk.StringVar(value="Frames: 0"),
            'points': tk.StringVar(value="Points: 0"),
            'distance': tk.StringVar(value="Distance: 0.0m"),
            'position': tk.StringVar(value="Position: (0.0, 0.0)"),
            'heading': tk.StringVar(value="Heading: 0.0°"),
            'status': tk.StringVar(value="Status: Waiting...")
        }
        
        for var in self.info_vars.values():
            ttk.Label(info_frame, textvariable=var, font=('Arial', 10)).pack(anchor=tk.W, pady=2)
        
        # Controls
        ttk.Label(info_frame, text="Controls", font=('Arial', 12, 'bold')).pack(pady=(20, 10))
        ttk.Button(info_frame, text="Reset View", command=self.reset_view).pack(fill=tk.X, pady=2)
        ttk.Button(info_frame, text="Clear Path", command=self.clear_path).pack(fill=tk.X, pady=2)
        
        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(info_frame, text="Follow Vehicle", variable=self.follow_var).pack(anchor=tk.W, pady=2)
        
        # Zoom controls
        zoom_frame = ttk.Frame(info_frame)
        zoom_frame.pack(fill=tk.X, pady=5)
        ttk.Button(zoom_frame, text="Zoom +", command=self.zoom_in, width=8).pack(side=tk.LEFT)
        ttk.Button(zoom_frame, text="Zoom -", command=self.zoom_out, width=8).pack(side=tk.RIGHT)
        
        # Data storage
        self.trajectory = deque(maxlen=1000)
        self.current_pose = None
        self.running = True
        
        # View parameters
        self.zoom = 30  # pixels per meter
        self.center_x = 400  # canvas center
        self.center_y = 300
        self.offset_x = 0
        self.offset_y = 0
        
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
        
        # Bind canvas events
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<MouseWheel>", self.on_mousewheel)
        
        # Start update timer
        self.update_display()
        
        self.get_logger().info('Tkinter 2D Visualizer initialized')

    def pose2d_callback(self, msg):
        """Update current pose"""
        try:
            # Validate incoming data
            if (math.isnan(msg.x) or math.isnan(msg.y) or math.isnan(msg.theta) or
                math.isinf(msg.x) or math.isinf(msg.y) or math.isinf(msg.theta)):
                self.get_logger().warn(f"Invalid pose data received: x={msg.x}, y={msg.y}, theta={msg.theta}")
                return
            
            self.current_pose = msg
            
            # Add to trajectory
            self.trajectory.append((msg.x, msg.y, msg.theta))
            
            # Calculate distance
            if self.last_pose is not None:
                dx = msg.x - self.last_pose[0]
                dy = msg.y - self.last_pose[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Only add reasonable distances (avoid jumps due to bad data)
                if not math.isnan(distance) and not math.isinf(distance) and distance < 100.0:
                    self.total_distance += distance
            
            self.last_pose = (msg.x, msg.y)
            self.frame_count += 1
            
            # Update info
            self.info_vars['status'].set("Status: Running")
            
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
                    
                    # Validate coordinates
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
                    
                    # Calculate distance
                    if prev_x is not None:
                        dx = x - prev_x
                        dy = y - prev_y
                        distance = math.sqrt(dx*dx + dy*dy)
                        
                        # Only add reasonable distances
                        if not math.isnan(distance) and not math.isinf(distance) and distance < 100.0:
                            self.total_distance += distance
                    
                    prev_x, prev_y = x, y
                
                self.frame_count = len(self.trajectory)
                
        except Exception as e:
            self.get_logger().error(f"Error in path_callback: {e}")

    def world_to_canvas(self, world_x, world_y):
        """Convert world coordinates to canvas coordinates"""
        try:
            # Validate input coordinates
            if math.isnan(world_x) or math.isnan(world_y) or math.isinf(world_x) or math.isinf(world_y):
                return 0, 0
            
            # Ensure zoom is valid
            zoom = max(self.zoom, 0.01)  # Prevent division by zero
            
            if self.follow_var.get() and self.current_pose:
                # Validate current pose
                current_x = self.current_pose.x if not math.isnan(self.current_pose.x) else 0.0
                current_y = self.current_pose.y if not math.isnan(self.current_pose.y) else 0.0
                
                # Center on vehicle
                canvas_x = self.center_x + (world_x - current_x) * zoom
                canvas_y = self.center_y - (world_y - current_y) * zoom
            else:
                # Fixed view
                canvas_x = self.center_x + (world_x + self.offset_x) * zoom
                canvas_y = self.center_y - (world_y + self.offset_y) * zoom
            
            # Validate output coordinates
            if math.isnan(canvas_x) or math.isnan(canvas_y) or math.isinf(canvas_x) or math.isinf(canvas_y):
                return 0, 0
                
            return canvas_x, canvas_y
            
        except (ValueError, OverflowError):
            return 0, 0

    def draw_grid(self):
        """Draw coordinate grid"""
        try:
            self.canvas.delete("grid")
            
            # Get canvas dimensions safely
            try:
                canvas_width = self.canvas.winfo_width()
                canvas_height = self.canvas.winfo_height()
                
                # Skip if canvas not properly initialized
                if canvas_width <= 1 or canvas_height <= 1:
                    return
            except tk.TclError:
                return
            
            if self.follow_var.get() and self.current_pose:
                center_world_x, center_world_y = self.current_pose.x, self.current_pose.y
                # Validate center coordinates
                if math.isnan(center_world_x) or math.isnan(center_world_y):
                    center_world_x, center_world_y = 0.0, 0.0
            else:
                center_world_x, center_world_y = -self.offset_x, -self.offset_y
            
            # Grid spacing
            if self.zoom > 50:
                grid_spacing = 1.0
            elif self.zoom > 20:
                grid_spacing = 2.0
            else:
                grid_spacing = 5.0
            
            # Calculate world dimensions
            world_width = canvas_width / max(self.zoom, 0.1)  # Avoid division by zero
            world_height = canvas_height / max(self.zoom, 0.1)
            
            # Calculate grid bounds
            start_x = int((center_world_x - world_width/2) / grid_spacing) * grid_spacing
            end_x = int((center_world_x + world_width/2) / grid_spacing + 1) * grid_spacing
            
            start_y = int((center_world_y - world_height/2) / grid_spacing) * grid_spacing  
            end_y = int((center_world_y + world_height/2) / grid_spacing + 1) * grid_spacing
            
            # Limit number of grid lines to prevent performance issues
            max_lines = 100
            x_step = max(int(grid_spacing), int((end_x - start_x) / max_lines))
            y_step = max(int(grid_spacing), int((end_y - start_y) / max_lines))
            
            # Draw vertical lines
            for x in range(int(start_x), int(end_x), x_step):
                try:
                    x1, y1 = self.world_to_canvas(x, center_world_y - world_height/2)
                    x2, y2 = self.world_to_canvas(x, center_world_y + world_height/2)
                    
                    # Validate coordinates
                    if (not any(math.isnan(coord) or math.isinf(coord) for coord in [x1, y1, x2, y2]) and
                        0 <= x1 <= canvas_width):
                        self.canvas.create_line(int(x1), int(y1), int(x2), int(y2), 
                                              fill='gray25', tags="grid")
                except (ValueError, OverflowError, tk.TclError):
                    continue
            
            # Draw horizontal lines
            for y in range(int(start_y), int(end_y), y_step):
                try:
                    x1, y1 = self.world_to_canvas(center_world_x - world_width/2, y)
                    x2, y2 = self.world_to_canvas(center_world_x + world_width/2, y)
                    
                    # Validate coordinates  
                    if (not any(math.isnan(coord) or math.isinf(coord) for coord in [x1, y1, x2, y2]) and
                        0 <= y1 <= canvas_height):
                        self.canvas.create_line(int(x1), int(y1), int(x2), int(y2), 
                                              fill='gray25', tags="grid")
                except (ValueError, OverflowError, tk.TclError):
                    continue
                    
        except Exception as e:
            self.get_logger().error(f"Error in draw_grid: {e}")

    def draw_trajectory(self):
        """Draw vehicle trajectory"""
        try:
            self.canvas.delete("trajectory")
            
            if len(self.trajectory) < 2:
                return
            
            # Get canvas dimensions safely
            try:
                canvas_width = self.canvas.winfo_width()
                canvas_height = self.canvas.winfo_height()
                
                # Skip if canvas not properly initialized
                if canvas_width <= 1 or canvas_height <= 1:
                    return
            except tk.TclError:
                return
            
            points = []
            for world_x, world_y, _ in self.trajectory:
                try:
                    canvas_x, canvas_y = self.world_to_canvas(world_x, world_y)
                    
                    # Validate coordinates
                    if (not math.isnan(canvas_x) and not math.isnan(canvas_y) and
                        not math.isinf(canvas_x) and not math.isinf(canvas_y) and
                        -1000 <= canvas_x <= canvas_width + 1000 and 
                        -1000 <= canvas_y <= canvas_height + 1000):
                        points.extend([int(canvas_x), int(canvas_y)])
                except (ValueError, OverflowError):
                    continue
            
            # Draw trajectory if we have valid points
            if len(points) >= 4:
                try:
                    self.canvas.create_line(points, fill='cyan', width=2, tags="trajectory", smooth=True)
                except tk.TclError as e:
                    self.get_logger().warn(f"Failed to draw trajectory: {e}")
                    # Fallback: draw without smoothing
                    try:
                        self.canvas.create_line(points, fill='cyan', width=2, tags="trajectory")
                    except tk.TclError:
                        pass  # Skip this frame
                        
        except Exception as e:
            self.get_logger().error(f"Error in draw_trajectory: {e}")
            # Don't crash, just skip this frame

    def draw_vehicle(self):
        """Draw current vehicle position and orientation"""
        try:
            self.canvas.delete("vehicle")
            
            if not self.current_pose:
                return
            
            canvas_x, canvas_y = self.world_to_canvas(self.current_pose.x, self.current_pose.y)
            
            # Validate coordinates
            if (math.isnan(canvas_x) or math.isnan(canvas_y) or 
                math.isinf(canvas_x) or math.isinf(canvas_y)):
                return
            
            # Draw vehicle as triangle
            angle = self.current_pose.theta
            if math.isnan(angle) or math.isinf(angle):
                angle = 0.0
                
            size = max(8, int(self.zoom * 0.2))
            
            # Triangle points
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)
            
            front_x = canvas_x + size * cos_a
            front_y = canvas_y - size * sin_a
            
            back_left_x = canvas_x - (size * 0.3) * cos_a - (size * 0.5) * sin_a
            back_left_y = canvas_y + (size * 0.3) * sin_a - (size * 0.5) * cos_a
            
            back_right_x = canvas_x - (size * 0.3) * cos_a + (size * 0.5) * sin_a
            back_right_y = canvas_y + (size * 0.3) * sin_a + (size * 0.5) * cos_a
            
            # Validate all points
            points = [front_x, front_y, back_left_x, back_left_y, back_right_x, back_right_y]
            if any(math.isnan(p) or math.isinf(p) for p in points):
                # Fallback: draw simple circle
                try:
                    self.canvas.create_oval(canvas_x-5, canvas_y-5, canvas_x+5, canvas_y+5, 
                                          fill='red', tags="vehicle")
                except tk.TclError:
                    pass
                return
            
            # Draw triangle
            try:
                self.canvas.create_polygon([int(p) for p in points],
                                         fill='red', outline='white', width=1, tags="vehicle")
                
                # Draw center dot
                self.canvas.create_oval(int(canvas_x-3), int(canvas_y-3), 
                                      int(canvas_x+3), int(canvas_y+3), 
                                      fill='yellow', tags="vehicle")
            except tk.TclError as e:
                self.get_logger().warn(f"Failed to draw vehicle: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error in draw_vehicle: {e}")

    def update_display(self):
        """Update the display periodically"""
        if not self.running:
            return
        
        try:
            # Update info labels safely
            self.info_vars['frames'].set(f"Frames: {self.frame_count}")
            self.info_vars['points'].set(f"Points: {len(self.trajectory)}")
            self.info_vars['distance'].set(f"Distance: {self.total_distance:.2f}m")
            
            if self.current_pose:
                # Validate pose values
                x = self.current_pose.x if not math.isnan(self.current_pose.x) else 0.0
                y = self.current_pose.y if not math.isnan(self.current_pose.y) else 0.0
                theta = self.current_pose.theta if not math.isnan(self.current_pose.theta) else 0.0
                
                self.info_vars['position'].set(f"Position: ({x:.2f}, {y:.2f})")
                self.info_vars['heading'].set(f"Heading: {math.degrees(theta):.1f}°")
            
            # Redraw everything with error handling
            try:
                self.draw_grid()
            except Exception as e:
                self.get_logger().warn(f"Failed to draw grid: {e}")
                
            try:
                self.draw_trajectory()
            except Exception as e:
                self.get_logger().warn(f"Failed to draw trajectory: {e}")
                
            try:
                self.draw_vehicle()
            except Exception as e:
                self.get_logger().warn(f"Failed to draw vehicle: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Error in update_display: {e}")
        
        # Always schedule next update, even if there were errors
        if self.running:
            try:
                self.root.after(50, self.update_display)  # 20 FPS
            except tk.TclError:
                self.running = False

    def reset_view(self):
        """Reset view parameters"""
        self.zoom = 30
        self.offset_x = 0
        self.offset_y = 0

    def clear_path(self):
        """Clear trajectory"""
        self.trajectory.clear()
        self.total_distance = 0.0
        self.frame_count = 0

    def zoom_in(self):
        """Increase zoom"""
        self.zoom = min(100, self.zoom * 1.2)

    def zoom_out(self):
        """Decrease zoom"""
        self.zoom = max(5, self.zoom / 1.2)

    def on_click(self, event):
        """Handle mouse click"""
        self.last_x = event.x
        self.last_y = event.y

    def on_drag(self, event):
        """Handle mouse drag"""
        if not self.follow_var.get():
            dx = (event.x - self.last_x) / self.zoom
            dy = (event.y - self.last_y) / self.zoom
            self.offset_x += dx
            self.offset_y -= dy
            self.last_x = event.x
            self.last_y = event.y

    def on_mousewheel(self, event):
        """Handle mouse wheel for zooming"""
        if event.delta > 0:
            self.zoom_in()
        else:
            self.zoom_out()

    def run_visualization(self):
        """Run the visualization"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.get_logger().info("Visualization closed")

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = TkinterVisualizer2D()
    
    # Run ROS2 in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(visualizer,))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Run visualization in main thread
        visualizer.run_visualization()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.running = False
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()