#!/usr/bin/env python3

import time
import sys
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading
import struct
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

class BallLocalizer3D:
    def __init__(self):
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.data_lock = threading.Lock()
        self.has_odom_data = False
        
        # Point cloud data
        self.latest_pointcloud = None
        self.pointcloud_lock = threading.Lock()
        self.has_pointcloud = False
        
        # Ball state
        self.current_ball_position = None  # 3D world coordinates from point cloud
        self.ball_detected = False
        self.current_ball_candidate = None  # 2D candidate position for visualization
        
        # Ball detection parameters
        self.ball_diameter = 0.5  # meters (50cm diameter ball)
        
        # Specific green ball HSV color range for RGB(90, 180, 120)
        target_rgb = np.uint8([[[90, 180, 120]]])
        target_hsv = cv2.cvtColor(target_rgb, cv2.COLOR_RGB2HSV)[0][0]
        print(f"Target RGB(90,180,120) = HSV({target_hsv[0]}, {target_hsv[1]}, {target_hsv[2]})")
        
        # More relaxed HSV range around the target color
        h_tolerance = 25  # Increased from 15
        s_tolerance = 80  # Increased from 60
        v_tolerance = 80  # Increased from 60
        
        self.hsv_lower = np.array([
            max(0, target_hsv[0] - h_tolerance),
            max(0, target_hsv[1] - s_tolerance), 
            max(0, target_hsv[2] - v_tolerance)
        ])
        self.hsv_upper = np.array([
            min(179, target_hsv[0] + h_tolerance),
            min(255, target_hsv[1] + s_tolerance),
            min(255, target_hsv[2] + v_tolerance)
        ])
        print(f"HSV range: {self.hsv_lower} to {self.hsv_upper}")
        
        # Visualization
        plt.ion()
        self.fig, (self.ax_map, self.ax_camera) = plt.subplots(1, 2, figsize=(15, 8))
        self.setup_plots()
        
        # Camera client
        self.video_client = VideoClient()
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()
        
        # Create lidar switch publisher
        self.lidar_switch_publisher = ChannelPublisher("rt/utlidar/switch", String_)
        self.lidar_switch_publisher.Init()
        
        # Setup subscribers
        self.setup_subscribers()
        
    def setup_plots(self):
        """Setup matplotlib plots"""
        # Map plot (3D visualization)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        self.ax_map.set_title('3D Ball Localization (Point Cloud Based)')
        
        # Camera plot
        self.ax_camera.set_title('Camera View - Ball Detection + Point Cloud')
        self.ax_camera.axis('off')
        
    def setup_subscribers(self):
        """Setup subscribers for robot odometry and point cloud data"""
        try:
            self.pose_subscriber = ChannelSubscriber("rt/utlidar/robot_pose", PoseStamped_)
            self.pose_subscriber.Init(self.pose_handler, 10)
            print("✅ Subscribed to robot pose")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to robot_pose: {e}")
            
        try:
            self.odom_subscriber = ChannelSubscriber("rt/lio_sam_ros2/mapping/odometry", Odometry_)
            self.odom_subscriber.Init(self.odom_handler, 10)
            print("✅ Subscribed to SLAM odometry")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to SLAM odometry: {e}")
            
        # Subscribe to point cloud data
        try:
            self.pointcloud_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
            self.pointcloud_subscriber.Init(self.pointcloud_handler, 10)
            print("✅ Subscribed to lidar point cloud")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to point cloud: {e}")
            
        # Try to subscribe to point cloud to image projection if available
        try:
            # This topic might provide the mapping between camera pixels and 3D points
            self.pc_to_image_subscriber = ChannelSubscriber("rt/pctoimage_local", PointCloud2_)
            self.pc_to_image_subscriber.Init(self.pc_to_image_handler, 10)
            print("✅ Subscribed to point cloud to image projection")
        except Exception as e:
            print(f"⚠️  Point cloud to image projection not available: {e}")
            
    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def pose_handler(self, msg: PoseStamped_):
        """Handle robot pose updates"""
        with self.data_lock:
            self.robot_x = msg.pose.position.x
            self.robot_y = msg.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w
            )
            self.has_odom_data = True
            
    def odom_handler(self, msg: Odometry_):
        """Handle SLAM odometry updates"""
        with self.data_lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            )
            self.has_odom_data = True
            
    def pointcloud_handler(self, msg: PointCloud2_):
        """Handle point cloud updates"""
        with self.pointcloud_lock:
            self.latest_pointcloud = msg
            self.has_pointcloud = True
            
    def pc_to_image_handler(self, msg: PointCloud2_):
        """Handle point cloud to image projection data"""
        # This might contain pre-computed mappings between image pixels and 3D points
        print(f"Received PC to image data: {msg.width} x {msg.height} points")
        
    def parse_pointcloud(self, msg: PointCloud2_):
        """Parse point cloud message into 3D points"""
        points = []
        try:
            point_step = msg.point_step
            data = bytes(msg.data)
            
            for i in range(msg.width):
                offset = i * point_step
                if offset + point_step <= len(data):
                    # Unpack point data: x, y, z (float32), skip padding, intensity (float32), ring, time
                    point_data = struct.unpack_from('<fff4xf H 2x f', data, offset)
                    x, y, z, intensity, ring, timestamp = point_data
                    
                    # Filter reasonable points (not too far, not at origin)
                    distance = math.sqrt(x**2 + y**2 + z**2)
                    if 0.1 < distance < 15.0 and not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points.append({
                            'x': x, 'y': y, 'z': z,
                            'intensity': intensity,
                            'ring': ring,
                            'distance': distance
                        })
        except Exception as e:
            print(f"Error parsing point cloud: {e}")
            
        return points
        
    def detect_green_ball_2d(self, frame):
        """Detect green ball in camera frame (2D detection only)"""
        if frame is None:
            return None, None
            
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for green color
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Clean up mask
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return frame, None
            
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < 150:  # Reduced minimum area (was 200)
            return frame, None
            
        # Get bounding circle
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        if radius < 10:  # Reduced minimum radius (was 15)
            return frame, None
            
        # Check circularity
        perimeter = cv2.arcLength(largest_contour, True)
        if perimeter > 0:
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.4:  # Must be somewhat circular
                return frame, None
        else:
            return frame, None
            
        # Draw detection
        cv2.circle(frame, center, radius, (0, 255, 255), 2)
        cv2.circle(frame, center, 2, (0, 0, 255), -1)
        cv2.putText(frame, f'Ball candidate: r={radius}', (center[0]-80, center[1]-radius-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return frame, {'center': center, 'radius': radius, 'area': area, 'pixel_x': center[0], 'pixel_y': center[1]}
        
    def get_3d_position_from_camera_point(self, pixel_x, pixel_y):
        """Get 3D world position from camera pixel using point cloud data"""
        if not self.has_pointcloud:
            return None
            
        with self.pointcloud_lock:
            if self.latest_pointcloud is None:
                return None
            pointcloud = self.latest_pointcloud
            
        # Parse the point cloud
        points_3d = self.parse_pointcloud(pointcloud)
        if not points_3d:
            return None
            
        # Project 3D points to camera coordinates and find closest to detection
        # This is a simplified approach - in practice you'd need camera calibration
        # For now, let's find points in the general direction of the ball
        
        with self.data_lock:
            robot_x, robot_y, robot_yaw = self.robot_x, self.robot_y, self.robot_yaw
            
        # Convert pixel to normalized camera coordinates
        # Assuming camera is centered and has known FOV
        frame_width = 640  # Typical camera width
        frame_height = 480  # Typical camera height
        
        # Normalize pixel coordinates (-1 to 1)
        norm_x = (pixel_x - frame_width/2) / (frame_width/2)
        norm_y = (pixel_y - frame_height/2) / (frame_height/2)
        
        # Calculate viewing direction in robot frame
        camera_fov_h = 70.0  # degrees horizontal FOV
        camera_fov_v = 50.0  # degrees vertical FOV (estimated)
        
        # Convert to angles
        angle_h = norm_x * (camera_fov_h / 2) * (math.pi / 180)
        angle_v = -norm_y * (camera_fov_v / 2) * (math.pi / 180)  # Negative because image Y is flipped
        
        # Find points in point cloud that are in this direction
        candidate_points = []
        
        for point in points_3d:
            # Point is in robot frame
            px, py, pz = point['x'], point['y'], point['z']
            
            # Calculate angles to this point
            point_distance = math.sqrt(px**2 + py**2 + pz**2)
            if point_distance < 0.5 or point_distance > 10.0:
                continue
                
            point_angle_h = math.atan2(py, px)
            point_angle_v = math.atan2(pz, math.sqrt(px**2 + py**2))
            
            # Check if point is in the viewing direction (with tolerance)
            angle_tolerance_h = 0.2  # radians (~11 degrees) - increased tolerance
            angle_tolerance_v = 0.25  # radians (~14 degrees) - increased tolerance
            
            if (abs(point_angle_h - angle_h) < angle_tolerance_h and 
                abs(point_angle_v - angle_v) < angle_tolerance_v):
                candidate_points.append(point)
                
        if not candidate_points:
            print(f"🔍 No 3D points found for camera detection at ({pixel_x}, {pixel_y}) - expected angles: h={angle_h*180/math.pi:.1f}°, v={angle_v*180/math.pi:.1f}°")
            return None
            
        # Find the closest reasonable point (likely the ball surface)
        # For a 50cm ball, look for points at reasonable distances
        best_point = None
        best_score = float('inf')
        
        for point in candidate_points:
            distance = point['distance']
            
            # Score based on distance (prefer closer points) and intensity
            score = distance  # Prefer closer points
            if point['intensity'] > 50:  # Prefer high-intensity returns
                score *= 0.8
                
            if score < best_score:
                best_score = score
                best_point = point
                
        if best_point:
            # Convert to world coordinates
            px, py, pz = best_point['x'], best_point['y'], best_point['z']
            
            # Transform from robot frame to world frame
            world_x = robot_x + px * math.cos(robot_yaw) - py * math.sin(robot_yaw)
            world_y = robot_y + px * math.sin(robot_yaw) + py * math.cos(robot_yaw)
            world_z = pz  # Assuming robot and world Z are aligned
            
            return {
                'x': world_x,
                'y': world_y, 
                'z': world_z,
                'distance': best_point['distance'],
                'intensity': best_point['intensity']
            }
            
        return None
        
    def validate_ball_size_3d(self, ball_3d_pos):
        """Validate that the detected object is approximately 50cm using 3D data"""
        if not ball_3d_pos or not self.has_pointcloud:
            return False
            
        # Get all points near the detected ball position
        with self.pointcloud_lock:
            if self.latest_pointcloud is None:
                return False
            points_3d = self.parse_pointcloud(self.latest_pointcloud)
            
        ball_x, ball_y, ball_z = ball_3d_pos['x'], ball_3d_pos['y'], ball_3d_pos['z']
        
        # Find all points within ball radius + some tolerance
        search_radius = self.ball_diameter * 0.7  # 70% of ball diameter
        nearby_points = []
        
        for point in points_3d:
            # Convert to world coordinates
            with self.data_lock:
                robot_x, robot_y, robot_yaw = self.robot_x, self.robot_y, self.robot_yaw
                
            px, py, pz = point['x'], point['y'], point['z']
            world_x = robot_x + px * math.cos(robot_yaw) - py * math.sin(robot_yaw)
            world_y = robot_y + px * math.sin(robot_yaw) + py * math.cos(robot_yaw)
            world_z = pz
            
            # Check distance to ball center
            dist_to_ball = math.sqrt((world_x - ball_x)**2 + (world_y - ball_y)**2 + (world_z - ball_z)**2)
            
            if dist_to_ball < search_radius:
                nearby_points.append({
                    'x': world_x, 'y': world_y, 'z': world_z,
                    'distance_to_center': dist_to_ball
                })
                
        # Check if we have reasonable number of points for a 50cm ball
        expected_points = 10  # Minimum points expected for a 50cm ball
        if len(nearby_points) < expected_points:
            return False
            
        # Check if points form roughly spherical distribution
        if nearby_points:
            distances = [p['distance_to_center'] for p in nearby_points]
            avg_distance = np.mean(distances)
            std_distance = np.std(distances)
            
            # Ball surface should be relatively consistent distance from center
            if std_distance > self.ball_diameter * 0.15:  # 15% of ball diameter
                return False
                
            # Average distance should be roughly ball radius
            expected_radius = self.ball_diameter / 2
            if abs(avg_distance - expected_radius) > expected_radius * 0.4:  # 40% tolerance
                return False
                
        return True
    
    def enable_lidar(self):
        """Turn on the lidar"""
        print("🔦 Enabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "ON"
        self.lidar_switch_publisher.Write(switch_msg)
        time.sleep(2)  # Give lidar time to start
        
    def disable_lidar(self):
        """Turn off the lidar"""
        print("🔦 Disabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "OFF"
        self.lidar_switch_publisher.Write(switch_msg)
        
    def estimate_ball_candidate_position(self, pixel_x, pixel_y, radius_pixels):
        """Estimate ball position using camera-only method for candidates without 3D data"""
        frame_width = 640
        frame_height = 480
        
        # Convert pixel to normalized image coordinates
        norm_x = (pixel_x - frame_width/2) / (frame_width/2)
        norm_y = (pixel_y - frame_height/2) / (frame_height/2)
        
        # Calculate angle from camera center
        camera_fov_horizontal = 70.0  # degrees
        angle_horizontal = norm_x * (camera_fov_horizontal / 2) * (math.pi / 180)
        
        # Estimate distance based on ball size in pixels (rough approximation)
        focal_length_pixels = frame_width / (2 * math.tan(math.radians(camera_fov_horizontal / 2)))
        pixel_diameter = radius_pixels * 2
        if pixel_diameter > 0:
            estimated_distance = (self.ball_diameter * focal_length_pixels) / pixel_diameter
            estimated_distance = max(0.5, min(estimated_distance, 8.0))  # Reasonable range
        else:
            estimated_distance = 3.0  # Default estimate
        
        # Convert to robot-relative coordinates
        rel_x = estimated_distance * math.cos(angle_horizontal)
        rel_y = estimated_distance * math.sin(angle_horizontal)
        
        # Convert to world coordinates
        with self.data_lock:
            world_x = self.robot_x + rel_x * math.cos(self.robot_yaw) - rel_y * math.sin(self.robot_yaw)
            world_y = self.robot_y + rel_x * math.sin(self.robot_yaw) + rel_y * math.cos(self.robot_yaw)
            
        return {'x': world_x, 'y': world_y, 'z': 0.25, 'estimated': True}  # Assume ball is 25cm off ground
    
    def update_ball_position(self, ball_3d_pos):
        """Update ball position with 3D coordinates"""
        self.current_ball_position = (ball_3d_pos['x'], ball_3d_pos['y'], ball_3d_pos['z'])
        self.ball_detected = True
        print(f"🟢 Ball detected at 3D position: ({ball_3d_pos['x']:.2f}, {ball_3d_pos['y']:.2f}, {ball_3d_pos['z']:.2f}) "
              f"distance: {ball_3d_pos['distance']:.2f}m, intensity: {ball_3d_pos['intensity']:.1f}")
    
    def update_ball_candidate(self, candidate_pos):
        """Update ball candidate position (camera-only estimate)"""
        self.current_ball_candidate = (candidate_pos['x'], candidate_pos['y'], candidate_pos['z'])
        print(f"🟡 Ball candidate at estimated position: ({candidate_pos['x']:.2f}, {candidate_pos['y']:.2f}, {candidate_pos['z']:.2f}) [camera-only]")
        
    def update_visualization(self, frame, ball_detection_2d):
        """Update visualization with 3D ball position"""
        # Update camera view
        self.ax_camera.clear()
        if frame is not None:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.ax_camera.imshow(frame_rgb)
        self.ax_camera.set_title('Camera + Point Cloud Ball Detection')
        self.ax_camera.axis('off')
        
        # Update map view
        self.ax_map.clear()
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        
        # Draw robot
        with self.data_lock:
            robot_x, robot_y, robot_yaw = self.robot_x, self.robot_y, self.robot_yaw
            
        # Robot arrow
        arrow_length = 0.5
        dx = arrow_length * math.cos(robot_yaw)
        dy = arrow_length * math.sin(robot_yaw)
        self.ax_map.arrow(robot_x, robot_y, dx, dy, head_width=0.2, head_length=0.2,
                         fc='blue', ec='blue', alpha=0.8, label='Robot')
        
        # Collect all positions to determine map bounds
        all_x = [robot_x]
        all_y = [robot_y]
        
        # Draw confirmed ball (3D validated)
        if self.current_ball_position:
            ball_x, ball_y, ball_z = self.current_ball_position
            
            # Draw ball as solid green circle
            circle = plt.Circle((ball_x, ball_y), self.ball_diameter/2, 
                              color='green', alpha=0.8, label='Ball (3D Confirmed)')
            self.ax_map.add_patch(circle)
            
            # Label with 3D coordinates
            self.ax_map.text(ball_x, ball_y + 0.4, f'BALL\n({ball_x:.1f},{ball_y:.1f},{ball_z:.1f})', 
                           ha='center', va='bottom', fontweight='bold', color='darkgreen')
            
            all_x.append(ball_x)
            all_y.append(ball_y)
            title = '3D Ball Localization - Ball Confirmed'
        else:
            title = '3D Ball Localization'
            
        # Draw ball candidate (camera-only estimate)
        if self.current_ball_candidate:
            cand_x, cand_y, cand_z = self.current_ball_candidate
            
            # Draw candidate as dashed yellow circle
            circle_candidate = plt.Circle((cand_x, cand_y), self.ball_diameter/2, 
                                        color='yellow', alpha=0.5, linestyle='--', 
                                        fill=False, linewidth=2, label='Ball Candidate (Camera Only)')
            self.ax_map.add_patch(circle_candidate)
            
            # Label with estimated coordinates
            self.ax_map.text(cand_x, cand_y - 0.4, f'CANDIDATE\n({cand_x:.1f},{cand_y:.1f})', 
                           ha='center', va='top', fontsize=9, color='orange')
            
            all_x.append(cand_x)
            all_y.append(cand_y)
            if not self.current_ball_position:
                title = '3D Ball Localization - Candidate Detected'
        
        self.ax_map.set_title(title)
        
        # Set map bounds to include all objects
        if len(all_x) > 1:  # More than just robot
            margin = 2.0
            self.ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)
            self.ax_map.legend()
        else:
            # Just robot visible
            margin = 3.0
            self.ax_map.set_xlim(robot_x - margin, robot_x + margin)
            self.ax_map.set_ylim(robot_y - margin, robot_y + margin)
            
        plt.draw()
        plt.pause(0.001)
        
    def run(self):
        """Main loop"""
        print("🚀 Starting 3D Ball Localizer...")
        print("Uses camera detection + lidar point cloud for accurate 3D positioning")
        print("Press Ctrl+C to stop")
        
        # Enable lidar first
        self.enable_lidar()
        
        # Wait for data
        while not self.has_odom_data:
            print("⏳ Waiting for robot position data...")
            time.sleep(1)
            
        while not self.has_pointcloud:
            print("⏳ Waiting for point cloud data...")
            time.sleep(1)
            
        print("✅ All data sources available")
        
        try:
            while plt.get_fignums():
                # Get camera frame
                code, data = self.video_client.GetImageSample()
                
                if code == 0:
                    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # 1. Detect ball in 2D camera image
                        annotated_frame, ball_2d = self.detect_green_ball_2d(frame)
                        
                        if ball_2d:
                            # 2. Get 3D position using point cloud
                            ball_3d = self.get_3d_position_from_camera_point(
                                ball_2d['center'][0], ball_2d['center'][1]
                            )
                            
                            if ball_3d:
                                # 3. Validate ball size in 3D
                                if self.validate_ball_size_3d(ball_3d):
                                    self.update_ball_position(ball_3d)
                                    self.current_ball_candidate = None  # Clear candidate when we have confirmed ball
                                    
                                    # Update camera annotation with 3D info
                                    cv2.putText(annotated_frame, f'3D: {ball_3d["distance"]:.1f}m', 
                                               (ball_2d['center'][0]-30, ball_2d['center'][1]+ball_2d['radius']+30),
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                else:
                                    # 3D validation failed - show as candidate
                                    self.ball_detected = False
                                    candidate_pos = self.estimate_ball_candidate_position(
                                        ball_2d['pixel_x'], ball_2d['pixel_y'], ball_2d['radius']
                                    )
                                    self.update_ball_candidate(candidate_pos)
                                    cv2.putText(annotated_frame, 'Invalid 3D size', 
                                               (ball_2d['center'][0]-50, ball_2d['center'][1]+ball_2d['radius']+30),
                                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                            else:
                                # No 3D point found - show as candidate
                                self.ball_detected = False
                                candidate_pos = self.estimate_ball_candidate_position(
                                    ball_2d['pixel_x'], ball_2d['pixel_y'], ball_2d['radius']
                                )
                                self.update_ball_candidate(candidate_pos)
                                cv2.putText(annotated_frame, 'No 3D point', 
                                           (ball_2d['center'][0]-40, ball_2d['center'][1]+ball_2d['radius']+30),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        else:
                            # No ball detected in camera
                            self.ball_detected = False
                            self.current_ball_candidate = None
                        
                        self.update_visualization(annotated_frame, ball_2d)
                else:
                    print(f"⚠️  Camera error: {code}")
                    
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n⏹️  3D Ball localization stopped")
        except Exception as e:
            print(f"❌ Error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            cv2.destroyAllWindows()
            self.disable_lidar()  # Always disable lidar on exit
            if self.current_ball_position:
                x, y, z = self.current_ball_position
                print(f"🟢 Final ball position: ({x:.2f}, {y:.2f}, {z:.2f})")

def main():
    print("🚀 Go2 3D Ball Localizer")
    print("=" * 60)
    print("Advanced ball localization using camera + lidar point cloud")
    print("")
    print("Features:")
    print("  - Camera-based ball detection (RGB 90,180,120)")
    print("  - 3D positioning via lidar point cloud")
    print("  - Ball size validation in 3D space")
    print("  - Accurate world coordinates")
    print("")
    
    print("Requirements:")
    print("  - 50cm green ball (RGB 90,180,120)")
    print("  - Lidar enabled and producing point clouds")
    print("  - Camera and robot odometry working")
    print("")
    
    input("Press Enter to start 3D ball localization...")
    
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    localizer = BallLocalizer3D()
    localizer.run()

if __name__ == "__main__":
    main()