#!/usr/bin/env python3
"""
MCP Server for Unitree Go2 Robot Control (No Ball Detection)

Provides Claude with access to:
- Camera feed and image capture
- Robot pose and position data
- Robot movement control
- Lidar point cloud data
"""

import asyncio
import json
import base64
import time
import math
import numpy as np
import cv2
import struct
from typing import Any, Dict, List, Optional
from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import mcp.types as types

# Unitree SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

class RobotMCPServer:
    def __init__(self, network_interface: str = None):
        self.server = Server("unitree-go2-robot")
        
        # Initialize Unitree SDK
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
        else:
            ChannelFactoryInitialize(0)
            
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.robot_pitch = 0.0  
        self.robot_roll = 0.0
        self.has_pose_data = False
        
        # Camera client
        self.video_client = VideoClient()
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()
        
        # Sport client for movement
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(3.0)
        self.sport_client.Init()
        self.is_standing = False
        
        # Lidar control
        self.lidar_switch_publisher = ChannelPublisher("rt/utlidar/switch", String_)
        self.lidar_switch_publisher.Init()
        
        # Point cloud data
        self.latest_pointcloud = None
        self.has_pointcloud = False
        
        # Setup subscribers
        self._setup_subscribers()
        
        # Register MCP tools
        self._register_tools()
        
    def _setup_subscribers(self):
        """Setup ROS topic subscribers"""
        try:
            self.pose_subscriber = ChannelSubscriber("rt/utlidar/robot_pose", PoseStamped_)
            self.pose_subscriber.Init(self._pose_handler, 10)
        except Exception as e:
            print(f"Failed to subscribe to robot pose: {e}")
            
        try:
            self.odom_subscriber = ChannelSubscriber("rt/lio_sam_ros2/mapping/odometry", Odometry_)
            self.odom_subscriber.Init(self._odom_handler, 10)
        except Exception as e:
            print(f"Failed to subscribe to odometry: {e}")
            
        try:
            self.pointcloud_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
            self.pointcloud_subscriber.Init(self._pointcloud_handler, 10)
        except Exception as e:
            print(f"Failed to subscribe to point cloud: {e}")
            
    def _quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
        
    def _pose_handler(self, msg: PoseStamped_):
        """Handle robot pose updates"""
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.robot_z = msg.pose.position.z
        
        self.robot_roll, self.robot_pitch, self.robot_yaw = self._quaternion_to_euler(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        )
        self.has_pose_data = True
        
    def _odom_handler(self, msg: Odometry_):
        """Handle SLAM odometry updates"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        
        self.robot_roll, self.robot_pitch, self.robot_yaw = self._quaternion_to_euler(
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )
        self.has_pose_data = True
        
    def _pointcloud_handler(self, msg: PointCloud2_):
        """Handle point cloud updates"""
        self.latest_pointcloud = msg
        self.has_pointcloud = True
        
    def _register_tools(self):
        """Register all MCP tools"""
        
        @self.server.list_tools()
        async def handle_list_tools() -> list[types.Tool]:
            return [
                # Camera tools
                types.Tool(
                    name="capture_image",
                    description="Capture an image from the robot's camera",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "save_path": {
                                "type": "string",
                                "description": "Optional path to save the image file"
                            }
                        }
                    }
                ),
                
                # Robot pose tools
                types.Tool(
                    name="get_robot_pose",
                    description="Get current robot position and orientation",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="get_robot_position_map",
                    description="Get robot position on the map with coordinate system info",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                # Movement control tools
                types.Tool(
                    name="robot_stand_up",
                    description="Make the robot stand up",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="robot_stand_down",
                    description="Make the robot lie down",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="robot_move",
                    description="Move the robot with specified velocities",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "forward_speed": {
                                "type": "number",
                                "description": "Forward/backward speed in m/s (-3.0 to 3.0)"
                            },
                            "sideways_speed": {
                                "type": "number", 
                                "description": "Left/right speed in m/s (-2.0 to 2.0)"
                            },
                            "rotation_speed": {
                                "type": "number",
                                "description": "Rotation speed in rad/s (-4.0 to 4.0)"
                            },
                            "duration": {
                                "type": "number",
                                "description": "Duration to move in seconds (default 1.0)"
                            }
                        },
                        "required": ["forward_speed", "sideways_speed", "rotation_speed"]
                    }
                ),
                
                types.Tool(
                    name="robot_stop",
                    description="Stop all robot movement",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                # Lidar tools
                types.Tool(
                    name="enable_lidar",
                    description="Enable the robot's lidar sensor",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="disable_lidar", 
                    description="Disable the robot's lidar sensor",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="get_point_cloud_data",
                    description="Get current lidar point cloud data",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "max_points": {
                                "type": "integer",
                                "description": "Maximum number of points to return (default 1000)"
                            },
                            "min_distance": {
                                "type": "number",
                                "description": "Minimum distance filter in meters (default 0.1)"
                            },
                            "max_distance": {
                                "type": "number", 
                                "description": "Maximum distance filter in meters (default 10.0)"
                            }
                        }
                    }
                ),
                
                # Utility tools
                types.Tool(
                    name="get_robot_status",
                    description="Get comprehensive robot status including all sensors",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                )
            ]
            
        @self.server.call_tool()
        async def handle_call_tool(name: str, arguments: dict) -> list[types.TextContent]:
            try:
                if name == "capture_image":
                    return await self._capture_image(arguments)
                elif name == "get_robot_pose":
                    return await self._get_robot_pose(arguments)
                elif name == "get_robot_position_map":
                    return await self._get_robot_position_map(arguments)
                elif name == "robot_stand_up":
                    return await self._robot_stand_up(arguments)
                elif name == "robot_stand_down":
                    return await self._robot_stand_down(arguments)
                elif name == "robot_move":
                    return await self._robot_move(arguments)
                elif name == "robot_stop":
                    return await self._robot_stop(arguments)
                elif name == "enable_lidar":
                    return await self._enable_lidar(arguments)
                elif name == "disable_lidar":
                    return await self._disable_lidar(arguments)
                elif name == "get_point_cloud_data":
                    return await self._get_point_cloud_data(arguments)
                elif name == "get_robot_status":
                    return await self._get_robot_status(arguments)
                else:
                    raise ValueError(f"Unknown tool: {name}")
                    
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"Error executing {name}: {str(e)}"
                )]
    
    async def _capture_image(self, args: dict) -> list[types.TextContent]:
        """Capture image from robot camera"""
        try:
            code, data = self.video_client.GetImageSample()
            
            if code != 0:
                return [types.TextContent(
                    type="text",
                    text=f"Camera error: {code}"
                )]
                
            # Convert to OpenCV image
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            
            if frame is None:
                return [types.TextContent(
                    type="text", 
                    text="Failed to decode camera image"
                )]
            
            # Convert to base64 for transmission
            _, buffer = cv2.imencode('.jpg', frame)
            image_b64 = base64.b64encode(buffer).decode('utf-8')
            
            # Save if path provided
            save_path = args.get('save_path')
            if save_path:
                cv2.imwrite(save_path, frame)
                
            return [types.TextContent(
                type="text",
                text=f"Image captured successfully. Resolution: {frame.shape[1]}x{frame.shape[0]}\n"
                     f"Base64 data: data:image/jpeg;base64,{image_b64}"
                     + (f"\nSaved to: {save_path}" if save_path else "")
            )]
            
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Camera capture failed: {str(e)}"
            )]
    
    async def _get_robot_pose(self, args: dict) -> list[types.TextContent]:
        """Get current robot pose"""
        if not self.has_pose_data:
            return [types.TextContent(
                type="text",
                text="No robot pose data available. Check robot connection and odometry."
            )]
            
        return [types.TextContent(
            type="text",
            text=f"""Robot Pose:
Position: x={self.robot_x:.3f}m, y={self.robot_y:.3f}m, z={self.robot_z:.3f}m
Orientation: roll={math.degrees(self.robot_roll):.1f}°, pitch={math.degrees(self.robot_pitch):.1f}°, yaw={math.degrees(self.robot_yaw):.1f}°
Quaternion: Available in raw data if needed"""
        )]
    
    async def _get_robot_position_map(self, args: dict) -> list[types.TextContent]:
        """Get robot position on map with coordinate system info"""
        if not self.has_pose_data:
            return [types.TextContent(
                type="text",
                text="No robot position data available."
            )]
            
        return [types.TextContent(
            type="text", 
            text=f"""Robot Map Position:
Coordinates: ({self.robot_x:.3f}, {self.robot_y:.3f})
Height: {self.robot_z:.3f}m
Heading: {math.degrees(self.robot_yaw):.1f}° (0° = East, 90° = North)
Coordinate System: SLAM world frame (meters)
Data Source: {'SLAM odometry + Lidar localization' if self.has_pose_data else 'Unknown'}"""
        )]
    
    async def _robot_stand_up(self, args: dict) -> list[types.TextContent]:
        """Make robot stand up"""
        try:
            ret = self.sport_client.StandUp()
            if ret == 0:
                self.is_standing = True
                await asyncio.sleep(3)  # Give time to stand
                return [types.TextContent(
                    type="text",
                    text="Robot successfully stood up and is ready for movement."
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"Failed to stand up robot. Error code: {ret}"
                )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Stand up failed: {str(e)}"
            )]
    
    async def _robot_stand_down(self, args: dict) -> list[types.TextContent]:
        """Make robot lie down"""
        try:
            ret = self.sport_client.StandDown()
            if ret == 0:
                self.is_standing = False
                return [types.TextContent(
                    type="text", 
                    text="Robot successfully laid down."
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"Failed to lay down robot. Error code: {ret}"
                )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Stand down failed: {str(e)}"
            )]
    
    async def _robot_move(self, args: dict) -> list[types.TextContent]:
        """Move robot with specified velocities"""
        if not self.is_standing:
            return [types.TextContent(
                type="text",
                text="Robot must be standing before it can move. Use robot_stand_up first."
            )]
        
        try:
            forward_speed = args.get('forward_speed', 0.0)
            sideways_speed = args.get('sideways_speed', 0.0)  
            rotation_speed = args.get('rotation_speed', 0.0)
            duration = args.get('duration', 1.0)
            
            # Safety limits
            forward_speed = max(-3.0, min(3.0, forward_speed))
            sideways_speed = max(-2.0, min(2.0, sideways_speed))
            rotation_speed = max(-4.0, min(4.0, rotation_speed))
            duration = max(0.1, min(10.0, duration))
            
            # Start movement
            ret = self.sport_client.Move(forward_speed, sideways_speed, rotation_speed)
            if ret != 0:
                return [types.TextContent(
                    type="text",
                    text=f"Movement command failed. Error code: {ret}"
                )]
            
            # Move for specified duration
            await asyncio.sleep(duration)
            
            # Stop movement
            self.sport_client.StopMove()
            
            return [types.TextContent(
                type="text",
                text=f"Robot moved for {duration}s at speeds: forward={forward_speed:.2f}m/s, "
                     f"sideways={sideways_speed:.2f}m/s, rotation={rotation_speed:.2f}rad/s"
            )]
            
        except Exception as e:
            # Emergency stop on any error
            self.sport_client.StopMove()
            return [types.TextContent(
                type="text",
                text=f"Movement failed: {str(e)}. Robot stopped for safety."
            )]
    
    async def _robot_stop(self, args: dict) -> list[types.TextContent]:
        """Stop all robot movement"""
        try:
            ret = self.sport_client.StopMove()
            return [types.TextContent(
                type="text",
                text="Robot movement stopped."
            )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Stop command failed: {str(e)}"
            )]
    
    async def _enable_lidar(self, args: dict) -> list[types.TextContent]:
        """Enable lidar sensor"""
        try:
            switch_msg = std_msgs_msg_dds__String_()
            switch_msg.data = "ON"
            self.lidar_switch_publisher.Write(switch_msg)
            
            await asyncio.sleep(2)  # Give lidar time to start
            
            return [types.TextContent(
                type="text",
                text="Lidar sensor enabled. Point cloud data should be available shortly."
            )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Failed to enable lidar: {str(e)}"
            )]
    
    async def _disable_lidar(self, args: dict) -> list[types.TextContent]:
        """Disable lidar sensor"""
        try:
            switch_msg = std_msgs_msg_dds__String_()
            switch_msg.data = "OFF"
            self.lidar_switch_publisher.Write(switch_msg)
            
            return [types.TextContent(
                type="text",
                text="Lidar sensor disabled."
            )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Failed to disable lidar: {str(e)}"
            )]
    
    async def _get_point_cloud_data(self, args: dict) -> list[types.TextContent]:
        """Get lidar point cloud data"""
        if not self.has_pointcloud or self.latest_pointcloud is None:
            return [types.TextContent(
                type="text",
                text="No point cloud data available. Enable lidar first with enable_lidar."
            )]
        
        try:
            max_points = args.get('max_points', 1000)
            min_distance = args.get('min_distance', 0.1)
            max_distance = args.get('max_distance', 10.0)
            
            # Parse point cloud
            points = []
            msg = self.latest_pointcloud
            point_step = msg.point_step
            data = bytes(msg.data)
            
            count = 0
            for i in range(0, min(msg.width, max_points * 2)):  # Sample more than needed
                if count >= max_points:
                    break
                    
                offset = i * point_step
                if offset + point_step <= len(data):
                    try:
                        point_data = struct.unpack_from('<fff4xf H 2x f', data, offset)
                        x, y, z, intensity, ring, timestamp = point_data
                        
                        distance = math.sqrt(x**2 + y**2 + z**2)
                        if min_distance <= distance <= max_distance and not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                            points.append({
                                'x': round(x, 3),
                                'y': round(y, 3), 
                                'z': round(z, 3),
                                'intensity': round(intensity, 1),
                                'distance': round(distance, 3)
                            })
                            count += 1
                    except:
                        continue
            
            return [types.TextContent(
                type="text",
                text=f"""Point Cloud Data:
Total points: {len(points)}/{msg.width}
Distance range: {min_distance}m to {max_distance}m
Sample points (first 10):
""" + "\n".join([f"  Point {i+1}: x={p['x']}, y={p['y']}, z={p['z']}, dist={p['distance']}m, intensity={p['intensity']}" 
                 for i, p in enumerate(points[:10])]) + 
f"\n\nFull data available as JSON: {json.dumps(points[:100])}"  # Return first 100 points as JSON
            )]
            
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Failed to parse point cloud: {str(e)}"
            )]
    
    async def _get_robot_status(self, args: dict) -> list[types.TextContent]:
        """Get comprehensive robot status"""
        status = "=== Unitree Go2 Robot Status ===\n\n"
        
        # Pose data
        if self.has_pose_data:
            status += f"""Position & Orientation:
  Location: ({self.robot_x:.3f}, {self.robot_y:.3f}, {self.robot_z:.3f})
  Orientation: Yaw={math.degrees(self.robot_yaw):.1f}°, Pitch={math.degrees(self.robot_pitch):.1f}°, Roll={math.degrees(self.robot_roll):.1f}°
  Status: ✅ Odometry Active
"""
        else:
            status += "Position & Orientation: ❌ No data available\n"
        
        # Camera status
        try:
            code, _ = self.video_client.GetImageSample()
            if code == 0:
                status += "Camera: ✅ Active and streaming\n"
            else:
                status += f"Camera: ⚠️  Error code {code}\n"
        except:
            status += "Camera: ❌ Connection failed\n"
        
        # Point cloud status
        if self.has_pointcloud:
            msg = self.latest_pointcloud
            status += f"Lidar: ✅ Active ({msg.width} points, {msg.height}x{msg.width} resolution)\n"
        else:
            status += "Lidar: ❌ No point cloud data\n"
        
        # Movement status
        status += f"Movement: {'✅ Standing (can move)' if self.is_standing else '⚠️  Lying down (use robot_stand_up)'}\n"
        
        return [types.TextContent(
            type="text",
            text=status
        )]

    async def run(self):
        """Run the MCP server"""
        async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
            await self.server.run(
                read_stream,
                write_stream,
                InitializationOptions(
                    server_name="unitree-go2-robot",
                    server_version="1.0.0",
                    capabilities=self.server.get_capabilities(
                        notification_options=NotificationOptions(),
                        experimental_capabilities={}
                    ),
                ),
            )

async def main():
    import sys
    network_interface = sys.argv[1] if len(sys.argv) > 1 else None
    
    print(f"Starting Unitree Go2 MCP Server (No Ball Detection)...")
    if network_interface:
        print(f"Using network interface: {network_interface}")
    
    server = RobotMCPServer(network_interface)
    await server.run()

if __name__ == "__main__":
    asyncio.run(main())