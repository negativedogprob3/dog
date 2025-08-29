#!/usr/bin/env python3
"""
Discrete Movement MCP Server for Unitree Go2 Robot Control

Provides Claude with discrete movement commands like robot_controller_simple.py:
- Camera feed and image capture
- Robot pose and position data
- Discrete movement commands (w/a/s/d/q/e style)
"""

import asyncio
import json
import base64
import time
import math
import numpy as np
import cv2
from typing import Any, Dict, List, Optional
from mcp.server import Server, NotificationOptions
from mcp.server.models import InitializationOptions
import mcp.server.stdio
import mcp.types as types

# Unitree SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_

class DiscreteRobotMCPServer:
    def __init__(self, network_interface: str = None):
        self.server = Server("unitree-go2-robot-discrete")
        
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
        
        # Movement parameters matching robot_controller_simple.py EXACTLY
        self.linear_speed = 1.0   # m/s forward/backward speed 
        self.lateral_speed = 0.7  # m/s left/right speed 
        self.angular_speed = 1.5  # rad/s rotation speed (matching robot_controller_simple.py)
        self.movement_duration = 0.5  # seconds per movement command
        self.turn_duration = 0.5  # same duration for all movements
        
        # Setup subscribers
        self._setup_subscribers()
        
        # Register MCP tools
        self._register_tools()
        
    def _setup_subscribers(self):
        """Setup ROS topic subscribers"""
        try:
            self.pose_subscriber = ChannelSubscriber("rt/utlidar/robot_pose", PoseStamped_)
            self.pose_subscriber.Init(self._pose_handler, 10)
            print("✅ Subscribed to robot pose")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to robot_pose: {e}")
            
        try:
            self.odom_subscriber = ChannelSubscriber("rt/lio_sam_ros2/mapping/odometry", Odometry_)
            self.odom_subscriber.Init(self._odom_handler, 10)
            print("✅ Subscribed to SLAM odometry")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to SLAM odometry: {e}")
            
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
                
                # Robot stance control
                types.Tool(
                    name="robot_stand",
                    description="Make the robot stand up",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="robot_down",
                    description="Make the robot lie down",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="robot_recovery",
                    description="Recovery stand (if robot has fallen)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                # Discrete movement commands (like robot_controller_simple.py)
                types.Tool(
                    name="move_forward",
                    description="Move forward for 0.5 seconds (like 'w' command)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="move_backward",
                    description="Move backward for 0.5 seconds (like 's' command)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="move_left",
                    description="Move left for 0.5 seconds (like 'a' command)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="move_right",
                    description="Move right for 0.5 seconds (like 'd' command)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="turn_left",
                    description="Turn left for 0.5 seconds (like 'q' command)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="turn_right",
                    description="Turn right for 0.5 seconds (like 'e' command)",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                types.Tool(
                    name="robot_stop",
                    description="Stop all robot movement immediately",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                ),
                
                # Speed control
                types.Tool(
                    name="set_speed_mode",
                    description="Set robot movement speed mode",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string",
                                "enum": ["slow", "normal", "fast", "turbo"],
                                "description": "Speed mode: slow(0.3), normal(1.0), fast(2.5), turbo(3.0) m/s"
                            }
                        },
                        "required": ["mode"]
                    }
                ),
                
                # Utility tools
                types.Tool(
                    name="get_robot_status",
                    description="Get comprehensive robot status",
                    inputSchema={
                        "type": "object",
                        "properties": {}
                    }
                )
            ]
            
        @self.server.call_tool()
        async def handle_call_tool(name: str, arguments: dict) -> list[types.TextContent | types.ImageContent]:
            try:
                if name == "capture_image":
                    return await self._capture_image(arguments)
                elif name == "get_robot_pose":
                    return await self._get_robot_pose(arguments)
                elif name == "get_robot_position_map":
                    return await self._get_robot_position_map(arguments)
                elif name == "robot_stand":
                    return await self._robot_stand(arguments)
                elif name == "robot_down":
                    return await self._robot_down(arguments)
                elif name == "robot_recovery":
                    return await self._robot_recovery(arguments)
                elif name == "move_forward":
                    return await self._move_forward(arguments)
                elif name == "move_backward":
                    return await self._move_backward(arguments)
                elif name == "move_left":
                    return await self._move_left(arguments)
                elif name == "move_right":
                    return await self._move_right(arguments)
                elif name == "turn_left":
                    return await self._turn_left(arguments)
                elif name == "turn_right":
                    return await self._turn_right(arguments)
                elif name == "robot_stop":
                    return await self._robot_stop(arguments)
                elif name == "set_speed_mode":
                    return await self._set_speed_mode(arguments)
                elif name == "get_robot_status":
                    return await self._get_robot_status(arguments)
                else:
                    raise ValueError(f"Unknown tool: {name}")
                    
            except Exception as e:
                return [types.TextContent(
                    type="text",
                    text=f"Error executing {name}: {str(e)}"
                )]
    
    async def _capture_image(self, args: dict) -> list[types.TextContent | types.ImageContent]:
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
                
            # Return both text description and the actual image
            results = [
                types.TextContent(
                    type="text",
                    text=f"Image captured from Go2 robot camera. Resolution: {frame.shape[1]}x{frame.shape[0]}"
                         + (f"\nSaved to: {save_path}" if save_path else "")
                ),
                types.ImageContent(
                    type="image",
                    data=image_b64,
                    mimeType="image/jpeg"
                )
            ]
            
            return results
            
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
Orientation: roll={math.degrees(self.robot_roll):.1f}°, pitch={math.degrees(self.robot_pitch):.1f}°, yaw={math.degrees(self.robot_yaw):.1f}°"""
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
Coordinate System: SLAM world frame (meters)"""
        )]
    
    async def _robot_stand(self, args: dict) -> list[types.TextContent]:
        """Make robot stand up"""
        try:
            if self.is_standing:
                return [types.TextContent(
                    type="text",
                    text="Robot is already standing."
                )]
                
            ret = self.sport_client.StandUp()
            if ret == 0:
                self.is_standing = True
                time.sleep(3)  # Give time to stand
                return [types.TextContent(
                    type="text",
                    text="🚀 Robot successfully stood up and is ready for movement."
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to stand up robot. Error code: {ret}"
                )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Stand up failed: {str(e)}"
            )]
    
    async def _robot_down(self, args: dict) -> list[types.TextContent]:
        """Make robot lie down"""
        try:
            if not self.is_standing:
                return [types.TextContent(
                    type="text",
                    text="Robot is already lying down."
                )]
                
            ret = self.sport_client.StandDown()
            if ret == 0:
                self.is_standing = False
                return [types.TextContent(
                    type="text", 
                    text="⬇️  Robot successfully laid down."
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Failed to lay down robot. Error code: {ret}"
                )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Stand down failed: {str(e)}"
            )]
    
    async def _robot_recovery(self, args: dict) -> list[types.TextContent]:
        """Recovery stand"""
        try:
            ret = self.sport_client.RecoveryStand()
            if ret == 0:
                self.is_standing = True
                return [types.TextContent(
                    type="text",
                    text="🔄 Recovery stand completed successfully."
                )]
            else:
                return [types.TextContent(
                    type="text",
                    text=f"❌ Recovery stand failed. Error code: {ret}"
                )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Recovery failed: {str(e)}"
            )]
    
    async def _discrete_move(self, vx: float, vy: float, vyaw: float, description: str) -> list[types.TextContent]:
        """Execute a discrete movement command - sends commands repeatedly like simple controller"""
        if not self.is_standing:
            return [types.TextContent(
                type="text",
                text="Robot must be standing to move. Use robot_stand first."
            )]
        
        try:
            print(f"DEBUG: Executing movement - vx={vx}, vy={vy}, vyaw={vyaw}")
            print(f"DEBUG: Robot standing status: {self.is_standing}")
            
            # Send movement commands repeatedly at 20Hz like simple controller
            start_time = time.time()
            command_count = 0
            
            while time.time() - start_time < self.movement_duration:
                ret = self.sport_client.Move(vx, vy, vyaw)
                command_count += 1
                
                if ret != 0:
                    print(f"DEBUG: Move command failed at attempt {command_count}: {ret}")
                    # Don't return immediately, try to continue
                
                time.sleep(0.05)  # 20Hz update rate like simple controller
            
            # Stop movement
            stop_ret = self.sport_client.StopMove()
            print(f"DEBUG: Sent {command_count} movement commands, stop result: {stop_ret}")
            
            # Determine speed for display
            if 'forward' in description.lower() or 'backward' in description.lower():
                speed_value = self.linear_speed
                speed_unit = "m/s"
            elif 'left' in description.lower() or 'right' in description.lower():
                speed_value = self.lateral_speed  
                speed_unit = "m/s"
            else:  # turning
                speed_value = abs(vyaw)
                speed_unit = "rad/s"
            
            return [types.TextContent(
                type="text",
                text=f"{description} (sent {command_count} commands over {self.movement_duration}s at {speed_value:.1f} {speed_unit})"
            )]
            
        except Exception as e:
            # Emergency stop on any error
            print(f"DEBUG: Exception during movement: {e}")
            self.sport_client.StopMove()
            return [types.TextContent(
                type="text",
                text=f"Movement failed: {str(e)}. Robot stopped for safety."
            )]
    
    async def _move_forward(self, args: dict) -> list[types.TextContent]:
        """Move forward (like 'w' key)"""
        return await self._discrete_move(self.linear_speed, 0.0, 0.0, "🔼 Moved forward")
    
    async def _move_backward(self, args: dict) -> list[types.TextContent]:
        """Move backward (like 's' key)"""
        return await self._discrete_move(-self.linear_speed, 0.0, 0.0, "🔽 Moved backward")
    
    async def _move_left(self, args: dict) -> list[types.TextContent]:
        """Move left (like 'a' key)"""
        return await self._discrete_move(0.0, self.lateral_speed, 0.0, "⬅️  Moved left")
    
    async def _move_right(self, args: dict) -> list[types.TextContent]:
        """Move right (like 'd' key)"""
        return await self._discrete_move(0.0, -self.lateral_speed, 0.0, "➡️  Moved right")
    
    async def _turn_left(self, args: dict) -> list[types.TextContent]:
        """Turn left (like 'q' key)"""
        print(f"DEBUG: Turn left command - angular_speed={self.angular_speed}")
        return await self._discrete_turn(self.angular_speed, "↺ Turned left")
    
    async def _turn_right(self, args: dict) -> list[types.TextContent]:
        """Turn right (like 'e' key)"""
        print(f"DEBUG: Turn right command - angular_speed={-self.angular_speed}")
        return await self._discrete_turn(-self.angular_speed, "↻ Turned right")
    
    async def _discrete_turn(self, vyaw: float, description: str) -> list[types.TextContent]:
        """Execute a discrete turning command - same as _discrete_move but for clarity"""
        # Use the same repeated command approach as other movements
        return await self._discrete_move(0.0, 0.0, vyaw, description)
    
    async def _robot_stop(self, args: dict) -> list[types.TextContent]:
        """Stop all robot movement"""
        try:
            ret = self.sport_client.StopMove()
            return [types.TextContent(
                type="text",
                text="🛑 Robot movement stopped."
            )]
        except Exception as e:
            return [types.TextContent(
                type="text",
                text=f"Stop command failed: {str(e)}"
            )]
    
    async def _set_speed_mode(self, args: dict) -> list[types.TextContent]:
        """Set movement speed mode"""
        mode = args.get('mode', 'normal')
        
        if mode == 'slow':
            self.linear_speed = 0.3
            self.lateral_speed = 0.2
            self.angular_speed = 0.5
            return [types.TextContent(
                type="text",
                text="🐌 Speed set to SLOW mode: 0.3 m/s"
            )]
        elif mode == 'normal':
            self.linear_speed = 1.0
            self.lateral_speed = 0.7
            self.angular_speed = 1.5
            return [types.TextContent(
                type="text",
                text="🚶 Speed set to NORMAL mode: 1.0 m/s"
            )]
        elif mode == 'fast':
            self.linear_speed = 2.5
            self.lateral_speed = 1.5
            self.angular_speed = 3.0
            return [types.TextContent(
                type="text",
                text="🏃 Speed set to FAST mode: 2.5 m/s"
            )]
        elif mode == 'turbo':
            self.linear_speed = 3.0
            self.lateral_speed = 2.0
            self.angular_speed = 4.0
            return [types.TextContent(
                type="text",
                text="🚀 Speed set to TURBO mode: 3.0 m/s - BE CAREFUL!"
            )]
        else:
            return [types.TextContent(
                type="text",
                text="Invalid speed mode. Use: slow, normal, fast, or turbo"
            )]
    
    async def _get_robot_status(self, args: dict) -> list[types.TextContent]:
        """Get comprehensive robot status"""
        status = "=== Unitree Go2 Robot Status (Discrete Movement) ===\n\n"
        
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
        
        # Movement status
        status += f"Movement: {'✅ Standing (can move)' if self.is_standing else '⚠️  Lying down (use robot_stand)'}\n"
        status += f"Speed Settings: Linear={self.linear_speed:.1f}m/s, Lateral={self.lateral_speed:.1f}m/s, Angular={self.angular_speed:.1f}rad/s\n"
        status += f"Movement Duration: {self.movement_duration}s per command\n"
        
        status += "\nAvailable Movement Commands: move_forward, move_backward, move_left, move_right, turn_left, turn_right\n"
        
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
                    server_name="unitree-go2-robot-discrete",
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
    
    print(f"Starting Discrete Movement Unitree Go2 MCP Server...")
    print("Features: Camera, Position, Discrete Movement (like robot_controller_simple.py)")
    if network_interface:
        print(f"Using network interface: {network_interface}")
    
    server = DiscreteRobotMCPServer(network_interface)
    await server.run()

if __name__ == "__main__":
    asyncio.run(main())