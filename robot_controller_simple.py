#!/usr/bin/env python3

import time
import sys
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class SimpleRobotController:
    def __init__(self):
        # Initialize sport client
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(3.0)
        self.sport_client.Init()
        
        # Movement parameters - moderate speeds
        self.linear_speed = 1.0   # m/s forward/backward speed 
        self.lateral_speed = 0.7  # m/s left/right speed 
        self.angular_speed = 1.5  # rad/s rotation speed
        
        # Control state
        self.is_running = True
        self.is_standing = False
        
        # Current velocities
        self.vx = 0.0  # forward/backward
        self.vy = 0.0  # left/right
        self.vyaw = 0.0  # rotation
        
        # Input handling
        self.current_command = ""
        self.last_command_time = time.time()
        self.movement_start_time = 0.0
        self.movement_duration = 0.5  # Move for 0.5 seconds per command
        
    def print_controls(self):
        """Print control instructions"""
        print("🎮 Go2 Robot Simple Controller")
        print("=" * 50)
        print("Commands (type and press ENTER):")
        print("  w       : Move Forward")
        print("  s       : Move Backward") 
        print("  a       : Move Left")
        print("  d       : Move Right")
        print("  q       : Turn Left")
        print("  e       : Turn Right")
        print("  stop    : Stop Movement")
        print("  space   : Stand Up/Down")
        print("  stand   : Stand Up")
        print("  down    : Stand Down")
        print("  recovery: Recovery Stand")
        print("  faster  : Increase Speed")
        print("  slower  : Decrease Speed")
        print("  slow    : Slow Mode (0.3 m/s)")
        print("  normal  : Normal Mode (1.5 m/s)")  
        print("  fast    : Fast Mode (2.5 m/s)")
        print("  turbo   : Turbo Mode (3.0 m/s)")
        print("  status  : Show Status")
        print("  quit    : Emergency Stop & Exit")
        print("")
        print(f"Current Speed: {self.linear_speed:.1f} m/s")
        print("=" * 50)
        print("🟡 Robot starting in DAMP mode. Type 'stand' to stand up.")
        
    def update_movement(self):
        """Send current movement command to robot"""
        if self.is_standing:
            ret = self.sport_client.Move(self.vx, self.vy, self.vyaw)
            if ret != 0:
                print(f"⚠️  Movement command failed: {ret}")
                
    def handle_command(self, command):
        """Process text command"""
        command = command.lower().strip()
        
        if not command:
            return
            
        # Movement commands
        if command == 'w':
            self.vx = self.linear_speed
            self.vy = 0.0
            self.vyaw = 0.0
            self.movement_start_time = time.time()
            print("🔼 Moving Forward")
            
        elif command == 's':
            self.vx = -self.linear_speed
            self.vy = 0.0
            self.vyaw = 0.0
            self.movement_start_time = time.time()
            print("🔽 Moving Backward")
            
        elif command == 'a':
            self.vx = 0.0
            self.vy = self.lateral_speed
            self.vyaw = 0.0
            self.movement_start_time = time.time()
            print("⬅️  Moving Left")
            
        elif command == 'd':
            self.vx = 0.0
            self.vy = -self.lateral_speed 
            self.vyaw = 0.0
            self.movement_start_time = time.time()
            print("➡️  Moving Right")
            
        elif command == 'q':
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = self.angular_speed
            self.movement_start_time = time.time()
            print("↺ Turning Left")
            
        elif command == 'e':
            self.vx = 0.0
            self.vy = 0.0
            self.vyaw = -self.angular_speed
            self.movement_start_time = time.time()
            print("↻ Turning Right")
            
        # Robot control commands
        elif command in ['space', 'stand']:
            if not self.is_standing:
                print("🚀 Standing Up...")
                ret = self.sport_client.StandUp()
                if ret == 0:
                    self.is_standing = True
                    print("✅ Robot is now standing")
                else:
                    print(f"❌ Stand up failed: {ret}")
            else:
                print("✅ Already standing")
                
        elif command == 'down':
            if self.is_standing:
                print("⬇️  Standing Down...")
                ret = self.sport_client.StandDown()
                if ret == 0:
                    self.is_standing = False
                    self.vx = self.vy = self.vyaw = 0.0
                    print("✅ Robot is now lying down")
                else:
                    print(f"❌ Stand down failed: {ret}")
            else:
                print("✅ Already lying down")
                
        elif command == 'recovery':
            print("🔄 Recovery Stand...")
            ret = self.sport_client.RecoveryStand()
            if ret == 0:
                self.is_standing = True
                print("✅ Recovery complete")
            else:
                print(f"❌ Recovery failed: {ret}")
                
        elif command == 'stop':
            self.vx = self.vy = self.vyaw = 0.0
            ret = self.sport_client.StopMove()
            print("🛑 Stopping movement")
            
        # Speed controls - allow much faster speeds!
        elif command == 'faster':
            self.linear_speed = min(3.0, self.linear_speed + 0.2)  # Up to 3 m/s
            self.lateral_speed = min(2.0, self.lateral_speed + 0.1)  # Up to 2 m/s
            self.angular_speed = min(4.0, self.angular_speed + 0.2)  # Up to 4 rad/s
            print(f"⚡ Speed increased: {self.linear_speed:.1f} m/s")
            
        elif command == 'slower':
            self.linear_speed = max(0.2, self.linear_speed - 0.2)  # Minimum 0.2 m/s
            self.lateral_speed = max(0.1, self.lateral_speed - 0.1)  # Minimum 0.1 m/s  
            self.angular_speed = max(0.5, self.angular_speed - 0.2)  # Minimum 0.5 rad/s
            print(f"🐌 Speed decreased: {self.linear_speed:.1f} m/s")
            
        # Speed presets
        elif command == 'slow':
            self.linear_speed = 0.3
            self.lateral_speed = 0.2
            self.angular_speed = 0.5
            print("🐌 Slow mode: 0.3 m/s")
            
        elif command == 'normal':
            self.linear_speed = 1.0
            self.lateral_speed = 0.7
            self.angular_speed = 1.5
            print("🚶 Normal mode: 1.0 m/s")
            
        elif command == 'fast':
            self.linear_speed = 2.5
            self.lateral_speed = 1.5
            self.angular_speed = 3.0
            print("🏃 Fast mode: 2.5 m/s")
            
        elif command == 'turbo':
            self.linear_speed = 3.0
            self.lateral_speed = 2.0
            self.angular_speed = 4.0
            print("🚀 TURBO mode: 3.0 m/s - BE CAREFUL!")
            
        # Status
        elif command == 'status':
            standing_status = "Standing" if self.is_standing else "Lying Down"
            print(f"📊 Status: {standing_status}")
            print(f"   Speed: {self.linear_speed:.1f} m/s")
            print(f"   Current velocity: vx={self.vx:.1f}, vy={self.vy:.1f}, vyaw={self.vyaw:.1f}")
            
        # Exit
        elif command in ['quit', 'exit', 'q']:
            print("🚨 Emergency Stop!")
            self.emergency_stop()
            self.is_running = False
            
        else:
            print(f"❓ Unknown command: '{command}'. Type 'status' for help.")
        
        self.last_command_time = time.time()
            
    def emergency_stop(self):
        """Emergency stop - immediately stop all movement"""
        self.vx = self.vy = self.vyaw = 0.0
        self.sport_client.StopMove()
        self.sport_client.Damp()
        print("🛑 Emergency stop activated - all movement stopped")
        
    def input_thread(self):
        """Thread to handle user input"""
        while self.is_running:
            try:
                command = input("> ").strip()
                if command:
                    self.handle_command(command)
            except (EOFError, KeyboardInterrupt):
                self.is_running = False
                break
                
    def movement_loop(self):
        """Main movement control loop"""
        while self.is_running:
            current_time = time.time()
            
            # Auto-stop movement after 0.5 seconds per command
            if self.movement_start_time > 0 and current_time - self.movement_start_time > self.movement_duration:
                if self.vx != 0 or self.vy != 0 or self.vyaw != 0:
                    self.vx = self.vy = self.vyaw = 0.0
                    self.movement_start_time = 0.0
                    if self.is_standing:
                        self.sport_client.StopMove()
                        print("⏰ Movement completed (0.5s)")
            
            # Update robot movement
            self.update_movement()
            
            # Control loop at 20Hz for better responsiveness
            time.sleep(0.05)
            
    def run(self):
        """Main controller loop"""
        try:
            self.print_controls()
            
            # Start in damp mode
            self.sport_client.Damp()
            
            # Start input thread
            input_thread = threading.Thread(target=self.input_thread, daemon=True)
            input_thread.start()
            
            # Run movement loop
            self.movement_loop()
            
        except KeyboardInterrupt:
            print("\n🛑 Controller stopped by Ctrl+C")
            self.emergency_stop()
        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.emergency_stop()
        finally:
            print("🔌 Controller disconnected")

def main():
    print("🚀 Go2 Robot Simple Controller")
    print("=" * 50)
    print("This will allow you to control your Go2 robot with text commands.")
    print("Commands are processed when you press ENTER.")
    print("")
    print("Connection Setup:")
    print("  1. Connect via Ethernet cable")
    print("  2. Set PC IP to 192.168.123.51/24") 
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Usage: python3 robot_controller_simple.py [network_interface]")
    print("")
    
    print("⚠️  WARNING: Ensure clear space around robot!")
    print("⚠️  Type 'quit' for emergency stop!")
    input("Press Enter to continue...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create and run controller
    controller = SimpleRobotController()
    controller.run()

if __name__ == "__main__":
    main()