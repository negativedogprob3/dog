#!/usr/bin/env python3

import time
import sys
import threading
import signal
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# Try to import keyboard library, fallback if not available
try:
    import keyboard
    KEYBOARD_AVAILABLE = True
    print("Using 'keyboard' library for real-time input")
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("keyboard library not found. Install with: python3 -m pip install keyboard --break-system-packages")

class RealTimeRobotController:
    def __init__(self):
        # Initialize sport client
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(3.0)
        self.sport_client.Init()
        
        # Movement parameters - start with slow speeds for safety
        self.linear_speed = 0.5   # m/s forward/backward speed (slow default)
        self.lateral_speed = 0.3  # m/s left/right speed (slow default)
        self.angular_speed = 2.0  # rad/s rotation speed (faster for better turning)
        
        # Control state
        self.is_running = True
        self.is_standing = False
        self.emergency_stop_triggered = False
        
        # Current velocities
        self.vx = 0.0  # forward/backward
        self.vy = 0.0  # left/right
        self.vyaw = 0.0  # rotation
        
        # Key state tracking
        self.keys_pressed = set()
        self.last_key_time = time.time()
        
    def print_controls(self):
        """Print control instructions"""
        print("🎮 Go2 Robot Real-Time Controller")
        print("=" * 50)
        print("Movement Controls (hold down keys):")
        print("  W       : Move Forward")
        print("  S       : Move Backward") 
        print("  A       : Move Left")
        print("  D       : Move Right")
        print("  Q       : Turn Left")
        print("  E       : Turn Right")
        print("")
        print("Robot Controls:")
        print("  SPACE   : Stand Up/Down Toggle")
        print("  R       : Recovery Stand")
        print("  H       : Stop/Hold Position")
        print("  ESC     : Emergency Stop & Exit")
        print("")
        print("Speed Controls:")
        print("  1       : Slow Mode (0.5 m/s)")
        print("  2       : Normal Mode (1.5 m/s)")
        print("  3       : Fast Mode (2.5 m/s)")
        print("  4       : Turbo Mode (3.0 m/s)")
        print("")
        print(f"Current Speed: {self.linear_speed:.1f} m/s")
        print("=" * 50)
        
    def set_speed_mode(self, mode):
        """Set movement speed based on mode"""
        if mode == 1:  # Slow
            self.linear_speed = 0.5
            self.lateral_speed = 0.3
            self.angular_speed = 1.0
            print("🐌 Slow mode: 0.5 m/s")
        elif mode == 2:  # Normal
            self.linear_speed = 1.5
            self.lateral_speed = 1.0
            self.angular_speed = 2.0
            print("🚶 Normal mode: 1.5 m/s")
        elif mode == 3:  # Fast
            self.linear_speed = 2.5
            self.lateral_speed = 1.5
            self.angular_speed = 3.0
            print("🏃 Fast mode: 2.5 m/s")
        elif mode == 4:  # Turbo
            self.linear_speed = 3.0
            self.lateral_speed = 2.0
            self.angular_speed = 4.0
            print("🚀 TURBO mode: 3.0 m/s - BE CAREFUL!")
        
    def update_movement_from_keys(self):
        """Update movement based on currently pressed keys"""
        # Reset velocities
        self.vx = self.vy = self.vyaw = 0.0
        
        # Check movement keys
        if 'w' in self.keys_pressed:
            self.vx = self.linear_speed
        if 's' in self.keys_pressed:
            self.vx = -self.linear_speed
        if 'a' in self.keys_pressed:
            self.vy = self.lateral_speed
        if 'd' in self.keys_pressed:
            self.vy = -self.lateral_speed
        if 'q' in self.keys_pressed:
            self.vyaw = self.angular_speed
        if 'e' in self.keys_pressed:
            self.vyaw = -self.angular_speed
            
        # Send movement command
        if self.is_standing and not self.emergency_stop_triggered:
            ret = self.sport_client.Move(self.vx, self.vy, self.vyaw)
            if ret != 0:
                print(f"⚠️  Movement command failed: {ret}")
                
    def handle_key_press(self, key):
        """Handle key press events"""
        key = key.lower()
        self.keys_pressed.add(key)
        self.last_key_time = time.time()
        
        # Handle non-movement keys
        if key == 'space':
            self.toggle_stand()
        elif key == 'r':
            self.recovery_stand()
        elif key == 'h':
            self.stop_movement()
        elif key == 'esc':
            self.emergency_stop()
        elif key in ['1', '2', '3', '4']:
            self.set_speed_mode(int(key))
            
    def handle_key_release(self, key):
        """Handle key release events"""
        key = key.lower()
        self.keys_pressed.discard(key)
        
    def toggle_stand(self):
        """Toggle between standing and lying down"""
        if not self.is_standing:
            print("🚀 Standing Up...")
            ret = self.sport_client.StandUp()
            if ret == 0:
                self.is_standing = True
                print("✅ Robot is now standing")
            else:
                print(f"❌ Stand up failed: {ret}")
        else:
            print("⬇️  Standing Down...")
            ret = self.sport_client.StandDown()
            if ret == 0:
                self.is_standing = False
                self.keys_pressed.clear()  # Clear movement keys
                print("✅ Robot is now lying down")
            else:
                print(f"❌ Stand down failed: {ret}")
                
    def recovery_stand(self):
        """Recovery stand"""
        print("🔄 Recovery Stand...")
        ret = self.sport_client.RecoveryStand()
        if ret == 0:
            self.is_standing = True
            print("✅ Recovery complete")
        else:
            print(f"❌ Recovery failed: {ret}")
            
    def stop_movement(self):
        """Stop all movement"""
        self.keys_pressed.clear()
        self.vx = self.vy = self.vyaw = 0.0
        ret = self.sport_client.StopMove()
        print("🛑 Stopping movement")
        
    def emergency_stop(self):
        """Emergency stop - immediately stop all movement"""
        self.emergency_stop_triggered = True
        self.keys_pressed.clear()
        self.vx = self.vy = self.vyaw = 0.0
        self.sport_client.StopMove()
        self.sport_client.Damp()
        self.is_running = False
        print("🚨 EMERGENCY STOP ACTIVATED!")
        
    def keyboard_listener(self):
        """Keyboard event listener using keyboard library"""
        if not KEYBOARD_AVAILABLE:
            return
            
        # Set up key event handlers
        keyboard.on_press(self.handle_key_press)
        keyboard.on_release(self.handle_key_release)
        
        print("🎮 Keyboard listener active. Hold down keys to move!")
        print("⚠️  Press ESC to emergency stop")
        
        # Keep listener running
        while self.is_running:
            time.sleep(0.1)
            
    def fallback_input_method(self):
        """Fallback method when keyboard library isn't available"""
        print("📝 Keyboard library not available. Using fallback method:")
        print("   Hold ENTER and type movement keys, then release ENTER")
        print("   Example: Hold ENTER, type 'w', release ENTER to move forward")
        print("")
        
        while self.is_running:
            try:
                # Get input
                user_input = input("Keys (w/a/s/d/q/e) or command: ").strip().lower()
                
                if not user_input:
                    self.keys_pressed.clear()
                    continue
                    
                # Handle commands
                if user_input == 'quit' or user_input == 'exit':
                    self.emergency_stop()
                    break
                elif user_input == 'stand':
                    self.toggle_stand()
                elif user_input == 'recovery':
                    self.recovery_stand()
                elif user_input == 'stop':
                    self.stop_movement()
                elif user_input in ['1', '2', '3', '4']:
                    self.set_speed_mode(int(user_input))
                else:
                    # Treat as movement keys
                    self.keys_pressed.clear()
                    for char in user_input:
                        if char in 'wasdqe':
                            self.keys_pressed.add(char)
                    
                    # Move for a short duration
                    start_time = time.time()
                    while time.time() - start_time < 0.5 and self.is_running:  # Move for 0.5 seconds
                        self.update_movement_from_keys()
                        time.sleep(0.02)
                    
                    # Stop movement
                    self.keys_pressed.clear()
                    self.vx = self.vy = self.vyaw = 0.0
                    if self.is_standing:
                        self.sport_client.StopMove()
                        
            except (EOFError, KeyboardInterrupt):
                self.emergency_stop()
                break
                
    def movement_loop(self):
        """Main movement control loop"""
        while self.is_running:
            current_time = time.time()
            
            if KEYBOARD_AVAILABLE:
                # Update movement based on currently pressed keys
                self.update_movement_from_keys()
            
            # Control loop at 50Hz
            time.sleep(0.02)
            
    def run(self):
        """Main controller loop"""
        try:
            self.print_controls()
            
            print("⚠️  Robot will start in DAMP mode. Press SPACE to stand up.")
            
            # Start in damp mode
            self.sport_client.Damp()
            
            # Set up signal handler for Ctrl+C
            signal.signal(signal.SIGINT, self.signal_handler)
            
            if KEYBOARD_AVAILABLE:
                # Start keyboard listener thread
                keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
                keyboard_thread.start()
                
                # Run movement loop
                self.movement_loop()
            else:
                # Use fallback input method
                self.fallback_input_method()
                
        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.emergency_stop()
        finally:
            print("🔌 Controller disconnected")
            
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C signal"""
        print("\n🛑 Controller stopped by Ctrl+C")
        self.emergency_stop()

def main():
    print("🚀 Go2 Robot Real-Time Controller")
    print("=" * 50)
    print("This will allow you to control your Go2 robot by holding down keys.")
    print("")
    
    if not KEYBOARD_AVAILABLE:
        print("⚠️  Installing 'keyboard' library is recommended for best experience:")
        print("   python3 -m pip install keyboard --break-system-packages")
        print("   (You may need to run as sudo on some systems)")
        print("")
    
    print("Connection Setup:")
    print("  1. Connect via Ethernet cable")
    print("  2. Set PC IP to 192.168.123.51/24") 
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Usage: python3 robot_controller.py [network_interface]")
    print("")
    
    print("⚠️  WARNING: Ensure clear space around robot!")
    print("⚠️  Keep ESC key ready for emergency stop!")
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
    controller = RealTimeRobotController()
    controller.run()

if __name__ == "__main__":
    main()