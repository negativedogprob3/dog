#!/usr/bin/env python3

import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_

class RobotDebugger:
    def __init__(self):
        # Initialize sport client
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(3.0)
        self.sport_client.Init()
        
        # Subscribe to sport mode state to see what's happening
        self.sport_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sport_subscriber.Init(self.sport_state_handler, 10)
        
        self.latest_state = None
        self.state_count = 0
        
    def sport_state_handler(self, msg: SportModeState_):
        """Handle sport mode state messages"""
        self.latest_state = msg
        self.state_count += 1
        
    def print_state(self):
        """Print current robot state"""
        if self.latest_state is None:
            print("❌ No sport state data received")
            return
            
        state = self.latest_state
        print(f"\n📊 Robot State (Message #{self.state_count}):")
        print(f"   Mode: {state.mode} (0=idle, 1=trot, 2=climb, etc.)")
        print(f"   Error Code: {state.error_code}")
        print(f"   Progress: {state.progress:.2f}")
        print(f"   Gait Type: {state.gait_type}")
        print(f"   Position: x={state.position[0]:.3f}, y={state.position[1]:.3f}, z={state.position[2]:.3f}")
        print(f"   Velocity: vx={state.velocity[0]:.3f}, vy={state.velocity[1]:.3f}, vz={state.velocity[2]:.3f}")
        print(f"   Yaw Speed: {state.yaw_speed:.3f}")
        print(f"   Body Height: {state.body_height:.3f}")
        print(f"   Foot Raise Height: {state.foot_raise_height:.3f}")
        
        # Check if robot is in the right state for movement
        if state.mode == 0:
            print("⚠️  Robot is in IDLE mode - may not respond to movement commands")
        elif state.error_code != 0:
            print(f"⚠️  Robot has error code: {state.error_code}")
            
    def test_movements(self):
        """Test different movement commands and see responses"""
        print("🚀 Testing robot movements...")
        
        # First, make sure robot is standing
        print("1. Standing up...")
        ret = self.sport_client.StandUp()
        print(f"   StandUp result: {ret}")
        time.sleep(3)
        self.print_state()
        
        # Test different movement modes
        print("\n2. Testing BalanceStand...")
        ret = self.sport_client.BalanceStand()
        print(f"   BalanceStand result: {ret}")
        time.sleep(2)
        self.print_state()
        
        # Try switching to different movement modes
        print("\n3. Trying to enable different movement modes...")
        
        # Try SwitchJoystick (might enable movement)
        try:
            print("   Trying SwitchJoystick...")
            ret = self.sport_client.SwitchJoystick(True)
            print(f"   SwitchJoystick(True) result: {ret}")
            time.sleep(1)
        except Exception as e:
            print(f"   SwitchJoystick failed: {e}")
            
        self.print_state()
        
        # Now try movement commands
        print("\n4. Testing Move commands...")
        
        movements = [
            ("Forward 1.5m/s", 1.5, 0.0, 0.0),
            ("Backward 1.5m/s", -1.5, 0.0, 0.0),
            ("Left 1.0m/s", 0.0, 1.0, 0.0),
            ("Right 1.0m/s", 0.0, -1.0, 0.0),
            ("Turn left fast", 0.0, 0.0, 2.0),
            ("Turn right fast", 0.0, 0.0, -2.0),
            ("Forward FAST 2.5m/s", 2.5, 0.0, 0.0),
            ("Diagonal move", 1.5, 1.0, 0.0),
        ]
        
        for desc, vx, vy, vyaw in movements:
            print(f"   Testing {desc}...")
            ret = self.sport_client.Move(vx, vy, vyaw)
            print(f"   Move({vx}, {vy}, {vyaw}) result: {ret}")
            time.sleep(2)
            self.print_state()
            
            # Stop after each movement
            ret = self.sport_client.StopMove()
            print(f"   StopMove result: {ret}")
            time.sleep(1)
            
        # Try some other movement functions
        print("\n5. Testing other movement functions...")
        
        try:
            print("   Trying ClassicWalk...")
            ret = self.sport_client.ClassicWalk(True)
            print(f"   ClassicWalk(True) result: {ret}")
            time.sleep(2)
            self.print_state()
            
            # Try moving with classic walk enabled
            print("   Move forward FAST with ClassicWalk...")
            ret = self.sport_client.Move(2.0, 0.0, 0.0)  # Much faster!
            print(f"   Move result: {ret}")
            time.sleep(3)
            self.print_state()
            
            ret = self.sport_client.StopMove()
            ret = self.sport_client.ClassicWalk(False)
            
        except Exception as e:
            print(f"   ClassicWalk test failed: {e}")
            
        # Final state
        print("\n6. Final robot state:")
        self.print_state()
        
        # Return to safe state
        print("\n7. Returning to safe state...")
        self.sport_client.StopMove()
        time.sleep(1)
        self.sport_client.StandDown()

def main():
    print("🔍 Go2 Robot Movement Debugger")
    print("=" * 50)
    print("This will test movement commands and show robot state.")
    print("")
    
    print("⚠️  WARNING: Ensure clear space around robot!")
    input("Press Enter to continue...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create debugger and run tests
    debugger = RobotDebugger()
    
    # Wait for first state message
    print("Waiting for robot state data...")
    start_time = time.time()
    while debugger.latest_state is None and time.time() - start_time < 5:
        time.sleep(0.1)
        
    if debugger.latest_state is None:
        print("❌ No robot state data received - check connection")
        return
        
    try:
        debugger.test_movements()
    except KeyboardInterrupt:
        print("\n🛑 Test stopped by user")
    except Exception as e:
        print(f"\n❌ Error during testing: {e}")
    finally:
        print("🔌 Debug session complete")

if __name__ == "__main__":
    main()