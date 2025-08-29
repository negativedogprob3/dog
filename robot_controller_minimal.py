#!/usr/bin/env python3

import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

def test_basic_connection():
    """Test basic connection without keyboard library"""
    print("🔍 Testing basic SDK connection...")
    
    try:
        # Initialize channel factory
        if len(sys.argv) > 1:
            interface = sys.argv[1]
            print(f"Using network interface: {interface}")
            ChannelFactoryInitialize(0, interface)
        else:
            print("Using default network interface")
            ChannelFactoryInitialize(0)
        
        print("✅ Channel factory initialized")
        
        # Initialize sport client
        sport_client = SportClient()
        sport_client.SetTimeout(3.0)
        sport_client.Init()
        
        print("✅ Sport client initialized")
        
        # Test basic commands
        print("Testing Damp command...")
        ret = sport_client.Damp()
        print(f"Damp result: {ret}")
        
        time.sleep(1)
        
        print("Testing StandUp command...")
        ret = sport_client.StandUp()
        print(f"StandUp result: {ret}")
        
        time.sleep(3)
        
        print("Testing simple movement...")
        ret = sport_client.Move(0.1, 0.0, 0.0)
        print(f"Move result: {ret}")
        
        time.sleep(2)
        
        print("Testing StopMove...")
        ret = sport_client.StopMove()
        print(f"StopMove result: {ret}")
        
        time.sleep(1)
        
        print("Testing StandDown...")
        ret = sport_client.StandDown()
        print(f"StandDown result: {ret}")
        
        print("✅ All basic tests completed successfully!")
        
    except Exception as e:
        print(f"❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("🚀 Minimal Robot Controller Test")
    print("This tests basic SDK functionality without keyboard input")
    print("")
    
    print("⚠️  WARNING: Robot will perform basic movements!")
    input("Press Enter to continue...")
    
    test_basic_connection()