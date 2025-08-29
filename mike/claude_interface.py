#!/usr/bin/env python3
"""
Claude API interface for natural language command processing
Extracted from server for standalone testing and debugging
"""

import json
import os
from typing import Optional

# Try to import Claude API
try:
    from anthropic import Anthropic
    CLAUDE_AVAILABLE = True
except ImportError as e:
    CLAUDE_AVAILABLE = False
    print(f"⚠️  Claude API not available: {e}")

class ClaudeInterface:
    """Claude API integration for natural language command processing"""
    
    def __init__(self):
        self.client = None
        self.connected = False
        if CLAUDE_AVAILABLE:
            try:
                # Check for API key in order: env var, dotfile
                api_key = self._get_api_key()
                if api_key:
                    self.client = Anthropic(api_key=api_key)
                    self.connected = True
                    print("🤖 Claude connected")
                else:
                    print("⚠️  No API key found")
            except Exception as e:
                print(f"⚠️  Failed to initialize Claude client: {e}")
    
    def _get_api_key(self):
        """Get API key from environment variable or dotfile"""
        # First try environment variable
        api_key = os.environ.get('ANTHROPIC_API_KEY')
        if api_key:
            return api_key
        
        # Then try dotfile
        dotfile_path = os.path.expanduser('~/.anthropic_api_key')
        if os.path.exists(dotfile_path):
            try:
                with open(dotfile_path, 'r') as f:
                    api_key = f.read().strip()
                if api_key:
                    return api_key
            except Exception as e:
                print(f"⚠️  Could not read API key from {dotfile_path}: {e}")
        
        return None
    
    def save_api_key(self, api_key: str):
        """Save API key to dotfile"""
        dotfile_path = os.path.expanduser('~/.anthropic_api_key')
        try:
            with open(dotfile_path, 'w') as f:
                f.write(api_key)
            # Set file permissions to be readable only by user
            os.chmod(dotfile_path, 0o600)
            print(f"✅ API key saved to {dotfile_path}")
            return True
        except Exception as e:
            print(f"❌ Failed to save API key: {e}")
            return False
    
    async def process_natural_language_command(self, command: str, robot_state: dict = None, use_robot_prompt: bool = False) -> dict:
        """Process natural language command and return structured response"""
        if not self.connected:
            return {
                "success": False,
                "error": "Claude API not available",
                "response": "Claude integration is not properly configured."
            }
        
        try:
            # Choose system prompt based on context
            if use_robot_prompt and robot_state:
                system_prompt = f"""You are controlling a Unitree Go2 quadruped robot dog. You can:

1. **Movement Commands** (SportClient.Move(vx, vy, vyaw)):
   - vx: Forward/backward speed in m/s (-3.0 to 3.0, positive = forward)
   - vy: Lateral left/right speed in m/s (-3.0 to 3.0, positive = right)  
   - vyaw: Rotational speed in rad/s (-3.0 to 3.0, positive = clockwise)
   - Speed presets: slow (0.5), normal (1.5), fast (2.5), turbo (3.0)

2. **Pose Commands** (SportClient methods):
   - "standup": StandUp() - Make robot stand
   - "standdown": StandDown() - Make robot lie down
   - "recovery": RecoveryStand() - Recover from unstable position
   - "stop": StopMove() - Immediately halt movement
   - "damp": Damp() - Enter passive/safe mode

3. **Advanced Movements**:
   - Circle/spin: high angular_velocity with zero linear velocity
   - Figure-8 patterns: combine angular and linear velocities
   - Dance moves: sequence of pose changes
   - Patrol patterns: forward, turn, repeat

4. **Query robot status** using the current state data

5. **Execute Python code** for calculations or complex operations

Current robot state: {json.dumps(robot_state, indent=2)}

Respond with JSON in one of these formats:

For movement commands:
{{
  "action": "move",
  "command": {{
    "vx": 1.5,
    "vy": 0.0,
    "vyaw": 0.0
  }},
  "english_message": "Moving forward at normal speed!",
  "dogspeak": "Woof! Here I go!"
}}

For spinning/turning:
{{
  "action": "move", 
  "command": {{
    "vx": 0.0,
    "vy": 0.0,
    "vyaw": 1.5
  }},
  "english_message": "Spinning in a circle!",
  "dogspeak": "Arf arf! Spinny time!"
}}

For pose commands:
{{
  "action": "pose",
  "command": "standup",
  "english_message": "Standing up like a good dog!",
  "dogspeak": "Woof! Up and ready!"
}}

For sequences (like pushups, dancing, etc):
{{
  "action": "sequence",
  "sequence": "standdown wait(2) standup wait(2) standdown wait(2) standup",
  "english_message": "I'll do pushups! Lie down, stand up, repeat!",
  "dogspeak": "Woof woof! Doggy pushups!"
}}

For Python code execution:
{{
  "action": "execute",
  "code": "print('Hello from robot!')\\nresult = 2 + 2\\nprint(f'Calculation: {{result}}')",
  "english_message": "Running a calculation for you",
  "dogspeak": "Arf arf! Computing!"
}}

For information/status:
{{
  "action": "info", 
  "english_message": "The robot is currently connected and ready to receive commands",
  "dogspeak": "Woof! I'm ready to play!"
}}

Be helpful, safe, and explain what you're doing."""
                content = f"User command: {command}"
            else:
                system_prompt = "You are a helpful AI assistant. Respond naturally to the user's questions and requests."
                content = command
            
            # Send request to Claude
            message = self.client.messages.create(
                model="claude-sonnet-4-20250514",  # Use Claude Sonnet 4
                max_tokens=1000,
                temperature=0.3,
                system=system_prompt,
                messages=[
                    {
                        "role": "user", 
                        "content": content
                    }
                ]
            )
            
            response_text = message.content[0].text
            
            # Try to parse JSON response if using robot prompt
            if use_robot_prompt:
                try:
                    response_data = json.loads(response_text)
                    return {
                        "success": True,
                        "response": response_data,
                        "raw_response": response_text
                    }
                except json.JSONDecodeError:
                    # If not valid JSON, return as info response
                    return {
                        "success": True,
                        "response": {
                            "action": "info",
                            "response": response_text,
                            "explanation": "Claude provided a text response"
                        },
                        "raw_response": response_text
                    }
            else:
                return {
                    "success": True,
                    "response": response_text
                }
                
        except Exception as e:
            print(f"❌ Claude API error: {e}")
            return {
                "success": False,
                "error": str(e),
                "response": f"Error processing command: {e}"
            }

# Non-async version for standalone testing
def process_command_sync(command: str, robot_state: dict = None) -> dict:
    """Synchronous version for testing"""
    import asyncio
    
    if robot_state is None:
        robot_state = {
            "position": {"x": 0, "y": 0, "z": 0},
            "orientation": {"roll": 0, "pitch": 0, "yaw": 0},
            "velocity": {"vx": 0, "vy": 0, "vz": 0},
            "battery_level": 85.5,
            "temperature": 32.1,
            "mode": "idle",
            "connected": True,
            "timestamp": 1693245123.456
        }
    
    claude = ClaudeInterface()
    
    # Run the async method in a sync context
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        result = loop.run_until_complete(
            claude.process_natural_language_command(command, robot_state)
        )
        return result
    finally:
        loop.close()

if __name__ == "__main__":
    # Test the interface
    test_command = "What's 2+2?"
    print(f"Testing command: '{test_command}'")
    result = process_command_sync(test_command)
    print(f"Result: {json.dumps(result, indent=2)}")