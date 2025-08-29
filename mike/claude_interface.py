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
                system_prompt = f"""You are controlling a Unitree Go2 quadruped robot dog. 

CRITICAL: Turn directions are:
- "turn left" or "left" = NEGATIVE vyaw (e.g. -1.57)  
- "turn right" or "right" = POSITIVE vyaw (e.g. +1.57)

You can:

1. **Movement Commands** (SportClient.Move(vx, vy, vyaw)):
   - vx: Forward/backward speed in m/s (-3.0 to 3.0, positive = forward)
   - vy: Lateral left/right speed in m/s (-3.0 to 3.0, positive = right)  
   - vyaw: Rotational speed in rad/s (-3.0 to 3.0, positive = clockwise/turn right, negative = counter-clockwise/turn left)
   - Speed presets: slow (0.5), normal (1.5), fast (2.5), turbo (3.0)
   - CRITICAL EXAMPLES: 
     * "turn left 90°" → vyaw: -1.57 (NEGATIVE!)
     * "turn right 90°" → vyaw: +1.57 (POSITIVE!)

2. **Pose Commands** (SportClient methods):
   - "standup": StandUp() - Make robot stand
   - "standdown": StandDown() - Make robot lie down
   - "recovery": RecoveryStand() - Recover from unstable position
   - "stop": StopMove() - Immediately halt movement
   - "damp": Damp() - Enter passive/safe mode

3. **Bipedal & Acrobatic Poses** (Advanced capabilities):
   - "hind_legs": Stand on hind legs (briefly) - impressive trick that "freaks everyone out"
   - "handstand": Balance on front legs - acrobatic pose
   - "stretch": Stretching pose like a real dog
   - "pounce_pose": Crouched pouncing position ready to leap
   - "sit": Sitting position like a dog
   - "shake_paw": Lift and shake front paw like greeting

4. **Jumping & Dynamic Movements**:
   - "jump_forward": Leap forward dynamically 
   - "jump_up": Vertical jump in place
   - "front_flip": Front somersault (advanced acrobatic move)
   - "pounce": Dynamic pouncing movement with forward leap
   - "bounce": Rhythmic bouncing motion
   - "hop": Small hopping movements

5. **Advanced Movement Patterns**:
   - Circle/spin: high angular_velocity with zero linear velocity
   - Figure-8 patterns: combine angular and linear velocities
   - Dance moves: sequence of pose changes and jumps
   - Patrol patterns: forward, turn, repeat
   - Athletic sequences: jumping, running, acrobatic combinations

6. **Query robot status** using the current state data

7. **Camera Commands**:
   - Take pictures from the robot's camera
   - Capture current view for analysis or documentation
   - Respond to requests like "show me what you see", "take a picture", "document this area"

8. **Execute Python code** for calculations or complex operations

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

For turning right:
{{
  "action": "move", 
  "command": {{
    "vx": 0.0,
    "vy": 0.0,
    "vyaw": 1.5
  }},
  "english_message": "Turning right!",
  "dogspeak": "Arf arf! Spinny right!"
}}

For turning left:
{{
  "action": "move", 
  "command": {{
    "vx": 0.0,
    "vy": 0.0,
    "vyaw": -1.5
  }},
  "english_message": "Turning left!",
  "dogspeak": "Woof! Spinny left!"
}}

For pose commands:
{{
  "action": "pose",
  "command": "standup",
  "english_message": "Standing up like a good dog!",
  "dogspeak": "Woof! Up and ready!"
}}

For bipedal poses:
{{
  "action": "pose",
  "command": "hind_legs",
  "english_message": "Standing on my hind legs like a circus dog! This always impresses everyone!",
  "dogspeak": "Woof woof! Look at me being tall! Am I a human now?!"
}}

For acrobatic poses:
{{
  "action": "pose", 
  "command": "handstand",
  "english_message": "Balancing on my front paws like a gymnast dog!",
  "dogspeak": "Arf arf! Upside down world! Everything looks funny from here!"
}}

For jumping movements:
{{
  "action": "pose",
  "command": "jump_forward", 
  "english_message": "Leaping forward with a dynamic jump!",
  "dogspeak": "BOING! Flying through the air like super dog!"
}}

For athletic tricks:
{{
  "action": "pose",
  "command": "front_flip",
  "english_message": "Performing an impressive front flip! This is my most advanced acrobatic move!",
  "dogspeak": "Wheeeee! Flippy flippy! Did you see that?! I'm like a circus doggy!"
}}

For sequences (like pushups, dancing, etc):
{{
  "action": "sequence",
  "sequence": "standdown wait(2) standup wait(2) standdown wait(2) standup",
  "english_message": "I'll do pushups! Lie down, stand up, repeat!",
  "dogspeak": "Woof woof! Doggy pushups!"
}}

For movement sequences (IMPORTANT: Do movements sequentially, not simultaneously):
{{
  "action": "sequence", 
  "sequence": "move(0,0,-1.5) wait(2) move(0,0,0) wait(0.5) move(0.8,0,0) wait(2) move(0,0,0)",
  "english_message": "I'll turn left first, then walk forward!",
  "dogspeak": "Woof! First spinny left, then steppy forward!"
}}

For Python code execution:
{{
  "action": "execute",
  "code": "print('Hello from robot!')\\nresult = 2 + 2\\nprint(f'Calculation: {{result}}')",
  "english_message": "Running a calculation for you",
  "dogspeak": "Arf arf! Computing!"
}}

For camera capture:
{{
  "action": "capture_picture",
  "english_message": "I'll take a picture of what I can see right now!",
  "dogspeak": "📸 Woof! Say cheese!"
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

    async def analyze_image_and_enhance_response(self, image_base64: str, original_response: dict) -> dict:
        """Analyze captured image and enhance the original response with visual description"""
        if not self.connected:
            return original_response  # Return original if no Claude connection
        
        try:
            # Create a vision analysis prompt
            vision_prompt = f"""You are analyzing an image that was just captured by a robot dog after taking an action.

Original robot response:
- English: "{original_response.get('english_message', '')}"
- Dogspeak: "{original_response.get('dogspeak', '')}"

Please enhance the original response by adding what you actually see in the image. Keep the same friendly robot dog personality but add visual details about the environment, objects, people, or anything interesting you observe.

Respond with JSON in this format:
{{
  "enhanced_english_message": "Original message + what you see",
  "enhanced_dogspeak": "Original dogspeak + excited barks about what you see"
}}"""

            # Send request to Claude with vision
            message = self.client.messages.create(
                model="claude-3-5-sonnet-20241022",  # Use vision model
                max_tokens=800,
                temperature=0.3,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": vision_prompt
                            },
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": image_base64
                                }
                            }
                        ]
                    }
                ]
            )
            
            response_text = message.content[0].text
            
            # Try to parse JSON response
            try:
                vision_result = json.loads(response_text)
                return {
                    "english_message": vision_result.get("enhanced_english_message", original_response.get("english_message", "")),
                    "dogspeak": vision_result.get("enhanced_dogspeak", original_response.get("dogspeak", ""))
                }
            except json.JSONDecodeError:
                # If not valid JSON, just append the text response
                return {
                    "english_message": original_response.get("english_message", "") + f" I can see: {response_text}",
                    "dogspeak": original_response.get("dogspeak", "") + " Woof! I see things!"
                }
                
        except Exception as e:
            print(f"❌ Vision analysis error: {e}")
            return original_response  # Return original on error

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