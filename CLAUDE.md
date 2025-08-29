Add things here that future versions of me should know about this codebase.

## Quick Start: "Run the Dog Thing"

To get the Unitree Go2 robot server working:

### Option 1: Already Set Up
```bash
cd mike
./server.sh  # Auto-detects Python 3.11 environment
```

### Option 2: First Time Setup (Required Once)
If the above fails, run this complete setup:

```bash
# Install pyenv for Python version management
brew install pyenv

# Add pyenv to shell (restart terminal after this)
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
echo '[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc  
echo 'eval "$(pyenv init - zsh)"' >> ~/.zshrc
exec "$SHELL"

# Install Python build dependencies
brew install openssl readline sqlite3 xz tcl-tk@8 libb2 zstd cmake

# Install Python 3.11 (required for Unitree SDK)
pyenv install 3.11
pyenv global 3.11

# Build CycloneDDS (required for robot communication)
# ⚠️  DO NOT use "cycloneDX" - that's a security tool, not the DDS library!
cd ~/
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install

# Install Unitree SDK with dependencies
cd /path/to/your/project  # Replace with actual path
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
export CYCLONEDDS_HOME=$HOME/cyclonedds/install
pip install -e .

# Install server dependencies
cd mike/  # Your mike directory
pip install -r requirements.txt

# Now you can run the server
./server.sh
```

### Client Usage
```bash
cd mike
./client.py  # WASD controls, ESC to quit
```

## Instructions
- Always open URLs for pull requests after creating or pushing them so the human can see them
- Always switch back to main and pull before starting new work
- ⚠️  **AVOID "cycloneDX"** - That's a security scanning tool, not the DDS communication library we need!

## Environment & Technical Notes
- **Python Requirements**: Must use Python 3.11 with pyenv for Unitree SDK compatibility
- **CycloneDDS**: Required for robot communication via DDS protocol
- **Unitree SDK**: Real robot control requires `unitree_sdk2py` package with proper CycloneDDS setup
- **Network Setup**: 
  - Ethernet: Robot at 192.168.123.18, PC should be 192.168.123.51/24
  - WiFi: Robot at 192.168.12.1, connect to network GO2_XXXXXX
- **Unitree Go2 Robot**: 
  - Hand controller L2 double-click disables obstacle avoidance for ball interaction
  - Camera access via SDK VideoClient or GStreamer: `udpsrc address=230.1.1.1 port=1720`
  - Multiple controller types: Handheld (bimanual) and Companion (side-following)
  - Server shows "✅ Connected: 🤖 Control API + 📹 Camera + 🌐 Network" when fully working