# LiDAR Tools

Tools for accessing and streaming LiDAR data from Unitree Go2 robot.

## Files

### Main Scripts
- `lidar_access.py` - Comprehensive LiDAR data access with visualization
- `check_lidar_status.py` - Check LiDAR state and connectivity
- `lidar_switch.py` - Turn LiDAR ON/OFF via DDS commands

### Diagnostic Tools
- `test_dds_connectivity.py` - Test basic DDS communication with robot
- `discover_topics.py` - Discover active DDS topics on robot
- `try_different_domains.py` - Test different DDS domain IDs
- `ros2_remote_launcher.py` - Attempt remote ROS2 service activation

### Documentation
- `lidar_diagnosis.md` - Detailed diagnosis of LiDAR data access issues

## Usage

### Prerequisites
```bash
# Install SDK in conda environment
source ~/miniconda3/bin/activate unitree
cd ../unitree_sdk2_python && pip install -e .
```

### Basic LiDAR Access
```bash
# Full LiDAR access with visualization
python3 lidar_access.py --interface en8 --visualize

# Check LiDAR status
python3 check_lidar_status.py en8

# Turn LiDAR ON/OFF
python3 lidar_switch.py en8 ON
```

### Diagnostics
```bash
# Test robot connectivity
python3 test_dds_connectivity.py en8

# Find active topics
python3 discover_topics.py en8
```

## Robot Setup Required

The robot's LiDAR driver service must be running. SSH into robot:
```bash
sshpass -p '123' ssh unitree@192.168.123.18
# Start LiDAR driver service (exact command TBD)
ros2 run livox_ros_driver2 livox_ros_driver2_node
```

## Network Configuration
- Robot IP: 192.168.123.18
- Computer IP: 192.168.123.51/24
- Interface: en8