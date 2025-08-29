Add things here that future versions of me should know about this codebase.

## Instructions
- Always open URLs for pull requests after creating or pushing them so the human can see them
- Always switch back to main and pull before starting new work

## Environment & Technical Notes
- **Python/pip**: Use `python3 -m pip` instead of `pip` - the system doesn't have `pip` in PATH
- **Unitree Go2**: 
  - Hand controller L2 double-click disables obstacle avoidance for ball interaction
  - Camera access via GStreamer pipeline: `udpsrc address=230.1.1.1 port=1720`
  - Default robot IP: 192.168.12.1 (ethernet) or WiFi network GO2_XXXXXX
  - Multiple controller types: Handheld (bimanual) and Companion (side-following)