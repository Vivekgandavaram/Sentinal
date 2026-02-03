# 🤖 Person-Following Robot with NS312 PIR Sensor

A smart autonomous robot that follows you around using AI-powered computer vision and motion detection. Built with Raspberry Pi, this robot can detect, track, and follow people while avoiding obstacles and adjusting its movements in real-time.

## ✨ Features

### 🎯 Core Capabilities
- **AI-Powered Person Detection**: Uses the IMX500 neural accelerator with MobileNetV2 for real-time person detection
- **Autonomous Following**: Automatically tracks and follows detected persons while maintaining optimal distance
- **360° Search Mode**: Intelligently searches for people when they move out of view
- **NS312 PIR Sensor Integration**: Detects motion behind the robot and performs 180° turns to locate people
- **Smart Movement Tracking**: Follows moving targets even when the robot is stationary
- **Web-Based Control Interface**: Beautiful, responsive web UI for monitoring and control

### 🧠 Intelligent Behaviors
- **Adaptive Speed Control**: Adjusts speed based on distance and centering requirements
- **Precise Centering**: Keeps the tracked person in the center of its field of view
- **Distance Maintenance**: Maintains optimal following distance (configurable)
- **Smooth Turning**: Proportional control for natural movement
- **Lost Person Recovery**: Automated 360° search when person is no longer detected

## 📋 Requirements

### Hardware
- **Raspberry Pi 4/5** (recommended for performance)
- **Raspberry Pi AI Camera** with IMX500 sensor
- **NS312 PIR Motion Sensor**
- **Motor Driver** (L298N or similar dual H-bridge)
- **2x DC Motors** for differential drive
- **Robot Chassis** with wheels
- **Power Supply** (appropriate for motors and Pi)

### Software Dependencies
```bash
# System packages
sudo apt-get update
sudo apt-get install python3-opencv python3-picamera2 python3-rpi-gpio

# Python packages (if needed)
pip3 install numpy opencv-python --break-system-packages
```

### Pre-installed on Raspberry Pi OS
- IMX500 firmware and models (usually in `/usr/share/imx500-models/`)
- Picamera2 library
- RPi.GPIO library

## 🔌 Hardware Connections

### Motor Connections
```
Left Motor:
  - Motor Pin 1 → GPIO 12
  - Motor Pin 2 → GPIO 13

Right Motor:
  - Motor Pin 1 → GPIO 17
  - Motor Pin 2 → GPIO 19
```

### NS312 PIR Sensor
```
NS312 Sensor → GPIO 27
VCC → 5V
GND → Ground
```

### Camera
- Connect the AI Camera to the appropriate CSI/camera port on your Raspberry Pi

## 🚀 Quick Start

### 1. Clone or Download
```bash
# Make the script executable
chmod +x robot_controller.py
```

### 2. Configure Settings (Optional)
Edit the configuration section at the top of `robot_controller.py`:

```python
# Motor GPIO pins
LEFT_MOTOR_PIN1 = 12
LEFT_MOTOR_PIN2 = 13
RIGHT_MOTOR_PIN1 = 17
RIGHT_MOTOR_PIN2 = 19

# NS312 sensor pin
NS312_SENSOR_PIN = 27

# Speed and behavior settings
INITIAL_SPEED = 60              # Base speed (0-100)
CENTER_TOLERANCE = 50           # Pixels for "centered" state
TARGET_PERSON_WIDTH = 350       # Target width for optimal distance
WIDTH_TOLERANCE = 60            # Tolerance for distance control

# Performance tuning
SEARCH_ROTATION_TIME = 12.0     # Seconds for full 360° search
MAX_NO_DETECTION_FRAMES = 10    # Frames before starting search
DETECTION_THRESHOLD = 0.55      # AI confidence threshold (0.0-1.0)
```

### 3. Run the Robot
```bash
sudo python3 robot_controller.py
```

### 4. Access the Web Interface
After starting, the script will display the robot's IP address:
```
✓ Access robot at: http://192.168.1.XXX:8080
```

Open this URL in any browser on your local network.

## 🎮 How to Use

### Web Interface Controls

1. **Live Preview**: Real-time camera feed with person detection boxes
2. **Auto Follow Toggle**: Enable/disable autonomous following mode
3. **Status Indicators**:
   - **Person Detected**: Green when a person is in view
   - **Centered**: Green when person is centered in frame
   - **Distance OK**: Green when at optimal following distance
   - **NS312**: Green when motion detected behind robot
4. **Current Action Display**: Shows what the robot is currently doing

### Operating Modes

#### Manual Mode (Auto Follow OFF)
- Robot remains stationary
- Camera still detects and displays people
- NS312 sensor shows detection status

#### Auto Follow Mode (Auto Follow ON)
The robot automatically:
1. **Searches** for people with 360° rotation
2. **Approaches** when person is too far
3. **Follows** when person moves while at correct distance
4. **Turns** to keep person centered
5. **Backs up** if too close
6. **Rotates 180°** when NS312 detects motion behind

## 🔧 Configuration Guide

### Speed Adjustments
```python
INITIAL_SPEED = 60                    # Base speed (increase for faster movement)
TURN_SPEED_FACTOR = 0.7               # Turning speed (0.0-1.0)
APPROACH_SPEED_FACTOR = 0.5           # Speed when approaching
FOLLOW_SPEED_FACTOR = 0.8             # Speed when following
SEARCH_SPEED_FACTOR = 0.6             # Speed during search rotation
```

### Distance Control
```python
TARGET_PERSON_WIDTH = 350    # Increase for closer following
WIDTH_TOLERANCE = 60         # Increase for more lenient distance control
MIN_PERSON_WIDTH = 80        # Minimum size to consider as person
```

### Centering Sensitivity
```python
CENTER_TOLERANCE = 50        # Pixels from center (decrease for tighter centering)
```

### Search Behavior
```python
SEARCH_ROTATION_TIME = 12.0        # Seconds for full rotation
MAX_NO_DETECTION_FRAMES = 10       # Frames to wait before searching
```

### NS312 Sensor
```python
NS312_CHECK_INTERVAL = 0.5         # Sensor polling interval (seconds)
NS312_PRESENCE_DEBOUNCE = 0.3      # Debounce time to avoid false triggers
```

## 🧪 Testing & Troubleshooting

### Verify Camera
```bash
# Test if camera is detected
rpicam-hello

# Check IMX500 models
ls /usr/share/imx500-models/
```

### Check GPIO Pins
```bash
# Install GPIO tools
sudo apt-get install raspi-gpio

# Check pin states
raspi-gpio get 12,13,17,19,27
```

### Common Issues

**Camera not initializing:**
- Ensure the camera is properly connected
- Check if IMX500 firmware is installed
- Verify camera is enabled in `raspi-config`

**Motors not responding:**
- Check all GPIO connections
- Verify motor driver power supply
- Test with manual GPIO control

**NS312 always triggered/never triggered:**
- Adjust sensor sensitivity (physical pot on sensor)
- Check wiring and power supply
- Modify `NS312_PRESENCE_DEBOUNCE` setting

**Web interface not accessible:**
- Check firewall settings
- Verify robot and client are on same network
- Try accessing via direct IP instead of hostname

**Person detection inaccurate:**
- Adjust `DETECTION_THRESHOLD` (lower = more sensitive)
- Ensure good lighting conditions
- Check camera focus and positioning

## 📊 Technical Details

### Detection System
- **Model**: MobileNetV2 SSD FPNLite (320x320)
- **Labels**: COCO dataset (80 classes, filtered for "person")
- **Inference**: Hardware-accelerated via IMX500
- **Frame Rate**: Configurable via camera settings

### Control Logic
1. **Person Detection**: Processes camera frames for person objects
2. **Tracking**: Identifies closest/largest person in frame
3. **Movement Decision**: Based on centering and distance
4. **Motor Control**: PWM signals to motor driver
5. **NS312 Monitor**: Separate thread for rear detection

### Movement States
- **Idle**: No auto mode, motors stopped
- **Searching 360**: Rotating to find person
- **Approach**: Moving forward toward distant person
- **Following**: Moving forward with centered person
- **Tracking**: Stationary, tracking person's movement
- **Turn Left/Right**: Rotating to center person
- **Back Up**: Moving backward if too close
- **Rotating 180**: Turning around for NS312 detection

## 🎨 Customization Ideas

### Extend Functionality
- Add obstacle avoidance with ultrasonic sensors
- Implement voice commands
- Add LED indicators for status
- Log movement patterns
- Create mobile app interface
- Add speed control from web interface
- Implement gesture recognition

### Modify Behavior
- Change to "patrol mode" instead of following
- Add multiple person tracking
- Implement face recognition for specific person following
- Create "stay near" mode instead of active following

## 📝 Code Structure

```
robot_controller.py
├── Configuration Constants
├── RobotState Class (shared state management)
├── NS312 Sensor Functions
├── Motor Control Functions
├── Camera & Detection Functions
├── Movement Tracking
├── NS312 Monitor Thread
├── Autonomous Control Loop
├── Web Server (HTTP interface)
└── Main Entry Point
```

## 🤝 Contributing

Feel free to fork, modify, and improve this project! Some areas for contribution:
- Better detection algorithms
- Smoother motion control
- Enhanced web interface
- Additional sensor support
- Battery monitoring
- Remote control features

## 📄 License

This project is open source. Feel free to use and modify for personal or educational purposes.

## ⚠️ Safety Notes

- Always test in a safe, open area
- Keep emergency stop accessible (Ctrl+C or power switch)
- Start with lower speeds and increase gradually
- Monitor battery levels to prevent damage
- Be cautious of stairs, edges, and obstacles
- Never leave robot unattended in auto mode

## 📞 Support

If you encounter issues:
1. Check the troubleshooting section above
2. Verify all hardware connections
3. Review configuration settings
4. Check system logs for errors

## 🎓 Learning Resources

- [Raspberry Pi Documentation](https://www.raspberrypi.org/documentation/)
- [OpenCV Python Tutorials](https://docs.opencv.org/master/d6/d00/tutorial_py_root.html)
- [IMX500 AI Camera Guide](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)
- [GPIO Programming](https://sourceforge.net/p/raspberry-gpio-python/wiki/Home/)

---

**Happy Building! 🤖✨**

Made with ❤️ for robotics enthusiasts and learners everywhere.
