# ADL Robot - ArUco Object Detection and Manipulation

This ROS2 package provides object detection using ArUco markers and robotic manipulation capabilities for Activities of Daily Living (ADL) tasks using a Kinova Gen3 7-DoF robotic arm and Intel RealSense camera.

## Features

- **ArUco Object Detection**: Detects objects marked with ArUco tags using RealSense RGB-D camera
- **3D Pose Estimation**: Calculates 6DoF poses of detected objects using camera calibration and depth information
- **Robotic Manipulation**: Controls Kinova Gen3 arm for pick and place operations
- **ADL Task Coordination**: High-level task management for complex ADL scenarios
- **ROS2 Services**: Provides pickup and placement services for external control

## Package Structure

```
adl_robot/
├── adl_robot/
│   ├── __init__.py
│   ├── aruco_detector.py          # ArUco detection node
│   ├── kinova_controller.py       # Kinova arm control
│   ├── object_pickup_service.py   # Pickup/place service coordinator
│   └── adl_coordinator.py         # High-level ADL task manager
├── config/
│   └── adl_robot_config.yaml      # Configuration parameters
├── launch/
│   ├── adl_robot.launch.py        # Full system launch
│   └── aruco_detection_only.launch.py  # Detection testing only
├── msg/
│   ├── DetectedObject.msg         # Single detected object
│   └── DetectedObjectArray.msg    # Array of detected objects
├── srv/
│   ├── PickupObject.srv           # Object pickup service
│   └── PlaceObject.srv            # Object placement service
├── package.xml
├── setup.py
└── README.md
```

## Prerequisites

### Hardware Requirements
- Kinova Gen3 7-DoF Robotic Arm
- Intel RealSense D435 (or compatible) RGB-D Camera
- Computer with ROS2 Humble

### Software Dependencies
- ROS2 Humble
- OpenCV with ArUco support
- RealSense ROS2 package
- MoveIt2 (for advanced motion planning)
- Kinova ROS2 driver

### ArUco Markers
Print ArUco markers from the DICT_4X4_250 dictionary. Recommended marker size: 5cm x 5cm.

## Installation

1. **Clone the repository into your ROS2 workspace:**
```bash
cd ~/ros2_ws/src
git clone <repository_url> adl_robot
```

2. **Install dependencies:**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Install Python dependencies:**
```bash
pip install opencv-python opencv-contrib-python transforms3d
```

4. **Build the package:**
```bash
cd ~/ros2_ws
colcon build --packages-select adl_robot
source install/setup.bash
```

## Usage

### 1. Object Detection Testing

To test ArUco detection with just the camera:

```bash
ros2 launch adl_robot aruco_detection_only.launch.py
```

This will:
- Start the RealSense camera
- Launch the ArUco detector
- Display debug images with detected markers

**View detected objects:**
```bash
ros2 topic echo /detected_objects
```

**View debug image:**
```bash
ros2 run image_view image_view image:=/aruco_debug_image
```

### 2. Full System Launch

To start the complete ADL robot system:

```bash
ros2 launch adl_robot adl_robot.launch.py
```

This launches:
- RealSense camera
- ArUco detector
- Kinova controller
- Object pickup service
- ADL coordinator
- RViz visualization

### 3. Manual Object Pickup

**Pick up an object by ArUco ID:**
```bash
ros2 service call /pickup_object adl_robot/srv/PickupObject "{marker_id: 0, object_type: 'book', target_pose: {header: {frame_id: ''}, pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, use_precise_grasp: true}"
```

**Place object at a location:**
```bash
ros2 service call /place_object adl_robot/srv/PlaceObject "{target_pose: {header: {frame_id: 'base_link', stamp: {sec: 0, nanosec: 0}}, pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, release_object: true}"
```

### 4. Predefined ADL Tasks

The ADL coordinator can execute complex tasks automatically:

**Monitor coordinator status:**
```bash
ros2 topic echo /rosout | grep adl_coordinator
```

## Configuration

### ArUco Marker Mapping

Edit `config/adl_robot_config.yaml` to map ArUco IDs to object types:

```yaml
object_mapping:
  0: "book"
  1: "water_bottle"
  2: "cup"
  # Add more mappings...
```

### Placement Locations

Define predefined locations for object placement:

```yaml
placement_locations:
  shelf_high: [0.6, -0.3, 0.8]
  table_center: [0.5, 0.0, 0.3]
  # Add more locations...
```

### Camera-Robot Calibration

**Important**: You must calibrate the camera position relative to the robot base. Update the static transform in the launch file:

```python
camera_base_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        'x', 'y', 'z',     # Translation (meters)
        'qx', 'qy', 'qz', 'qw',  # Quaternion rotation
        'base_link', 'camera_link'
    ]
)
```

## Troubleshooting

### Common Issues

1. **No objects detected:**
   - Ensure ArUco markers are visible and well-lit
   - Check marker size matches configuration (default: 5cm)
   - Verify camera topics are publishing: `ros2 topic list | grep camera`

2. **TF transform errors:**
   - Calibrate camera-to-robot transform
   - Check that all required frames are being published: `ros2 run tf2_tools view_frames.py`

3. **Kinova arm not moving:**
   - Ensure Kinova driver is running
   - Check joint trajectory action server: `ros2 action list | grep follow_joint_trajectory`
   - Verify arm is not in fault state

4. **Service call failures:**
   - Check service availability: `ros2 service list`
   - Ensure all nodes are running: `ros2 node list`

### Debug Commands

**Check node status:**
```bash
ros2 node list
ros2 node info /aruco_detector
```

**Monitor topics:**
```bash
ros2 topic list
ros2 topic hz /detected_objects
ros2 topic echo /detected_objects --once
```

**Test services:**
```bash
ros2 service list
ros2 service type /pickup_object
```

**View TF tree:**
```bash
ros2 run tf2_tools view_frames.py
```

## Development

### Adding New Object Types

1. Update the object mapping in `config/adl_robot_config.yaml`
2. Create corresponding ArUco markers with the assigned IDs
3. Optionally add object-specific grasp strategies in `kinova_controller.py`

### Custom ADL Tasks

Add new task methods to `adl_coordinator.py`:

```python
def execute_adl_task_custom(self):
    """Execute custom ADL task"""
    # Define task steps
    # Use self.pickup_object_by_id() and self.place_object_at_location()
```

### Integration with UI

The package provides ROS2 services that can be called from:
- Web interfaces
- Mobile apps
- Voice command systems
- Other ROS2 nodes

## Safety Considerations

- Always ensure the robot workspace is clear of people
- Test with lightweight objects first
- Monitor robot movements and be ready to stop execution
- Validate reachability before attempting pickup/placement
- Use proper collision detection and workspace limits

## Contributing

1. Follow ROS2 coding standards
2. Add unit tests for new functionality
3. Update documentation for new features
4. Test thoroughly with real hardware

## License

MIT License - see LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review ROS2 logs: `ros2 log view`
3. Create an issue in the repository with full error logs and system information