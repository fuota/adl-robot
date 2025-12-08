# adl-robot

Human-robot collaborative system using a Kinova Gen3 arm to perform Activities of Daily Living (ADLs) such as pouring, placing, and delivering objects, with tablet and voice control for human-in-the-loop assistance.

## Mobile UI Setup

The mobile app connects directly to ROS via rosbridge WebSocket. The `backend/` folder is optional and not required for mobile operation.

### Prerequisites

1. **Node.js and npm/pnpm** - For running the mobile app
2. **Expo CLI** - For development builds
3. **ROS 2 with rosbridge** - For robot communication
4. **iOS/Android development environment** - For native builds

### Installation

1. Navigate to the mobile directory:
```bash
cd mobile
```

2. Install dependencies:
```bash
npm install
# or
pnpm install
```

### ROS Bridge Setup

The mobile app connects to ROS via rosbridge WebSocket. You need to:

1. **Install rosbridge** (if not already installed):
```bash
sudo apt-get install ros-humble-rosbridge-suite
```

2. **Start rosbridge WebSocket server**:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

3. **Configure ROS bridge URL** (if needed):
   - Default URL: `ws://192.168.0.217:9090`
   - Edit `mobile/contexts/RosContext.tsx` to change the connection URL
   - Make sure the IP address matches your ROS machine's IP

4. **Ensure ROS services are running**:
   - The app expects ROS services like `/get_available_tasks`, `/prepare_medicine`, etc.
   - Make sure your ROS nodes are running and advertising these services

### Running the Mobile App

#### iOS
```bash
cd mobile
npm run ios
# or
npx expo run:ios
```

#### Android
```bash
cd mobile
npm run android
# or
npx expo run:android
```

#### Web (for testing)
```bash
cd mobile
npm run web
# or
npx expo start --web
```

### Development

Start the Expo development server:
```bash
cd mobile
npm start
# or
npx expo start
```

Then:
- Press `i` for iOS simulator
- Press `a` for Android emulator
- Press `w` for web browser
- Scan QR code with Expo Go app on your device

### Features

- **Direct ROS Integration**: Connects to ROS via rosbridge WebSocket
- **Real-time Task Progress**: Subscribes to `/task_progress` topic for live updates
- **Task Execution**: Calls ROS services to start/stop tasks
- **Camera Stream**: Displays live camera feed from ROS image topics
- **Voice Control**: Voice commands for task control (start, stop, navigate)
- **Object Detection**: Visualizes detected objects from ROS topics

### Configuration

- **ROS Bridge URL**: `mobile/contexts/RosContext.tsx` (line 26)
- **Backend URL** (optional, for web fallback): `mobile/config/backend.ts`

### Troubleshooting

1. **Connection Issues**:
   - Verify rosbridge is running: `ros2 topic list` should show rosbridge topics
   - Check firewall settings on port 9090
   - Ensure mobile device and ROS machine are on the same network
   - Update IP address in `RosContext.tsx` if needed

2. **Tasks Not Loading**:
   - Verify ROS service `/get_available_tasks` is available: `ros2 service list | grep get_available_tasks`
   - Check ROS node is running and advertising services

3. **Camera Not Showing**:
   - Verify camera topic exists: `ros2 topic list | grep camera`
   - Check topic name matches in `CameraStream.tsx` (default: `/camera/camera/color/image_raw/compressed`)

