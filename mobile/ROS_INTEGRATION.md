# ROS Integration - Mobile App

## Overview

The mobile app now integrates with the ROS backend to dynamically load tasks, monitor real-time progress, and execute robot tasks.

## ROS Endpoints Used

### 1. Service Calls

#### Get Available Tasks

- **Service:** `/get_available_tasks`
- **Type:** `std_srvs/Trigger`
- **Response:** JSON containing all available tasks with their definitions

```json
{
  "tasks": {
    "prepare_medicine": {
      "name": "Prepare Medicine",
      "description": "Places water cup, plate, and pours water and medicine",
      "total_steps": 4,
      "steps": [...],
      "estimated_duration_seconds": 120
    },
    ...
  }
}
```

#### Execute Tasks

- **Services:**
  - `/prepare_medicine` - 4 steps: Place water cup → Place plate → Pour water → Pour medicine
  - `/setup_tableware` - 3 steps: Place bowl → Place fork → Place spoon
  - `/organize_books` - 2 steps: Place book 1 → Place book 2
- **Type:** `std_srvs/Trigger`
- **Response:** `{ success: boolean, message: string }`

### 2. Topic Subscriptions

#### Task Progress Updates

- **Topic:** `/task_progress`
- **Type:** `std_msgs/String` (JSON payload)
- **Message Format:**

```json
{
  "task": "prepare_medicine",
  "current_step": 2,
  "total_steps": 4,
  "description": "Placing plate",
  "status": "in-progress", // "in-progress" | "completed" | "failed"
  "progress_percent": 50
}
```

#### Object Detection (Optional - for future use)

- `/detected_objects/poses` - Object positions
- `/detected_objects/ids` - Object marker IDs
- `/detected_objects/names` - Object names

## Implementation Details

### File Changes

1. **`types/tasks.ts`** (NEW)
   - Type definitions for task data structures
   - Synced with ROS backend message formats

2. **`contexts/TaskContext.tsx`** (UPDATED)
   - Loads available tasks from `/get_available_tasks` service
   - Subscribes to `/task_progress` for real-time updates
   - Provides `startTask()` function to execute tasks via ROS services
   - Auto-updates task state based on progress messages

3. **`app/index.tsx`** (UPDATED)
   - Displays tasks from TaskContext instead of hardcoded data
   - Shows task metadata (steps, estimated duration)
   - Loading state while fetching from ROS

4. **`app/task.tsx`** (UPDATED)
   - Real-time progress bar updates
   - Step-by-step status indicators
   - Dynamic badge (Ready/Running/Completed/Failed)
   - Start button triggers ROS service calls

5. **`components/tasks/TaskStep.tsx`** (UPDATED)
   - Added "failed" status with red styling
   - Visual indicators for all states

## Usage

### Starting a Task

```typescript
const { startTask } = useTask();

// Start medicine preparation
startTask("1"); // Calls /prepare_medicine service
```

### Monitoring Progress

Progress updates are automatically handled by TaskContext. The UI will update in real-time as the robot completes each step.

### Task States

- **not-started** - Task hasn't been executed yet
- **in-progress** - Task is currently running (updated via `/task_progress`)
- **completed** - All steps finished successfully
- **failed** - Task encountered an error

## Connection Requirements

1. **ROS Bridge WebSocket**
   - Must be running on `ws://192.168.0.217:9090` (configurable in `RosContext.tsx`)
   - Install: `sudo apt-get install ros-humble-rosbridge-suite`
   - Run: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

2. **Backend Services**
   - Ensure `real_detection_pick_place.py` is running
   - Services must be advertised on ROS network

## Testing

### Test without Robot

The app gracefully handles disconnection by showing default task data. When ROS is unavailable:

- Tasks are shown with default metadata
- Start button will not execute (shows error)
- No progress updates

### Test with Robot

1. Start rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
2. Start detection node: `ros2 run detect_moveit real_detection_pick_place`
3. Open mobile app - tasks should load automatically
4. Click on a task to view details
5. Press "Start" to execute
6. Watch real-time progress updates

## Future Enhancements

- [ ] Add pause/resume functionality
- [ ] Add stop/cancel task functionality
- [ ] Implement voice command integration with ROS
- [ ] Subscribe to object detection topics for visualization
- [ ] Add camera feed from ROS image topics
- [ ] Task history and logging
- [ ] Manual control primitives (grasp, place, move)
