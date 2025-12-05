"""
ROS Subscriber Service
Listens to ROS topics for task completion updates and forwards them to the WebSocket server.
"""
import rospy
from std_msgs.msg import String
import json
import threading
from services.action_mapper import get_step_index_for_action

# Global reference to socketio instance (will be set by app)
socketio_instance = None

def set_socketio(socketio):
    """Set the Flask-SocketIO instance for emitting events."""
    global socketio_instance
    socketio_instance = socketio

def task_status_callback(msg):
    """
    Callback for /robot/task_status topic.
    Expected message format: JSON string with task_id, action, status
    Example: '{"task_id": "1", "action": "place_cup", "status": "completed"}'
    """
    try:
        data = json.loads(msg.data)
        task_id = str(data.get("task_id", ""))
        action = data.get("action", "")
        status = data.get("status", "completed")
        
        print(f"[ROS Subscriber] Received task update: task_id={task_id}, action={action}, status={status}")
        
        # Map action to step index using action mapper
        step_index = get_step_index_for_action(task_id, action)
        
        # Emit WebSocket event to all connected clients
        if socketio_instance:
            event_data = {
                'taskId': task_id,
                'action': action,
                'status': status
            }
            # Include stepIndex if mapping was found
            if step_index is not None:
                event_data['stepIndex'] = step_index
                print(f"[ROS Subscriber] Mapped action '{action}' to step index {step_index} for task {task_id}")
            
            socketio_instance.emit('task_step_update', event_data)
            print(f"[ROS Subscriber] Emitted WebSocket event for task {task_id}, action {action}")
        else:
            print("[ROS Subscriber] Warning: SocketIO instance not set, cannot emit event")
            
    except json.JSONDecodeError as e:
        print(f"[ROS Subscriber] Error parsing JSON: {e}")
    except Exception as e:
        print(f"[ROS Subscriber] Error in callback: {e}")

def action_complete_callback(msg):
    """
    Callback for /robot/action_complete topic.
    Expected message format: JSON string with task_id, action
    Example: '{"task_id": "1", "action": "place_cup"}'
    """
    try:
        data = json.loads(msg.data)
        task_id = str(data.get("task_id", ""))
        action = data.get("action", "")
        
        print(f"[ROS Subscriber] Received action complete: task_id={task_id}, action={action}")
        
        # Map action to step index using action mapper
        step_index = get_step_index_for_action(task_id, action)
        
        # Emit WebSocket event with status "completed"
        if socketio_instance:
            event_data = {
                'taskId': task_id,
                'action': action,
                'status': 'completed'
            }
            # Include stepIndex if mapping was found
            if step_index is not None:
                event_data['stepIndex'] = step_index
                print(f"[ROS Subscriber] Mapped action '{action}' to step index {step_index} for task {task_id}")
            
            socketio_instance.emit('task_step_update', event_data)
            print(f"[ROS Subscriber] Emitted WebSocket event for completed action {action}")
        else:
            print("[ROS Subscriber] Warning: SocketIO instance not set, cannot emit event")
            
    except json.JSONDecodeError as e:
        print(f"[ROS Subscriber] Error parsing JSON: {e}")
    except Exception as e:
        print(f"[ROS Subscriber] Error in callback: {e}")

def task_progress_callback(msg):
    """
    Callback for /task_progress topic (published by MoveIt2).
    Expected message format: JSON string with task, current_step, total_steps, description, status, progress_percent
    Example: '{"task": "prepare_medicine", "current_step": 2, "total_steps": 4, "description": "Placing plate", "status": "in-progress", "progress_percent": 50}'
    
    This ensures that:
    - When a step completes, it remains highlighted (status: "completed")
    - When a new step starts, previous steps are marked as completed
    - Progress bar updates correctly (25% per completed step for 4 steps)
    """
    try:
        progress_data = json.loads(msg.data)
        task_name = progress_data.get("task", "")
        current_step = progress_data.get("current_step", 0)
        status = progress_data.get("status", "in-progress")
        total_steps = progress_data.get("total_steps", 0)
        progress_percent = progress_data.get("progress_percent", 0)
        
        # Map task name (from MoveIt2) to task ID (mobile app)
        # Based on action_mapper.py: Task 1 = Set up table, Task 2 = Prepare medicine, Task 3 = Organize books
        task_name_to_id = {
            "prepare_medicine": "2",  # Task 2 = Prepare medicine
            "set_up_table": "1",      # Task 1 = Set up the table
            "organize_books": "3"     # Task 3 = Organize books
        }
        
        task_id = task_name_to_id.get(task_name)
        if not task_id:
            print(f"[ROS Subscriber] Unknown task name: {task_name}")
            return
        
        print(f"[ROS Subscriber] Received task progress: task={task_name} (id={task_id}), step={current_step}/{total_steps}, status={status}, progress={progress_percent}%")
        
        if not socketio_instance:
            print("[ROS Subscriber] Warning: SocketIO instance not set, cannot emit event")
            return
        
        # Convert current_step from 1-indexed to 0-indexed
        step_index = current_step - 1 if current_step > 0 else 0
        
        # Calculate progress percent based on completed steps if not provided
        # Each completed step = 100/total_steps percent
        if progress_percent == 0 and total_steps > 0:
            if status == "completed":
                # If current step is completed, count it as completed
                completed_steps = current_step
            elif status == "in-progress":
                # If current step is in-progress, previous steps are completed
                completed_steps = current_step - 1
            else:
                completed_steps = max(0, current_step - 1)
            progress_percent = int((completed_steps / total_steps) * 100) if total_steps > 0 else 0
        
        # When a new step starts (status = "in-progress" and current_step > 1),
        # we need to mark the previous step as completed
        if status == "in-progress" and current_step > 1:
            previous_step_index = step_index - 1
            # Emit completion event for previous step
            previous_event_data = {
                'taskId': task_id,
                'stepIndex': previous_step_index,
                'status': 'completed',
                'description': progress_data.get("description", ""),
                'progressPercent': int(((current_step - 1) / total_steps) * 100) if total_steps > 0 else 0
            }
            socketio_instance.emit('task_step_update', previous_event_data)
            print(f"[ROS Subscriber] Emitted completion event for previous step {current_step - 1} (stepIndex={previous_step_index})")
        
        # Emit event for current step
        event_data = {
            'taskId': task_id,
            'stepIndex': step_index,
            'status': status,
            'description': progress_data.get("description", ""),
            'progressPercent': progress_percent
        }
        
        socketio_instance.emit('task_step_update', event_data)
        print(f"[ROS Subscriber] Emitted WebSocket event for task {task_id}, step {current_step} (stepIndex={step_index}), status={status}, progress={progress_percent}%")
            
    except json.JSONDecodeError as e:
        print(f"[ROS Subscriber] Error parsing JSON: {e}")
    except Exception as e:
        print(f"[ROS Subscriber] Error in task_progress callback: {e}")

def start_ros_subscriber():
    """Initialize ROS node and start subscribing to task status topics."""
    try:
        # Only initialize if not already initialized
        if not rospy.core.is_initialized():
            rospy.init_node("flask_task_subscriber", anonymous=True)
            print("[ROS Subscriber] ROS node initialized")
        
        # Subscribe to task status topic
        rospy.Subscriber("/robot/task_status", String, task_status_callback)
        print("[ROS Subscriber] Subscribed to /robot/task_status")
        
        # Subscribe to action complete topic (alternative)
        rospy.Subscriber("/robot/action_complete", String, action_complete_callback)
        print("[ROS Subscriber] Subscribed to /robot/action_complete")
        
        # Subscribe to task progress topic (published by MoveIt2)
        rospy.Subscriber("/task_progress", String, task_progress_callback)
        print("[ROS Subscriber] Subscribed to /task_progress")
        
        # Start ROS spinner in a separate thread
        def spin_ros():
            rospy.spin()
        
        spinner_thread = threading.Thread(target=spin_ros, daemon=True)
        spinner_thread.start()
        print("[ROS Subscriber] ROS spinner started in background thread")
        
    except Exception as e:
        print(f"[ROS Subscriber] Error starting ROS subscriber: {e}")
        print("[ROS Subscriber] Continuing without ROS connection (for development)")


