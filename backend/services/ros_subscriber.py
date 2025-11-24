"""
ROS Subscriber Service
Listens to ROS topics for task completion updates and forwards them to the WebSocket server.
"""
import rospy
from std_msgs.msg import String
import json
import threading

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
        
        # Emit WebSocket event to all connected clients
        if socketio_instance:
            socketio_instance.emit('task_step_update', {
                'taskId': task_id,
                'action': action,
                'status': status
            })
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
        
        # Emit WebSocket event with status "completed"
        if socketio_instance:
            socketio_instance.emit('task_step_update', {
                'taskId': task_id,
                'action': action,
                'status': 'completed'
            })
            print(f"[ROS Subscriber] Emitted WebSocket event for completed action {action}")
        else:
            print("[ROS Subscriber] Warning: SocketIO instance not set, cannot emit event")
            
    except json.JSONDecodeError as e:
        print(f"[ROS Subscriber] Error parsing JSON: {e}")
    except Exception as e:
        print(f"[ROS Subscriber] Error in callback: {e}")

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
        
        # Start ROS spinner in a separate thread
        def spin_ros():
            rospy.spin()
        
        spinner_thread = threading.Thread(target=spin_ros, daemon=True)
        spinner_thread.start()
        print("[ROS Subscriber] ROS spinner started in background thread")
        
    except Exception as e:
        print(f"[ROS Subscriber] Error starting ROS subscriber: {e}")
        print("[ROS Subscriber] Continuing without ROS connection (for development)")


