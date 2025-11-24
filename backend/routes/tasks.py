from flask import Blueprint, request, jsonify
from flask_socketio import emit
from services.task_state import start_task, update_action, get_task_state
from services.ros_bridge import send_task_to_ros
from services.action_mapper import get_step_index_for_action
import socketio as sio_client

tasks_bp = Blueprint("tasks", __name__)

# Get SocketIO instance from app context (will be set by app.py)
_socketio = None

def set_socketio_instance(socketio):
    """Set the SocketIO instance for emitting events."""
    global _socketio
    _socketio = socketio

@tasks_bp.route("/", methods=["POST"])
def start():
    """POST /task
        { "task_id": "1" } or { "task_id": "make_cereal" }
    """
    data = request.get_json()
    task_id = data.get("task_id")
    state = start_task(task_id)   # Initialize state
    send_task_to_ros(task_id)     # Notify ROS
    
    # Emit WebSocket event to notify clients that task started
    if _socketio:
        _socketio.emit('task_started', {
            'taskId': str(task_id),
            'state': state
        })
    
    return jsonify(state)

@tasks_bp.route("/status", methods=["GET"])
def status():
    return jsonify(get_task_state())

@tasks_bp.route("/update", methods=["POST"])
def update():
    """
        POST /task/update
        { "task_id": "1", "action": "place_cup", "status": "completed" }
        or
        { "action": "grasp_cup", "status": "done" }
    """
    data = request.get_json()
    task_id = data.get("task_id")
    action = data.get("action", "")
    status = data.get("status", "done")
    
    # Update task state
    state = update_action(action, status)
    
    # Map action to step index and emit WebSocket event
    if task_id and _socketio:
        step_index = get_step_index_for_action(task_id, action)
        if step_index is not None:
            _socketio.emit('task_step_update', {
                'taskId': str(task_id),
                'stepIndex': step_index,
                'action': action,
                'status': 'completed' if status in ['done', 'completed'] else status
            })
            print(f"[Tasks] Emitted step update: task={task_id}, step={step_index}, action={action}")
    
    return jsonify(state)
