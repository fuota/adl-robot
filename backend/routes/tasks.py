from flask import Blueprint, request, jsonify
from services.task_state import start_task, update_action, get_task_state
from services.ros_bridge import send_task_to_ros

tasks_bp = Blueprint("tasks", __name__)

@tasks_bp.route("/", methods=["POST"])
def start():
    """POST /task
        { "task_id": "make_cereal" }
    """
    data = request.get_json()
    task_id = data.get("task_id")
    state = start_task(task_id)   # Initialize state
    send_task_to_ros(task_id)     # Notify ROS
    return jsonify(state)

@tasks_bp.route("/status", methods=["GET"])
def status():
    return jsonify(get_task_state())

@tasks_bp.route("/update", methods=["POST"])
def update():
    """
        POST /task/update
        { "action": "grasp_cup", "status": "done" }
    """
    # This should be called by ROS (or your ROS bridge)
    data = request.get_json()
    state = update_action(data["action"], data["status"])
    return jsonify(state)
