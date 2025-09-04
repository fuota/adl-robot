# ros_nodes/task_executor.py
import rospy
from std_msgs.msg import String
from services.task_library import get_actions_for_task
from kinova_api import KinovaArm  # placeholder for Kinova SDK wrapper
import requests

def task_callback(msg):
    task_id = msg.data
    rospy.loginfo(f"Received task: {task_id}")
    actions = get_actions_for_task(task_id)

    # Initialize Kinova arm
    arm = KinovaArm()

    for action in actions:
        rospy.loginfo(f"Executing {action}")
        success = execute_action(arm, action)

        # Report back to Flask backend
        requests.post("http://localhost:5000/task/update", json={
            "action": action,
            "status": "done" if success else "error"
        })

        if not success:
            break

def execute_action(arm, action):
    """Map primitive actions to Kinova SDK calls"""
    if action == "grasp_cup":
        return arm.grasp("cup")
    elif action == "pour_milk":
        return arm.pour("milk_carton")
    elif action == "pour_cereal":
        return arm.pour("cereal_box")
    elif action == "stir":
        return arm.stir("spoon")
    else:
        rospy.logwarn(f"Unknown action: {action}")
        return False

if __name__ == "__main__":
    rospy.init_node("task_executor")
    rospy.Subscriber("/robot/task", String, task_callback)
    rospy.spin()
