from services.task_library import get_actions_for_task

task_state = {
    "task_id": None,
    "status": "idle",
    "current_action": None,
    "actions": [],
    "progress": 0,
    "error": None
}

def start_task(task_id):
    global task_state
    actions = get_actions_for_task(task_id)
    task_state = {
        "task_id": task_id,
        "status": "in_progress",
        "current_action": None,
        "actions": [{"name": a, "status": "pending"} for a in actions],
        "progress": 0,
        "error": None
    }
    return task_state

def update_action(action_name, status):
    global task_state
    task_state["current_action"] = action_name
    for a in task_state["actions"]:
        if a["name"] == action_name:
            a["status"] = status
    done = sum(1 for a in task_state["actions"] if a["status"] == "done")
    if task_state["actions"]:
        task_state["progress"] = int(100 * done / len(task_state["actions"]))
    if all(a["status"] == "done" for a in task_state["actions"]):
        task_state["status"] = "done"
    return task_state

def get_task_state():
    return task_state
