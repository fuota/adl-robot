def run_task(task_name):
    # TODO: send predefined sequence to Kinova API
    return {"status": "started", "task": task_name}

def primitive_action(action, obj_id):
    # TODO: translate to Kinova motion commands
    return {"status": "executed", "action": action, "object_id": obj_id}
