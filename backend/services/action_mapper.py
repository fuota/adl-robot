"""
Maps ROS action names to mobile app step indices for each task.
This allows the backend to know which step to update when ROS completes an action.
"""

# Mapping: task_id -> { ros_action_name -> step_index }
ACTION_TO_STEP_MAP = {
    "1": {  # Set up the table
        "arrange_plate": 0,
        "place_plate": 0,
        "arrange_spoon": 1,
        "place_spoon": 1,
        "arrange_milk": 2,
        "place_milk": 2,
        "arrange_glass": 3,
        "place_glass": 3,
    },
    "2": {  # Prepare medicine
        "pour_water": 0,
        "place_water_bottle": 0,
        "place_cup": 1,
        "place_plate": 2,
        "place_medicine_bottle": 3,
        "pour_medicine": 3,
    },
    "3": {  # Organize books
        "pick_up_book": 0,
        "grasp_book": 0,
        "place_book": 1,
        "place_book_appropriately": 1,
        "pick_up_objects": 2,
        "grasp_objects": 2,
        "arrange_objects": 3,
        "arrange_neatly": 3,
    },
}

def get_step_index_for_action(task_id: str, action_name: str) -> int | None:
    """
    Get the step index for a given ROS action in a specific task.
    Returns None if the action doesn't map to any step.
    """
    task_map = ACTION_TO_STEP_MAP.get(str(task_id))
    if not task_map:
        return None
    
    # Try exact match first
    if action_name in task_map:
        return task_map[action_name]
    
    # Try partial match (e.g., "grasp_milk" matches "milk")
    action_lower = action_name.lower()
    for ros_action, step_idx in task_map.items():
        if ros_action.lower() in action_lower or action_lower in ros_action.lower():
            return step_idx
    
    return None

def get_all_actions_for_task(task_id: str) -> list[str]:
    """Get all ROS actions that map to steps for a given task."""
    task_map = ACTION_TO_STEP_MAP.get(str(task_id))
    if not task_map:
        return []
    return list(task_map.keys())


