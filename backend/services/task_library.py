TASKS = {
    "make_cereal": ["grasp_cup", "pour_milk", "pour_cereal", "stir"],
    "place_book": ["grasp_book", "move_to_shelf", "release_book"],
    "set_table": ["place_plate", "place_cup", "place_utensils"]
}

def get_actions_for_task(task_id):
    return TASKS.get(task_id, [])
