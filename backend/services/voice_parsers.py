def process_voice(command):
    """
    Parse voice command and return task mapping.
    Matches voice commands to task IDs based on keywords.
    """
    if not command:
        return {"status": "error", "message": "No command provided"}
    
    command_lower = command.lower()
    
    # Map voice commands to task IDs
    if "prepare medicine" in command_lower or "prepare the medicine" in command_lower:
        return {"status": "success", "task_id": "1", "command": command}
    elif "set up table" in command_lower or "set the table" in command_lower or "setup table" in command_lower:
        return {"status": "success", "task_id": "2", "command": command}
    elif "organize books" in command_lower or "organise books" in command_lower:
        return {"status": "success", "task_id": "3", "command": command}
    else:
        return {"status": "unknown", "command": command, "message": "Command not recognized"}
