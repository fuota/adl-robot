from flask import Blueprint, request, jsonify
from services.voice_parser import process_voice

voice_bp = Blueprint("voice", __name__)

@voice_bp.route("/", methods=["POST"])
def voice_command():
    data = request.get_json()
    command = data.get("command")
    result = process_voice(command)
    return jsonify(result)
