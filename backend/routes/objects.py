from flask import Blueprint, jsonify
from services.detection import detect_objects

objects_bp = Blueprint("objects", __name__)

@objects_bp.route("/", methods=["GET"])
def get_objects():
    results = detect_objects()
    return jsonify(results)
