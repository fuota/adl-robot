from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO, emit
from routes.objects import objects_bp
from routes.tasks import tasks_bp, set_socketio_instance
from routes.voice import voice_bp
from routes.camera import camera_bp
from services.ros_subscriber import set_socketio, start_ros_subscriber

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

# Initialize SocketIO for WebSocket support
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading', logger=True, engineio_logger=True)

# Set SocketIO instance for ROS subscriber and tasks
set_socketio(socketio)
set_socketio_instance(socketio)

# Register Blueprints
app.register_blueprint(objects_bp, url_prefix="/objects")
app.register_blueprint(tasks_bp, url_prefix="/task")
app.register_blueprint(voice_bp, url_prefix="/voice")
app.register_blueprint(camera_bp, url_prefix="/camera")

# WebSocket event handlers
@socketio.on('connect')
def handle_connect():
    print(f'[WebSocket] Client connected')
    emit('connected', {'message': 'Connected to task update server'})

@socketio.on('disconnect')
def handle_disconnect():
    print(f'[WebSocket] Client disconnected')

@app.route("/")
def home():
    return {"message": "Robotic Arm Backend Running"}

if __name__ == "__main__":
    # Start ROS subscriber in background
    try:
        start_ros_subscriber()
    except Exception as e:
        print(f"[App] Warning: Could not start ROS subscriber: {e}")
        print("[App] Continuing without ROS connection (for development)")
    
    # Run app with SocketIO
    socketio.run(app, host="0.0.0.0", port=5000, debug=True, allow_unsafe_werkzeug=True)
