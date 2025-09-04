from flask import Flask
from routes.objects import objects_bp
from routes.tasks import tasks_bp
from routes.voice import voice_bp

app = Flask(__name__)

# Register Blueprints
app.register_blueprint(objects_bp, url_prefix="/objects")
app.register_blueprint(tasks_bp, url_prefix="/task")
app.register_blueprint(voice_bp, url_prefix="/voice")

@app.route("/")
def home():
    return {"message": "Robotic Arm Backend Running"}

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
