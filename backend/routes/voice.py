from flask import Blueprint, request, jsonify
from services.voice_parsers import process_voice
import os
import requests

voice_bp = Blueprint("voice", __name__)


@voice_bp.route("/", methods=["POST"])
def voice_command():
    data = request.get_json()
    command = data.get("command")
    result = process_voice(command)
    return jsonify(result)


@voice_bp.route("/audio", methods=["POST"])
def voice_audio():
    """
    Accepts a multipart/form-data file upload with field name 'file'.
    Forwards the audio file to OpenAI Whisper (if OPENAI_API_KEY is set) to
    transcribe, then passes the transcribed text to process_voice and
    returns the result.
    """
    if 'file' not in request.files:
        return jsonify({"error": "no file provided"}), 400

    f = request.files['file']
    # Save to a temporary location
    tmp_path = f"/tmp/{f.filename}"
    f.save(tmp_path)
    print(f"[voice_audio] Received file={f.filename} saved_to={tmp_path}")

    openai_key = os.environ.get('OPENAI_API_KEY')
    transcript = None

    if openai_key:
        try:
            with open(tmp_path, 'rb') as fh:
                files = {"file": (f.filename, fh)}
                data = {"model": "whisper-1"}
                headers = {"Authorization": f"Bearer {openai_key}"}
                resp = requests.post('https://api.openai.com/v1/audio/transcriptions', headers=headers, files=files, data=data)
                resp.raise_for_status()
                j = resp.json()
                transcript = j.get('text')
        except Exception as e:
            print(f"[voice_audio] transcription failed: {e}")
            return jsonify({"error": "transcription_failed", "detail": str(e)}), 500
    else:
        return jsonify({"error": "OPENAI_API_KEY not set on backend"}), 500

    if not transcript:
        return jsonify({"error": "no_transcript"}), 500

    result = process_voice(transcript)
    print(f"[voice_audio] transcript={transcript} result={result}")
    return jsonify({"transcript": transcript, "result": result})
