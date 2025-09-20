from flask import Blueprint, Response, jsonify, request
import cv2
from services.camera_stream import camera
import base64
import os

camera_bp = Blueprint('camera', __name__)

def generate_frames():
    """Generate video frames for streaming"""
    while True:
        try:
            # Get color frame from camera
            frame = camera.get_color_frame()
            if frame is None:
                continue
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
            
            # Convert to bytes
            frame_bytes = buffer.tobytes()
            
            # Yield frame in MJPEG format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                   
        except Exception as e:
            print(f"Error generating frame: {e}")
            break

def generate_depth_frames():
    """Generate depth frames for streaming"""
    while True:
        try:
            # Get depth colormap from camera
            depth_frame = camera.get_depth_colormap()
            if depth_frame is None:
                continue
            
            # Encode frame as JPEG
            ret, buffer = cv2.imencode('.jpg', depth_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
            
            # Convert to bytes
            frame_bytes = buffer.tobytes()
            
            # Yield frame in MJPEG format
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                   
        except Exception as e:
            print(f"Error generating depth frame: {e}")
            break

@camera_bp.route('/stream')
def video_stream():
    """Stream color video from RealSense camera"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@camera_bp.route('/depth-stream')
def depth_stream():
    """Stream depth video from RealSense camera"""
    return Response(generate_depth_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@camera_bp.route('/frame')
def get_frame():
    """Get a single color frame as base64 encoded image"""
    try:
        frame = camera.get_color_frame()
        if frame is None:
            return jsonify({"error": "No frame available"}), 404
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            return jsonify({"error": "Failed to encode frame"}), 500
        
        # Convert to base64
        frame_base64 = base64.b64encode(buffer).decode('utf-8')
        
        return jsonify({
            "frame": frame_base64,
            "format": "jpeg",
            "timestamp": camera.last_frame_time
        })
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@camera_bp.route('/depth-frame')
def get_depth_frame():
    """Get a single depth frame as base64 encoded image"""
    try:
        depth_frame = camera.get_depth_colormap()
        if depth_frame is None:
            return jsonify({"error": "No depth frame available"}), 404
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', depth_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            return jsonify({"error": "Failed to encode depth frame"}), 500
        
        # Convert to base64
        frame_base64 = base64.b64encode(buffer).decode('utf-8')
        
        return jsonify({
            "frame": frame_base64,
            "format": "jpeg",
            "type": "depth_colormap",
            "timestamp": camera.last_frame_time
        })
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@camera_bp.route('/viewer')
def stream_viewer():
    """Serve the stream viewer HTML page"""
    try:
        # Read the HTML file
        html_path = os.path.join(os.path.dirname(__file__), '..', 'stream_viewer.html')
        with open(html_path, 'r', encoding='utf-8') as f:
            html_content = f.read()
        
        # Replace the stream URL with the actual endpoint
        stream_url = request.url_root + 'camera/stream'
        html_content = html_content.replace('/camera/stream', stream_url)
        
        return html_content
    except Exception as e:
        return f"Error loading viewer: {str(e)}", 500
