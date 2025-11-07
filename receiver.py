from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO, emit
import cv2
import numpy as np
import base64

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# Current frame from drone
latest_frame = None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        global latest_frame
        while True:
            if latest_frame is not None:
                ret, buffer = cv2.imencode('.jpg', latest_frame)
                frame_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Receive video frames from drone ---
@app.route('/video_frame', methods=['POST'])
def video_frame():
    global latest_frame
    data = request.get_json()
    if not data or "frame" not in data:
        return jsonify({"error": "no frame"}), 400

    # Decode base64 frame
    frame_data = base64.b64decode(data["frame"])
    np_arr = np.frombuffer(frame_data, np.uint8)
    latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return jsonify({"status": "ok"})

# --- Receive alerts ---
@app.route('/alert', methods=['POST'])
def receive_alert():
    alert_data = request.get_json()
    socketio.emit('new_alert', alert_data)
    print("Received alert:", alert_data)
    return jsonify({"status": "ok"})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
