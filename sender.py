import cv2
import time
import requests
import base64
import json

SERVER_URL = "http://127.0.0.1:5000"

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- Send video frame to receiver ---
    _, buffer = cv2.imencode('.jpg', frame)
    frame_b64 = base64.b64encode(buffer).decode('utf-8')
    requests.post(f"{SERVER_URL}/video_frame", json={"frame": frame_b64})

    cv2.imshow('Drone View (press A for alert, Q to quit)', frame)
    key = cv2.waitKey(1) & 0xFF

    # Press 'A' to send test alert
    if key == ord('a'):
        alert = {
            "alert_id": f"test-{int(time.time())}",
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "detection": {
                "class": "person_in_water",
                "confidence": 0.91,
                "latitude": 48.8566,
                "longitude": 2.3522
            },
            "priority": "high"
        }
        requests.post(f"{SERVER_URL}/alert", json=alert)
        print("Sent alert:", json.dumps(alert, indent=2))

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
