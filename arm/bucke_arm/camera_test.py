import cv2
from flask import Flask, Response

app = Flask(__name__)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

def frames():
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        ok, jpg = cv2.imencode(".jpg", frame)
        if ok:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + jpg.tobytes() + b"\r\n")

@app.route("/")
def video():
    return Response(
        frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

app.run(host="0.0.0.0", port=5000, threaded=True)
