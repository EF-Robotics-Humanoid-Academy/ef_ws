import cv2
import numpy as np

# Open Berxel RGB camera (index 0 is typical; change if needed)
cap = cv2.VideoCapture(0, cv2.CAP_MSMF)

if not cap.isOpened():
    raise RuntimeError("Failed to open Berxel RGB camera")

# Optional: set resolution and FPS
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

try:
    while True:
        ret, color = cap.read()
        if not ret or color is None:
            print("Failed to read frame")
            continue

        cv2.imshow("Berxel RGB", color)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
