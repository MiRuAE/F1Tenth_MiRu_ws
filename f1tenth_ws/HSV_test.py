import cv2
import numpy as np


# Parameter settings
camera_index = 2
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([180, 255, 100])


cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("wrong index")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("cannot read frame")
        break

   
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    # visualize
    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

