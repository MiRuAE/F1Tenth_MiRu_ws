import cv2

pipeline = "v4l2src device=/dev/video0 ! videoconvert ! appsink"
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)  # GStreamer 파이프라인 사용

if not cap.isOpened():
    print("카메라 열기 실패!")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임 읽기 실패!")
        break

    cv2.imshow('GStreamer Camera Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

