import cv2

def process_frame():
    camera_index = 0
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 180)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
    
    blockSize = 501
    C = -20
    gaus_blur_size = 5
    
    canny_inf = 50
    canny_sup = 150
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임 캡쳐에 실패했습니다!")
            break

        # 프레임 크기 가져오기
        height, width = frame.shape[:2]
        # ROI: 프레임의 아래쪽 절반 선택
        roi_frame = frame[height//2:height, 0:width]
        # 그레이스케일 변환
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        
	# 가우시안 블러
        blurred = cv2.GaussianBlur(gray, (gaus_blur_size, gaus_blur_size), 0)
        # 어댑티브 임계처리
        binary = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                       cv2.THRESH_BINARY, blockSize, C)

        

        # Canny Edge 검출
        edges = cv2.Canny(blurred, canny_inf, canny_sup)

        # 결과 출력
        cv2.imshow("Original Frame", frame)
        cv2.imshow("ROI Frame", roi_frame)
        cv2.imshow("binary", binary)
        cv2.imshow("Edges", edges)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    process_frame()
