#!/usr/bin/env python3
#
# Logitech Brio 4K Camera Interface
#
# Modified from JetsonHacks USB Camera example
# MIT License
#

import sys
import cv2

class BrioCamera:
    def __init__(self, camera_id="/dev/video0", resolution="4K"):
        self.window_title = "Logitech Brio Camera"
        self.camera_id = camera_id
        self.resolution = resolution
        self.resolutions = {
            "4K": (4096, 2160, 30),    # 4K @ 30fps
            "1080p": (1920, 1080, 60), # 1080p @ 60fps
            "720p": (1280, 720, 90)    # 720p @ 90fps
        }

    def configure_camera(self, cap):
        """Configure camera settings for Logitech Brio"""
        width, height, fps = self.resolutions.get(self.resolution, self.resolutions["1080p"])
        
        # Set video capture properties
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        # Additional Brio-specific settings
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # Enable autofocus
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Enable auto exposure
        
        # Verify settings
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"Camera configured with resolution: {actual_width}x{actual_height} @ {actual_fps}fps")

    def show_camera(self):
        """Initialize and display camera feed"""
        video_capture = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        
        if video_capture.isOpened():
            try:
                # Configure camera settings
                self.configure_camera(video_capture)
                
                # Create window
                window_handle = cv2.namedWindow(
                    self.window_title, cv2.WINDOW_AUTOSIZE)
                
                while True:
                    ret_val, frame = video_capture.read()
                    if not ret_val:
                        print("Error reading frame")
                        break

                    # Check if window is still open
                    if cv2.getWindowProperty(self.window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                        cv2.imshow(self.window_title, frame)
                    else:
                        break

                    keyCode = cv2.waitKey(10) & 0xFF
                    # Stop the program on the ESC key or 'q'
                    if keyCode == 27 or keyCode == ord('q'):
                        break

            finally:
                video_capture.release()
                cv2.destroyAllWindows()
        else:
            print("Unable to open camera")
            print("Please check if the camera is connected and the device path is correct")

def main():
    # You can change the resolution to "4K", "1080p", or "720p"
    camera = BrioCamera(resolution="720p")
    camera.show_camera()

if __name__ == "__main__":
    main()
