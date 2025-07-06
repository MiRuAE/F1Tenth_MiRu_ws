
from __future__ import annotations

import contextlib
from pathlib import Path
from typing import Optional, List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

with contextlib.suppress(ImportError):
    from ultralytics import YOLO  # type: ignore
import torch
import cv2

# ──────────────────────────────────────────────────────────────
# LaneCenterTracker
# ──────────────────────────────────────────────────────────────
class LaneCenterTracker:
    """차선 마스크 → polyfit 곡선 & 차선 중심 추정 + 시각화 데이터 보관."""

    def __init__(self, lane_width_px: float, ema_alpha: Optional[float] = 0.5, poly_deg: int = 2):
        self.lane_width_px = lane_width_px
        self.ema_alpha = ema_alpha
        self.poly_deg = poly_deg
        self.center_px: Optional[float] = None
        self.left_x_prev: Optional[float] = None
        self.right_x_prev: Optional[float] = None
        self.poly_coeffs: List[Optional[np.ndarray]] = []  # [left, right]
        self.last_masks_full: List[np.ndarray] = []        # full‑size binary masks
        self.last_confs: List[float] = []

    # ----------------------------------------------------------
    @staticmethod
    def _mask_bottom_x(mask: np.ndarray) -> Optional[float]:
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return None
        y_max = ys.max()
        xs_at_bottom = xs[ys == y_max]
        return float(xs_at_bottom.mean()) if xs_at_bottom.size else None

    # ----------------------------------------------------------
    @staticmethod
    def _fit_poly(mask: np.ndarray, deg: int, sample_step: int = 5) -> Optional[np.ndarray]:
        ys, xs = np.nonzero(mask)
        if xs.size < deg + 1:
            return None
        ys_s = ys[::sample_step]
        xs_s = xs[::sample_step]
        try:
            coeffs = np.polyfit(ys_s, xs_s, deg)
            return coeffs
        except Exception:
            return None
    # ----------------------------------------------------------
    @staticmethod
    def _mask_slope(mask: np.ndarray) -> float | None:
        """binary mask → k = dx/dy (직선 기울기)."""
        lines = cv2.HoughLinesP(mask.astype(np.uint8) * 255, 1, np.pi / 180,
                                threshold=30, minLineLength=20, maxLineGap=15)
        if lines is None:
            return None
        x1, y1, x2, y2 = max(
            lines,
            key=lambda l: np.hypot(l[0, 2] - l[0, 0], l[0, 3] - l[0, 1])
        )[0]
        dy = abs(y2 - y1)
        if dy == 0:
            return None
        return (x2 - x1) / dy
    # ----------------------------------------------------------
    def _ema(self, new: float) -> float:
        if self.center_px is None or self.ema_alpha is None:
            self.center_px = new
        else:
            a = self.ema_alpha
            self.center_px = a * new + (1 - a) * self.center_px
        return self.center_px

    # ----------------------------------------------------------
    def update(self, masks_full: List[np.ndarray], confs: np.ndarray, roi: slice, img_width: int) -> Optional[float]:
        """차선 중심 & 시각화용 데이터 계산."""
        self.last_masks_full = masks_full  # 시각화용 보관
        if self.center_px is None:
            self.center_px = img_width / 2.0

        # ROI 추출 & 마스크 크기 비교
        masks_roi = [m[roi] for m in masks_full]
        sizes = np.array([m.sum() for m in masks_roi])
        
        if sizes.max() == 0:
            self.poly_coeffs = []
            return self.center_px

        order = sizes.argsort()[::-1]
        confs = confs[order]

        # ── 두 차선 모두 보이는 경우 ────────────────────────
        if len(order) >= 2:
            idx1, idx2 = order[:2]
            x1 = self._mask_bottom_x(masks_roi[idx1])
            x2 = self._mask_bottom_x(masks_roi[idx2])
            if x1 is not None and x2 is not None:
                # 왼/오른 결정 (x 작은 쪽이 왼쪽)
                if x1 < x2:
                    left_idx, right_idx = idx1, idx2
                else:
                    left_idx, right_idx = idx2, idx1
                    x1, x2 = x2, x1
                self.last_masks_full = [masks_full[left_idx], masks_full[right_idx]]
                self.last_confs      = [confs[0], confs[1]]
                self.poly_coeffs = [
                    self._fit_poly(masks_roi[left_idx], self.poly_deg),
                    self._fit_poly(masks_roi[right_idx], self.poly_deg),
                ]
                return self._ema((x1 + x2) / 2.0)

        # ── 한쪽 차선만 보일 때 ──────────────────────────────
        idx = order[0]
        xb = self._mask_bottom_x(masks_roi[idx])
        if xb is None:
            return self.center_px

        self.last_masks_full = [masks_full[idx]]
        self.last_confs      = [confs[0]]
        self.poly_coeffs = [self._fit_poly(masks_roi[idx], self.poly_deg)]

        cand_left = xb + self.lane_width_px / 2.0
        cand_right = xb - self.lane_width_px / 2.0

        if self.center_px is not None:
            if abs(cand_left - self.center_px) < abs(cand_right - self.center_px):
                self.left_x_prev = xb
                return self._ema(cand_left)
            else:
                self.right_x_prev = xb
                return self._ema(cand_right)

        # fallback: 과거 좌/우 위치
        dl = abs(xb - self.left_x_prev) if self.left_x_prev is not None else np.inf
        dr = abs(xb - self.right_x_prev) if self.right_x_prev is not None else np.inf
        if dl < dr:
            self.left_x_prev = xb
            return self._ema(cand_left)
        else:
            self.right_x_prev = xb
            return self._ema(cand_right)

    # ----------------------------------------------------------
    def draw_visuals(self, frame: np.ndarray, roi: slice):
        """마스크·polyfit·중심점을 오버레이한 시각화 프레임 반환."""
        H, W = frame.shape[:2]
        vis = frame.copy()

        # ── 마스크 컬러 오버레이 ────────────────────────────
        mask_colors = [
            (255, 0, 0),      # 파랑(왼)
            (0, 255, 255),    # 노랑(오른)
            (0, 255, 0),      # 초록(추가)
            (255, 0, 255),    # 마젠타(추가)
        ]
        overlay = vis.copy()
        for idx, m in enumerate(self.last_masks_full):
            color = mask_colors[idx % len(mask_colors)]
            overlay[m.astype(bool)] = color        # ← 실제 색 채우기
        vis = cv2.addWeighted(vis, 0.5, overlay, 0.5, 0)
        


        # ── polyfit 곡선 ───────────────────────────────────
        colors_poly = [(255, 0, 0), (0, 255, 255)]  # 왼/오
        y_vals_local = np.arange(0, roi.stop - roi.start)
        for idx, coeffs in enumerate(self.poly_coeffs):
            if coeffs is None:
                continue
            xs = np.polyval(coeffs, y_vals_local).astype(np.int32)
            pts = np.stack([xs, y_vals_local + roi.start], axis=1)
            cv2.polylines(vis, [pts], False, colors_poly[idx % len(colors_poly)], 2)

        # ── 중심점 ─────────────────────────────────────────
        if self.center_px is not None:
            cx = int(round(self.center_px))
            cy = roi.stop - 5
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(vis, f"{cx}px", (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        for idx, m in enumerate(self.last_masks_full):
            if idx < len(self.last_confs):
                ys, xs = np.nonzero(m)
                if xs.size:
                    conf_txt = f"{self.last_confs[idx]:.2f}"
                    x_b = int(xs.mean())
                    y_b = int(ys.max()) - 5
                    color = mask_colors[idx % len(mask_colors)]
                    cv2.putText(vis, conf_txt, (x_b - 15, y_b),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        return vis


# ──────────────────────────────────────────────────────────────
# LaneNode (Camera Capture)
# ──────────────────────────────────────────────────────────────
class LaneNode(Node):
    """YOLO-Seg → 마스크 & 차선 중심 퍼블리셔 (직접 카메라 스트림 + 시각화)."""

    def __init__(self):
        super().__init__("lane_node_cam")

        # ── ROS parameters ────────────────────────────────────
        self.declare_parameters(
            namespace="",
            parameters=[
                ("model_path", "/home/shchon11/programming/lane_ws/src/lane_pkg/model/best.pt"),
                ("camera_index", 0),
                ("frame_width", 640),
                ("frame_height", 360),
                ("fps", 30),
                ("conf_threshold", 0.8),
                ("mask_topic", "masks"),
                ("lane_center_topic", "lane_center"),
                ("lane_width_px", 700.0),
                ("ema_alpha", 0.5),
                ("roi_ratio", 0.5),
                ("mask_thr", 0.5),
                ("visualize", True),
            ],
        )

        gp = lambda n: self.get_parameter(n).get_parameter_value()
        self.cfg = {
            "model_path": gp("model_path").string_value,
            "camera_index": gp("camera_index").integer_value,
            "frame_width": gp("frame_width").integer_value,
            "frame_height": gp("frame_height").integer_value,
            "fps": gp("fps").integer_value,
            "conf_threshold": gp("conf_threshold").double_value,
            "mask_topic": gp("mask_topic").string_value,
            "lane_center_topic": gp("lane_center_topic").string_value,
            "lane_width_px": gp("lane_width_px").double_value,
            "ema_alpha": gp("ema_alpha").double_value,
            "roi_ratio": gp("roi_ratio").double_value,
            "mask_thr": gp("mask_thr").double_value,
            "visualize": gp("visualize").bool_value,
        }

        # Model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = None
        try:
            if Path(self.cfg["model_path"]).is_file():
                self.model = YOLO(
                    self.cfg["model_path"], task="segment"
                ).to(self.device)  # type: ignore
                self.model.fuse()
                self.get_logger().info(
                    f"Loaded model {self.cfg['model_path']} on {self.device}"
                )
            else:
                self.get_logger().error(f"Model not found: {self.cfg['model_path']}")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

        # Camera
        self.cap = cv2.VideoCapture(self.cfg["camera_index"], cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().fatal("Cannot open camera — exiting.")
            raise RuntimeError("Camera open failed")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["frame_width"])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["frame_height"])
        self.cap.set(cv2.CAP_PROP_FPS, self.cfg["fps"])

        w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps_real = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera opened ({w}×{h} @ {fps_real} fps)")

        # ROS publishers
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.bridge = CvBridge()
        self.mask_pub = self.create_publisher(Image, self.cfg["mask_topic"], qos_sensor)
        self.center_pub = self.create_publisher(Float32, self.cfg["lane_center_topic"], 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.create_subscription(String, "current_mission", self.mission_callback, 10)
        # Tracker
        self.tracker = LaneCenterTracker(
            lane_width_px=self.cfg["lane_width_px"],
            ema_alpha=self.cfg["ema_alpha"],
            poly_deg=2,
        )

        # Main loop
        self.create_timer(1.0 / self.cfg["fps"], self.timer_cb)
        
        # PID parameter
        self.declare_parameters(
            namespace="",
            parameters=[
                ("Kp", 0.004),
                ("Ki", 0.0),
                ("Kd", 0.001),
            ],
        )
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value

        
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.is_active = False
    # ------------------------------------------------------        
    def mission_callback(self, msg: String):
        self.is_active = (msg.data == "MISSION_A")
        state = "ON" if self.is_active else "OFF"
        self.get_logger().info(f"Lane-following mission → {state}")

    # ------------------------------------------------------
    @staticmethod
    def speed_control(slope_rad: float) -> float:
        k, x0 = 5.0, np.pi / 4
        sig   = 1.0 / (1 + np.exp(k * (abs(slope_rad) - x0)))
        return 2.0 - sig 
    # ------------------------------------------------------
    @torch.inference_mode()
    def timer_cb(self):
        if not self.is_active:
            return
        ret, frame = self.cap.read()
        if not ret or self.model is None:
            return

        res = self.model.predict(
            frame,
            conf=self.cfg["conf_threshold"],
            device=self.device,
            verbose=False,
        )[0]

        H, W = res.orig_shape[:2]
        roi = slice(int(H * self.cfg["roi_ratio"]), H)

        if res.masks is None or res.masks.data.shape[0] == 0:
            return

        masks_bin  = (torch.sigmoid(res.masks.data) > self.cfg["mask_thr"]).cpu().numpy()
        masks_full = [cv2.resize(m.astype(np.uint8), (W, H), cv2.INTER_NEAREST) for m in masks_bin]
        confs_full = res.boxes.conf.cpu().numpy()

        center = self.tracker.update(masks_full, confs_full, roi, img_width=W)
        if center is not None:
            self.center_pub.publish(Float32(data=float(center)))

        # mask publish
        composite = np.zeros((H, W), dtype=np.uint8)
        for m in masks_full:
            composite[m.astype(bool)] = 255
        mask_msg = self.bridge.cv2_to_imgmsg(composite, encoding="mono8")
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(mask_msg)
        
        
        # steer control
        error       = center - (W / 2.0)
        self.integral += error
        derivative  = error - self.prev_error
        steer_cmd   = -(self.Kp * error +
                        self.Ki * self.integral +
                        self.Kd * derivative) / 3.0
        self.prev_error = error

        
        # speed control
        left_k  = self.tracker._mask_slope(self.tracker.last_masks_full[0]) if self.tracker.last_masks_full else None
        right_k = (self.tracker._mask_slope(self.tracker.last_masks_full[1])
                   if len(self.tracker.last_masks_full) == 2 else None)
        if left_k is not None and right_k is not None:
            slope_rad = np.arctan(2 / (left_k + right_k))
        elif left_k is not None:
            slope_rad = np.arctan(abs(left_k))
        elif right_k is not None:
            slope_rad = np.arctan(abs(right_k))
        else:
            slope_rad = 0.0
        speed = self.speed_control(slope_rad)
        
        # Ackermann Message publish
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = float(steer_cmd)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)
        
        # visualization
        if self.cfg["visualize"]:
            vis = self.tracker.draw_visuals(frame, roi)
            cv2.imshow("lane_vis", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rclpy.shutdown()

    # ------------------------------------------------------
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()



# main

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        torch.cuda.empty_cache()
        if rclpy.ok():
            rclpy.shutdown()


