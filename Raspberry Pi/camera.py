#!/usr/bin/env python3
"""
Person tracking with OpenCV on Raspberry Pi CM5.
Detects people, computes normalized offsets from frame center, and sends
those offsets to an ESP32 (e.g., to drive pan/tilt motors).

- Uses HOG person detector (CPU friendly on Pi).
- Offsets are normalized to [-1, 1] and sent as scaled integers over serial.
- If serial is unavailable, messages are printed to stdout for debugging.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - runtime fallback
    serial = None


@dataclass
class TrackerConfig:
    camera_index: int = 0
    frame_width: int = 640
    frame_height: int = 480
    serial_port: str = "/dev/serial0"  # CM5 UART alias
    serial_baudrate: int = 115_200
    detection_scale: float = 1.05
    detection_win_stride: Tuple[int, int] = (8, 8)
    detection_padding: Tuple[int, int] = (8, 8)
    min_confidence: float = 0.3  # HOG weights are not calibrated; used as relative threshold
    smoothing_alpha: float = 0.25  # EMA smoothing for offsets
    show_preview: bool = True


class ESP32Link:
    """Serial link to an ESP32 for sending tracking offsets."""

    def __init__(self, port: str, baudrate: int, fallback_to_stdout: bool = True) -> None:
        self.port = port
        self.baudrate = baudrate
        self.fallback_to_stdout = fallback_to_stdout
        self._ser: Optional["serial.Serial"] = None
        self._connect()

    def _connect(self) -> None:
        if serial is None:
            print("pyserial not installed; falling back to stdout logging.")
            return
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            time.sleep(2.0)  # Allow ESP32 to reset after opening serial
            print(f"Connected to ESP32 on {self.port} @ {self.baudrate} baud")
        except Exception as exc:  # pragma: no cover - runtime safety
            print(f"Could not open serial port {self.port}: {exc}\nFalling back to stdout logging.")
            self._ser = None

    def send_offset(self, dx_norm: float, dy_norm: float, detected: bool) -> None:
        """Send normalized offsets to the ESP32.

        Values are scaled to integers in thousandths to keep parsing simple on the MCU.
        Format: "TRACK,<dx_milli>,<dy_milli>,<detected>\n".
        """

        dx_milli = int(max(-1.0, min(1.0, dx_norm)) * 1000)
        dy_milli = int(max(-1.0, min(1.0, dy_norm)) * 1000)
        detected_flag = 1 if detected else 0
        message = f"TRACK,{dx_milli},{dy_milli},{detected_flag}\n"

        if self._ser and self._ser.is_open:
            try:
                self._ser.write(message.encode("utf-8"))
                return
            except Exception as exc:  # pragma: no cover - runtime safety
                print(f"Serial write failed: {exc}. Falling back to stdout.")
        if self.fallback_to_stdout:
            print(message.strip())


class PersonTracker:
    def __init__(self, config: TrackerConfig) -> None:
        self.cfg = config
        self.cap = self._init_camera()
        self.link = ESP32Link(config.serial_port, config.serial_baudrate)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        self.filtered_dx = 0.0
        self.filtered_dy = 0.0

    def _init_camera(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self.cfg.camera_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.cfg.camera_index}")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.frame_height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap

    def _pick_best_detection(self, boxes, weights) -> Optional[Tuple[int, int, int, int, float]]:
        if len(boxes) == 0:
            return None

        best_idx = 0
        best_score = -1.0
        for i, (box, score) in enumerate(zip(boxes, weights)):
            # HOG weights are relative; combine weight and box size to favor closer targets
            area = box[2] * box[3]
            combined = float(score) + area * 1e-6
            if combined > best_score:
                best_score = combined
                best_idx = i
        x, y, w, h = boxes[best_idx]
        return x, y, w, h, float(weights[best_idx]) if len(weights) > best_idx else 0.0

    def _compute_offsets(self, box, frame_shape) -> Tuple[float, float]:
        x, y, w, h, _ = box
        frame_h, frame_w = frame_shape[:2]
        cx = x + w * 0.5
        cy = y + h * 0.5
        fx = frame_w * 0.5
        fy = frame_h * 0.5

        dx = (cx - fx) / fx  # left = negative, right = positive
        dy = -((cy - fy) / fy)  # up = positive, down = negative
        return dx, dy

    def _smooth_offsets(self, dx: float, dy: float) -> Tuple[float, float]:
        alpha = self.cfg.smoothing_alpha
        self.filtered_dx = (1 - alpha) * self.filtered_dx + alpha * dx
        self.filtered_dy = (1 - alpha) * self.filtered_dy + alpha * dy
        return self.filtered_dx, self.filtered_dy

    def _annotate(self, frame, box: Optional[Tuple[int, int, int, int, float]], dx: float, dy: float) -> None:
        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2
        cv2.line(frame, (cx, 0), (cx, h), (0, 255, 0), 1)
        cv2.line(frame, (0, cy), (w, cy), (0, 255, 0), 1)

        if box:
            x, y, bw, bh, score = box
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (255, 0, 0), 2)
            cv2.circle(frame, (int(x + bw * 0.5), int(y + bh * 0.5)), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"conf: {score:.2f}", (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.putText(
            frame,
            f"dx: {dx:+.2f} dy: {dy:+.2f}",
            (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def run(self) -> None:
        print("Starting person tracking. Press 'q' to exit.")
        while True:
            ok, frame = self.cap.read()
            if not ok:
                print("Frame grab failed; exiting.")
                break

            boxes, weights = self.hog.detectMultiScale(
                frame,
                winStride=self.cfg.detection_win_stride,
                padding=self.cfg.detection_padding,
                scale=self.cfg.detection_scale,
            )
            best = self._pick_best_detection(boxes, weights)

            if best and best[4] >= self.cfg.min_confidence:
                dx, dy = self._compute_offsets(best, frame.shape)
                dx_s, dy_s = self._smooth_offsets(dx, dy)
                self.link.send_offset(dx_s, dy_s, detected=True)
            else:
                best = None
                dx_s, dy_s = self._smooth_offsets(0.0, 0.0)
                self.link.send_offset(dx_s, dy_s, detected=False)

            if self.cfg.show_preview:
                self._annotate(frame, best, dx_s, dy_s)
                cv2.imshow("Person Tracker", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

        self.cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    cfg = TrackerConfig()
    tracker = PersonTracker(cfg)
    tracker.run()


if __name__ == "__main__":
    main()
