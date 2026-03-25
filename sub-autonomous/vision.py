import threading
import time

import cv2
import numpy as np

from config import (
    AREA_THRESHOLD,
    CAMERA_INDEX,
    FRAME_HEIGHT,
    FRAME_WIDTH,
    HSV_LOWER,
    HSV_UPPER,
    SHOW_MASK_WINDOW,
)


class CameraManager:
    """Continuously reads camera frames in a background thread."""

    def __init__(self, src=CAMERA_INDEX):
        self.cap = cv2.VideoCapture(src)
        self.frame = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {CAMERA_INDEX}")

        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()

        while self.frame is None and not self._stop_event.is_set():
            time.sleep(0.01)

        return self

    def _update(self):
        while not self._stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                continue

            frame = cv2.flip(frame, 1)
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            with self._lock:
                self.frame = frame

    def get_frame(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self._stop_event.set()

        if self._thread:
            self._thread.join(timeout=1.0)

        self.cap.release()
        cv2.destroyAllWindows()


class Vision:
    def __init__(self):
        self.camera = CameraManager().start()

    @staticmethod
    def yellow_mask(frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(HSV_LOWER, dtype=np.uint8)
        upper = np.array(HSV_UPPER, dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def detect_target(self):
        frame = self.camera.get_frame()
        if frame is None:
            return False, None, None, 0.0, float("inf"), None, None

        mask = self.yellow_mask(frame)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        display = frame.copy()
        h, w = display.shape[:2]
        frame_cx, frame_cy = w // 2, h // 2

        cv2.circle(display, (frame_cx, frame_cy), 5, (0, 0, 255), -1)

        best = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < AREA_THRESHOLD:
                continue

            x, y, bw, bh = cv2.boundingRect(contour)
            cx = x + bw // 2
            cy = y + bh // 2

            pixel_dist = ((cx - frame_cx) ** 2 + (cy - frame_cy) ** 2) ** 0.5

            cv2.rectangle(display, (x, y), (x + bw, y + bh), (255, 0, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 255, 255), -1)
            cv2.putText(
                display,
                f"Area:{int(area)} D:{int(pixel_dist)}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

            if best is None or area > best["area"]:
                best = {
                    "cx": cx,
                    "cy": cy,
                    "area": float(area),
                    "pixel_dist": float(pixel_dist),
                }

        if best is None:
            return False, None, None, 0.0, float("inf"), display, mask

        return (
            True,
            best["cx"],
            best["cy"],
            best["area"],
            best["pixel_dist"],
            display,
            mask,
        )

    def show_preview(self, frame=None, mask=None):
        if frame is None:
            frame = self.camera.get_frame()

        if frame is not None:
            cv2.imshow("Detection", frame)

        if SHOW_MASK_WINDOW and mask is not None:
            cv2.imshow("Mask", mask)

        cv2.waitKey(1)

    def release(self):
        self.camera.stop()
