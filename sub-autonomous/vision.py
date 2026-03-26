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

        # Pre-create kernel (avoid realloc every frame)
        self.kernel = np.ones((5, 5), np.uint8)

    def preprocess(self, frame):
        # Light blur → reduces noise without heavy cost
        return cv2.GaussianBlur(frame, (5, 5), 0)

    def yellow_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array(HSV_LOWER, dtype=np.uint8)
        upper = np.array(HSV_UPPER, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        # ---- Morphological cleanup ----
        # Remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        # Merge fragmented regions
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        return mask

    def detect_target(self):
        frame = self.camera.get_frame()
        if frame is None:
            return False, None, None, 0.0, float("inf"), None, None

        frame = self.preprocess(frame)
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

            # ---- Shape filtering ----
            aspect_ratio = bw / float(bh)

            # Reject very skinny / weird shapes (side noise)
            if aspect_ratio < 0.3 or aspect_ratio > 3:
                continue

            # Solidity (removes fragmented blobs)
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)

            if hull_area == 0:
                continue

            solidity = area / hull_area
            if solidity < 0.5:
                continue

            # ---- Center calculation ----
            cx = x + bw // 2
            cy = y + bh // 2

            pixel_dist = ((cx - frame_cx) ** 2 + (cy - frame_cy) ** 2) ** 0.5

            # ---- Draw ----
            cv2.rectangle(display, (x, y), (x + bw, y + bh), (255, 0, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 255, 255), -1)

            cv2.putText(
                display,
                f"A:{int(area)} S:{solidity:.2f}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

            # ---- Best contour selection ----
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

if __name__ == "__main__":
    vision = Vision()

    try:
        while True:
            detected, cx, cy, area, pixel_dist, display, mask = vision.detect_target()
            vision.show_preview(display, mask)

            if detected:
                print(f"Detected target at pixel ({cx}, {cy}) with area {area:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        vision.release()