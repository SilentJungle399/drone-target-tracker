import cv2
import time
import threading
import numpy as np
import logging
from ultralytics import YOLO

from config import FRAME_WIDTH, FRAME_HEIGHT, YOLO_CONFIDENCE, CENTER_TOLERANCE


logger = logging.getLogger(__name__)


class CameraManager:
    """Background-threaded camera that continuously reads frames from a single
    VideoCapture instance.  All program stages share one CameraManager so the
    capture device is opened only once."""

    def __init__(self, src=1):
        self.cap = cv2.VideoCapture(src)
        self.frame = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._update, daemon=True)
        self._thread.start()
        # Block until the first frame arrives so callers are never handed None
        # immediately after start().
        while self.frame is None:
            time.sleep(0.01)
        return self

    def _update(self):
        while not self._stop_event.is_set():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.flip(frame, -1)
                with self._lock:
                    self.frame = frame

    def get_frame(self):
        """Return the most recent frame.  No copy – callers must not mutate it."""
        with self._lock:
            return self.frame

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join()
        self.cap.release()
        cv2.destroyAllWindows()


class Vision:
    def __init__(self, dev=False, controller=None):
        self.dev = dev
        self.controller = controller  # only needed in dev mode for heading compensation

        self.model = YOLO("best-colab.pt", task="detect").to("cuda")
        logger.info("YOLO model loaded")

        self.camera = CameraManager().start()

    def _rotate_frame(self, frame):
        """Rotate frame to compensate for drone yaw (dev/simulation only)."""
        if self.controller is None:
            return frame

        _, _, heading = self.controller.get_gps_reading(heading_=True)
        angle = (heading + 360) % 360

        h, w = frame.shape[:2]
        M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
        return cv2.warpAffine(frame, M, (w, h))

    def detect_bullseye(self):
        """Run YOLO inference on the latest camera frame.

        Returns
        -------
        (detected, center_x, center_y, display_frame)
            display_frame – annotated copy of the frame (dev mode) or the raw
            frame (non-dev mode).  Always suitable for cv2.imshow().
        """
        raw = self.camera.get_frame()

        if raw is None:
            return False, None, None, None

        # resize frame to model input size
        frame = cv2.resize(raw, (FRAME_WIDTH, FRAME_HEIGHT))

        # in dev/sim mode, rotate to cancel out drone yaw so detections are
        # relative to the drone body frame, not compass north
        # if self.dev:
        #     frame = self._rotate_frame(frame)

        # Copy once for annotation; raw camera frame is never mutated.
        display = frame.copy()

        detected = False
        center_x = None
        center_y = None

        results = self.model.predict(
            frame,
            imgsz=FRAME_WIDTH,
            verbose=False
        )

        for result in results:
            for box in result.boxes:
                conf = float(box.conf)
                if conf < YOLO_CONFIDENCE:
                    continue

                detected = True

                x1, y1, x2, y2 = box.xyxy[0]

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                if self.dev:
                    # bounding box
                    cv2.rectangle(
                        display,
                        (int(x1), int(y1)),
                        (int(x2), int(y2)),
                        (0, 255, 0),
                        2
                    )

                    # confidence text
                    cv2.putText(
                        display,
                        f"{conf:.2f}",
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    # bbox center dot
                    cv2.circle(
                        display,
                        (int(center_x), int(center_y)),
                        6,
                        (255, 0, 0),
                        -1
                    )

                break

        if self.dev:
            frame_center_x = FRAME_WIDTH // 2
            frame_center_y = FRAME_HEIGHT // 2

            # frame center dot
            cv2.circle(
                display,
                (frame_center_x, frame_center_y),
                6,
                (0, 0, 255),
                -1
            )

            # tolerance square
            cv2.rectangle(
                display,
                (
                    frame_center_x - CENTER_TOLERANCE,
                    frame_center_y - CENTER_TOLERANCE
                ),
                (
                    frame_center_x + CENTER_TOLERANCE,
                    frame_center_y + CENTER_TOLERANCE
                ),
                (255, 255, 0),
                2
            )

        return detected, center_x, center_y, display

    def show_preview(self, frame=None):
        """Display a frame in the preview window.

        Parameters
        ----------
        frame : ndarray or None
            Pass an annotated frame (from detect_bullseye) to show inference
            overlays.  Pass None (or omit) to show the raw camera feed.
        """
        if not self.dev:
            return
        if frame is None:
            frame = self.camera.get_frame()
        if frame is not None:
            cv2.imshow("view", frame)
            cv2.waitKey(1)

    def release(self):
        self.camera.stop()