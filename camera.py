import cv2
import numpy as np

# =========================
# CONFIG
# =========================
FRAME_W = 1280
FRAME_H = 720
AREA_THRESHOLD = 100

# =========================
# HSV MASK FUNCTION
# =========================
def yellow_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array([20, 70, 120])
    upper = np.array([35, 255, 255])

    return cv2.inRange(hsv, lower, upper)

# =========================
# MAIN
# =========================
def main():
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error: Could not open camera")
        return

    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    # cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)

        # Create mask
        mask = yellow_mask(frame)

        # Find contours
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Draw detections
        for c in contours:
            area = cv2.contourArea(c)
            if area < AREA_THRESHOLD:
                continue

            x, y, w, h = cv2.boundingRect(c)
            cx = x + w // 2
            cy = y + h // 2

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # Draw center
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Label
            cv2.putText(
                frame,
                f"Area: {int(area)}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1
            )

        # Show outputs
        cv2.imshow("Detection", frame)
        # cv2.imshow("Mask", mask)

        # Exit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()