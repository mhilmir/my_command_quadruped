import cv2
from ultralytics import YOLO

# Load YOLO model
model = YOLO('yolov8n.pt')

# Open the camera or video
cap = cv2.VideoCapture(4)
# 0 for built-in laptop webcam
# 6 for realsense color frame


# Globals
tracker = None
tracking = False
detected_boxes = []

# Mouse click callback
clicked_point = None

def click_event(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)

cv2.namedWindow("YOLO Detection + Tracking")
cv2.setMouseCallback("YOLO Detection + Tracking", click_event)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    if not tracking:
        results = model(frame_rgb)
        detected_boxes = []

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()  # (x1, y1, x2, y2)
            for box in boxes:
                x1, y1, x2, y2 = map(int, box)
                detected_boxes.append((x1, y1, x2, y2))
                # Draw all detected boxes
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Check for user click inside any box
        if clicked_point:
            cx, cy = clicked_point
            for x1, y1, x2, y2 in detected_boxes:
                if x1 <= cx <= x2 and y1 <= cy <= y2:
                    # Start tracking this box
                    bbox = (x1, y1, x2 - x1, y2 - y1)
                    tracker = cv2.TrackerCSRT_create()
                    tracker.init(frame, bbox)
                    tracking = True
                    break
            clicked_point = None  # Reset

    else:
        # Tracking in progress
        success, bbox = tracker.update(frame)

        if success:
            x, y, w, h = map(int, bbox)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, "Tracking", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Lost Tracking", (50, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            tracking = False  # Re-detect

    cv2.imshow("YOLO Detection + Tracking", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):
        # Cancel tracking
        tracking = False
        tracker = None
        print("[INFO] Tracking canceled. Returning to detection mode.")

cap.release()
cv2.destroyAllWindows()
