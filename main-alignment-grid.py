import cv2
import numpy as np

# ==========================================
# SETTINGS
# ==========================================

CAMERA_INDEX = 0

MIN_AREA = 5000
DEFAULT_EDGE_THRESHOLD = 0.30  # Default minimum fraction of target perimeter matching Canny edges

targets = []
calibrated = False


# ==========================================
# GEOMETRY
# ==========================================

def order_points(points):
    points = np.array(points, dtype=np.float32)

    s = points.sum(axis=1)
    d = np.diff(points, axis=1).reshape(-1)

    return np.array([
        points[np.argmin(s)],  # top-left
        points[np.argmin(d)],  # top-right
        points[np.argmax(s)],  # bottom-right
        points[np.argmax(d)]   # bottom-left
    ], dtype=np.float32)


# ==========================================
# DETECT RECTANGLES (For Calibration Reference)
# ==========================================

def detect_rectangles(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 100, 255])

    mask = cv2.inRange(hsv, lower_white, upper_white)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    rectangles = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.025 * perimeter, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            corners = order_points(approx.reshape(4, 2))
            rectangles.append(corners)

    return rectangles


# ==========================================
# GENERATE TIGHT, CENTERED HASHTAG
# ==========================================

def generate_centered_hashtag(reference, frame_shape):
    h_img, w_img = frame_shape[:2]
    image_center = np.array([w_img / 2, h_img / 2], dtype=np.float32)

    ref_pts = order_points(reference)
    v = ref_pts[1] - ref_pts[0]
    angle_rad = np.arctan2(v[1], v[0])

    W = np.linalg.norm(ref_pts[1] - ref_pts[0])
    H = np.linalg.norm(ref_pts[3] - ref_pts[0])
    
    if W > H:
        W, H = H, W

    GAP = W * 0.4  # Tighter gap for hashtag shape

    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    R = np.array([[cos_a, -sin_a], [sin_a, cos_a]], dtype=np.float32)

    half_w = W / 2.0
    half_h = H / 2.0
    gap_half = GAP / 2.0

    bars_local = []

    # 1. Left Vertical Bar
    lv_cx = -half_w - gap_half
    lv_pts = np.array([
        [-half_w, -half_h],
        [half_w, -half_h],
        [half_w, half_h],
        [-half_w, half_h]
    ], dtype=np.float32) + np.array([lv_cx, 0], dtype=np.float32)
    bars_local.append(lv_pts)

    # 2. Right Vertical Bar
    rv_cx = half_w + gap_half
    rv_pts = np.array([
        [-half_w, -half_h],
        [half_w, -half_h],
        [half_w, half_h],
        [-half_w, half_h]
    ], dtype=np.float32) + np.array([rv_cx, 0], dtype=np.float32)
    bars_local.append(rv_pts)

    # 3. Top Horizontal Bar
    th_cy = -half_w - gap_half
    th_pts = np.array([
        [-half_h, -half_w],
        [half_h, -half_w],
        [half_h, half_w],
        [-half_h, half_w]
    ], dtype=np.float32) + np.array([0, th_cy], dtype=np.float32)
    bars_local.append(th_pts)

    # 4. Bottom Horizontal Bar
    bh_cy = half_w + gap_half
    bh_pts = np.array([
        [-half_h, -half_w],
        [half_h, -half_w],
        [half_h, half_w],
        [-half_h, half_w]
    ], dtype=np.float32) + np.array([0, bh_cy], dtype=np.float32)
    bars_local.append(bh_pts)

    hashtag_targets = []
    for bar in bars_local:
        rotated_bar = np.dot(bar, R.T) + image_center
        ordered_bar = order_points(rotated_bar)
        hashtag_targets.append(ordered_bar)

    return hashtag_targets


# ==========================================
# CANNY EDGE ALIGNMENT CHECK
# ==========================================

def compute_edge_alignment_score(edges, target_polygon):
    pts = target_polygon.astype(np.float32)
    score_sum = 0
    total_points = 0
    h_img, w_img = edges.shape

    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        length = np.linalg.norm(p2 - p1)
        num_samples = int(length / 3)
        if num_samples < 1:
            num_samples = 1

        for t in np.linspace(0, 1, num_samples):
            pt = (1 - t) * p1 + t * p2
            x, y = int(pt[0]), int(pt[1])
            if 0 <= y < h_img and 0 <= x < w_img:
                window = edges[max(0, y-1):min(h_img, y+2), max(0, x-1):min(w_img, x+2)]
                if np.any(window > 0):
                    score_sum += 1
            total_points += 1

    return score_sum / total_points if total_points > 0 else 0.0


# ==========================================
# MAIN PROGRAM
# ==========================================

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Cannot open webcam.")

window_name = "Hashtag Assembly"
cv2.namedWindow(window_name)

# Create trackbar for tolerance (0 to 100 percentage)
# Initial value is set to DEFAULT_EDGE_THRESHOLD * 100 (e.g. 30)
cv2.createTrackbar("Tolerance %", window_name, int(DEFAULT_EDGE_THRESHOLD * 100), 100, lambda x: None)

print("\nHASHTAG ASSEMBLY")
print("----------------")
print("1. Place at least one rectangle in view.")
print("2. Press C to generate the centered hashtag grid.")
print("3. Use the 'Tolerance %' slider to adjust edge sensitivity.")
print("R = Reset calibration")
print("Q = Quit")
print("----------------\n")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    display = frame.copy()
    detected = detect_rectangles(frame)

    # Get current tolerance value from the trackbar (0 - 100 -> 0.0 - 1.0)
    tolerance_slider_val = cv2.getTrackbarPos("Tolerance %", window_name)
    current_edge_threshold = tolerance_slider_val / 100.0

    # Compute Canny edges for alignment checking
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    if calibrated:
        correct_targets = set()

        for i, target in enumerate(targets):
            score = compute_edge_alignment_score(edges, target)
            is_aligned = score >= current_edge_threshold

            color = (0, 255, 0) if is_aligned else (0, 0, 255)

            if is_aligned:
                correct_targets.add(i)

            points = target.astype(np.int32)
            cv2.polylines(display, [points], True, color, 2)
            
            center = np.mean(points, axis=0).astype(int)
            cv2.putText(display, f"T{i+1}: {score:.2f}", tuple(center), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        count = len(correct_targets)
        if count == 4:
            status = "HASHTAG COMPLETE!"
            status_color = (0, 255, 0)
        else:
            status = f"ALIGNED: {count}/4"
            status_color = (0, 0, 255)

        cv2.putText(display, status, (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)

    else:
        cv2.putText(display, "Press C to generate centered hashtag", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        for rect in detected:
            cv2.polylines(display, [rect.astype(np.int32)], True, (200, 200, 200), 1)

    cv2.imshow(window_name, display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("c"):
        if not detected:
            print("No rectangle detected to calibrate from.")
        else:
            h, w = frame.shape[:2]
            image_center = np.array([w / 2, h / 2])

            reference = min(
                detected,
                key=lambda rect: np.linalg.norm(np.mean(rect, axis=0) - image_center)
            )

            targets = generate_centered_hashtag(reference, frame.shape)
            calibrated = True
            print("Centered tight hashtag grid generated!")

    elif key == ord("r"):
        targets = []
        calibrated = False
        print("Calibration reset.")

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()