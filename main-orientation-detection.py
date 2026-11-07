import cv2
import numpy as np
from collections import deque, Counter

CAM_INDEX = 0
MIN_AREA = 1500        # min filled area of a profile (pixels)
MAX_SOLIDITY = 0.93    # filled_area / hull_area; above this the shape has no U opening
GROUND_TOL = 15        # px: max gap to the ground line to count as "on ground"
DILATE_ITERS = 2       # edge thickening; lower it if the U opening gets closed
HISTORY = 7            # frames for majority-vote smoothing

NAMES = {1: "UP", 2: "DOWN", 3: "SIDEWAYS"}

state = {"clicks": [], "line": None}


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        state["clicks"].append((x, y))
        if len(state["clicks"]) == 2:
            state["line"] = tuple(state["clicks"])
            state["clicks"] = []


def ground_geometry(line, shape):
    """Returns p1, g (unit vector along ground), n (unit 'up' normal), s_map."""
    h, w = shape[:2]
    if line is None:
        line = ((0, int(h * 0.9)), (w - 1, int(h * 0.9)))
    p1 = np.array(line[0], float)
    p2 = np.array(line[1], float)
    if p2[0] < p1[0]:
        p1, p2 = p2, p1
    g = p2 - p1
    if np.linalg.norm(g) < 1:
        p1, p2 = np.array([0.0, h * 0.9]), np.array([w - 1.0, h * 0.9])
        g = p2 - p1
    g /= np.linalg.norm(g)
    n = np.array([g[1], -g[0]])
    if n[1] > 0:           # make the normal point toward the top of the image
        n = -n
    ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
    s_map = (xs - p1[0]) * n[0] + (ys - p1[1]) * n[1]   # >0 = above the ground
    return p1, g, n, s_map


def get_edges(gray, low, high, auto):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    if auto:
        high, _ = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        low = 0.5 * high
    return cv2.Canny(blur, low, high)


def fill_profiles(edges, above):
    edges = edges.copy()
    edges[~above] = 0
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thick = cv2.dilate(edges, k, iterations=DILATE_ITERS)
    thick = cv2.morphologyEx(thick, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(thick, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) > MIN_AREA]
    contours.sort(key=lambda c: cv2.boundingRect(c)[0])   # left -> right
    filled = np.zeros_like(edges)
    cv2.drawContours(filled, contours, -1, 255, -1)
    return filled, contours


def centroid(c):
    m = cv2.moments(c)
    if m["m00"] == 0:
        return None
    return np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]])


def classify(cnt, p1, g, n):
    hull = cv2.convexHull(cnt)
    a_h = cv2.contourArea(hull)
    if a_h == 0:
        return None
    solidity = cv2.contourArea(cnt) / a_h
    cs, ch = centroid(cnt), centroid(hull)
    if cs is None or ch is None:
        return None

    # gap between the profile and the ground line
    pts = cnt.reshape(-1, 2).astype(float)
    gap = float(((pts - p1) @ n).min())

    info = {"hull": hull, "cs": cs, "solidity": solidity, "gap": gap}
    if solidity > MAX_SOLIDITY:
        info["pos"] = None
        return info

    # the hull centroid is shifted toward the opening of the U
    d = ch - cs
    norm = np.linalg.norm(d)
    if norm < 1e-6:
        info["pos"] = None
        return info
    theta = np.degrees(np.arccos(np.clip(d @ n / norm, -1, 1)))  # angle vs "up"
    if theta < 45:
        pos = 1
    elif theta > 135:
        pos = 2
    else:
        pos = 3
    info.update(pos=pos, d=d / norm, theta=theta,
                side="right" if d @ g > 0 else "left")
    return info


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", on_mouse)
    cv2.namedWindow("Edges")
    cv2.createTrackbar("low", "Edges", 50, 255, lambda v: None)
    cv2.createTrackbar("high", "Edges", 150, 255, lambda v: None)

    auto, paused, frame = True, False, None
    cache, votes = {}, {}

    while True:
        if not paused or frame is None:
            ok, frame = cap.read()
            if not ok:
                break

        key = (state["line"], frame.shape)
        if cache.get("key") != key:
            cache = {"key": key, "geo": ground_geometry(state["line"], frame.shape)}
        p1, g, n, s_map = cache["geo"]
        above = s_map > 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        low = cv2.getTrackbarPos("low", "Edges")
        high = cv2.getTrackbarPos("high", "Edges")
        edges = get_edges(gray, low, high, auto)
        edges[~above] = 0
        filled, contours = fill_profiles(edges, above)

        view = frame.copy()
        a = (p1 - g * 3000).astype(int)
        b = (p1 + g * 3000).astype(int)
        cv2.line(view, tuple(a), tuple(b), (0, 255, 255), 2)
        for c in state["clicks"]:
            cv2.circle(view, c, 5, (0, 255, 255), -1)

        for i, cnt in enumerate(contours):
            info = classify(cnt, p1, g, n)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.drawContours(view, [cnt], -1, (0, 255, 0), 2)
            tag = f"P{i + 1}"

            if info is None or info["pos"] is None:
                text = f"{tag}: not a U"
                votes.pop(i, None)
            else:
                votes.setdefault(i, deque(maxlen=HISTORY)).append(info["pos"])
                pos = Counter(votes[i]).most_common(1)[0][0]
                text = f"{tag}: position {pos} ({NAMES[pos]}"
                if pos == 3:
                    text += f", opens {info['side']}"
                text += ")"
                if info["gap"] > GROUND_TOL:
                    text += " [off ground]"
                cv2.drawContours(view, [info["hull"]], -1, (255, 128, 0), 1)
                p0 = tuple(info["cs"].astype(int))
                p_end = tuple((info["cs"] + info["d"] * 70).astype(int))
                cv2.arrowedLine(view, p0, p_end, (0, 0, 255), 3, tipLength=0.3)

            cv2.putText(view, text, (x, max(y - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        for k in list(votes):
            if k >= len(contours):
                votes.pop(k)

        mode = "auto" if auto else "manual"
        cv2.putText(view, f"canny: {mode} | click 2 pts = ground | r reset | a auto/manual | space pause | q quit",
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow("Frame", view)
        cv2.imshow("Edges", edges)
        cv2.imshow("Filled mask", filled)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        elif k == ord("r"):
            state["line"], state["clicks"] = None, []
        elif k == ord("a"):
            auto = not auto
        elif k == ord(" "):
            paused = not paused

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()