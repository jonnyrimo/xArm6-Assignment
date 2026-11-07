"""
RRT* path planning + precision feature for a holonomic rectangular body.

- No obstacles. Start and target positions are random inside the workspace.
- Final area for RRT*: a GREY rectangle centered at the target position, with a
  random orientation.
- Precision area: a GREEN rectangle centered at the target, with the same
  orientation, fully inside the grey one.
- The moving body is a RED rectangle with a random initial orientation.
- The target orientation of the body is the orientation of the final rectangles.

Phase 1 - RRT*: the grey tree grows until it reaches the grey area. The best path
          is highlighted and followed by the body.
Phase 2 - Precision feature: at the end of the path, a proportional controller
          (velocity ~ Euclidean position error, angular velocity ~ orientation
          error) brings the body inside the green area with the right orientation.

Requirements: numpy, matplotlib
    pip install numpy matplotlib
"""

from dataclasses import dataclass

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
SEED = None                                       # set to None for a different run each time
WORKSPACE = (0.0, 100.0, 0.0, 100.0)           # xmin, xmax, ymin, ymax
MIN_START_GOAL_DIST = 50.0                     # min distance between start and target

# Body (red rectangle): full length (along its heading) and width
BODY_LENGTH = 4.0
BODY_WIDTH = 2.0

# Final areas: half sizes (along x, along y) in the rectangle's own frame
GOAL_HALF = np.array([10.0, 5.0])              # grey area used by RRT*   (20 x 10)
PRECISION_HALF = np.array([3.0, 1.5])          # green precision area     (6 x 3)

# RRT*
MAX_ITER = 5000
EXTRA_ITER_AFTER_GOAL = 1200                   # keep optimizing after 1st solution
STEP_SIZE = 2.0
GOAL_SAMPLE_RATE = 0.05
GAMMA = 60.0
MAX_NEIGHBOR_RADIUS = 15.0
DRAW_EVERY = 25

# Path following (phase 1)
V_MAX = 15.0                                   # max linear speed [units/s]
DT = 0.05                                      # simulation time step [s]
WAYPOINT_TOL = 0.3

# Precision feature (phase 2)
ANGLE_TOL = np.deg2rad(2.0)                    # accepted orientation error [rad]
KP_POS = 1.5                                   # v = KP_POS * position error
KP_ANG = 3.0                                   # w = KP_ANG * angle error
OMEGA_MAX = 2.0                                # max angular speed [rad/s]
PRECISION_MAX_TIME = 30.0                      # safety timeout [s]

assert np.all(PRECISION_HALF < GOAL_HALF), "precision area must be inside the final area"


# ----------------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------------
def wrap_angle(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def rect_corners(center, theta, half):
    hl, hw = half
    local = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
    return center + local @ rot(theta).T


def heading_segment(center, theta, half_len):
    """Line from the rectangle center to the middle of its 'front' edge."""
    tip = center + rot(theta) @ np.array([half_len, 0.0])
    return np.array([center, tip])


def in_rect(p, center, theta, half):
    """True if point p is inside the oriented rectangle."""
    d = rot(theta).T @ (p - center)
    return bool(abs(d[0]) <= half[0] and abs(d[1]) <= half[1])


BODY_HALF = np.array([BODY_LENGTH / 2.0, BODY_WIDTH / 2.0])


# ----------------------------------------------------------------------------
# Problem definition (random start / target / target orientation)
# ----------------------------------------------------------------------------
@dataclass
class Problem:
    start: np.ndarray
    goal: np.ndarray          # target position (center of the final rectangles)
    theta_goal: float         # orientation of the final rectangles = target orientation


def make_problem(rng):
    xmin, xmax, ymin, ymax = WORKSPACE
    # The grey rectangle must fit in the workspace whatever its orientation
    m_goal = float(np.hypot(*GOAL_HALF)) + 1.0
    m_start = 3.0
    while True:
        start = rng.uniform([xmin + m_start, ymin + m_start], [xmax - m_start, ymax - m_start])
        goal = rng.uniform([xmin + m_goal, ymin + m_goal], [xmax - m_goal, ymax - m_goal])
        if np.linalg.norm(goal - start) >= MIN_START_GOAL_DIST:
            break
    theta_goal = rng.uniform(-np.pi, np.pi)
    return Problem(start, goal, theta_goal)


# ----------------------------------------------------------------------------
# RRT*
# ----------------------------------------------------------------------------
class Node:
    __slots__ = ("pos", "parent", "cost", "children")

    def __init__(self, pos, parent=None, cost=0.0):
        self.pos = pos
        self.parent = parent          # index of parent node
        self.cost = cost              # cost-to-come from start
        self.children = []            # indices of children


class RRTStar:
    def __init__(self, rng, problem):
        self.rng = rng
        self.pb = problem
        self.nodes = [Node(problem.start.copy())]
        self.positions = np.zeros((MAX_ITER + 1, 2))
        self.positions[0] = problem.start
        self.goal_nodes = []          # indices of nodes inside the grey rectangle

    def sample(self):
        if self.rng.random() < GOAL_SAMPLE_RATE:
            return self.pb.goal.copy()
        xmin, xmax, ymin, ymax = WORKSPACE
        return np.array([self.rng.uniform(xmin, xmax), self.rng.uniform(ymin, ymax)])

    def nearest(self, p):
        n = len(self.nodes)
        d2 = np.sum((self.positions[:n] - p) ** 2, axis=1)
        return int(np.argmin(d2))

    @staticmethod
    def steer(p_from, p_to):
        d = p_to - p_from
        dist = np.linalg.norm(d)
        if dist <= STEP_SIZE:
            return p_to.copy()
        return p_from + d / dist * STEP_SIZE

    def neighbors(self, p):
        n = len(self.nodes)
        r = min(GAMMA * np.sqrt(np.log(n + 1) / (n + 1)), MAX_NEIGHBOR_RADIUS)
        d = np.linalg.norm(self.positions[:n] - p, axis=1)
        return np.where(d <= r)[0]

    def propagate_cost(self, idx, delta):
        """Add `delta` to the cost of all descendants of node idx."""
        stack = list(self.nodes[idx].children)
        while stack:
            c = stack.pop()
            self.nodes[c].cost += delta
            stack.extend(self.nodes[c].children)

    def extend(self):
        """One RRT* iteration."""
        q_rand = self.sample()
        i_near = self.nearest(q_rand)
        q_new = self.steer(self.nodes[i_near].pos, q_rand)

        near_ids = self.neighbors(q_new)

        # Choose the best parent among neighbours
        best_parent = i_near
        best_cost = self.nodes[i_near].cost + np.linalg.norm(q_new - self.nodes[i_near].pos)
        for i in near_ids:
            c = self.nodes[i].cost + np.linalg.norm(q_new - self.nodes[i].pos)
            if c < best_cost:
                best_parent, best_cost = int(i), c

        # Insert the new node
        new_idx = len(self.nodes)
        self.nodes.append(Node(q_new, best_parent, best_cost))
        self.positions[new_idx] = q_new
        self.nodes[best_parent].children.append(new_idx)

        # Rewire neighbours through the new node if that lowers their cost
        for i in near_ids:
            i = int(i)
            if i == best_parent:
                continue
            node = self.nodes[i]
            c_new = best_cost + np.linalg.norm(node.pos - q_new)
            if c_new < node.cost:
                self.nodes[node.parent].children.remove(i)
                node.parent = new_idx
                self.nodes[new_idx].children.append(i)
                delta = c_new - node.cost
                node.cost = c_new
                self.propagate_cost(i, delta)

        if in_rect(q_new, self.pb.goal, self.pb.theta_goal, GOAL_HALF):
            self.goal_nodes.append(new_idx)

    def best_goal_node(self):
        if not self.goal_nodes:
            return None
        return min(self.goal_nodes, key=lambda i: self.nodes[i].cost)

    def extract_path(self, idx):
        path = []
        while idx is not None:
            path.append(self.nodes[idx].pos.copy())
            idx = self.nodes[idx].parent
        return np.array(path[::-1])

    def edge_segments(self):
        return [(self.nodes[n.parent].pos, n.pos) for n in self.nodes if n.parent is not None]


# ----------------------------------------------------------------------------
# Rectangle body (pose + artists)
# ----------------------------------------------------------------------------
class RectBody:
    def __init__(self, ax, pos, theta):
        self.pos = pos.copy()
        self.theta = float(theta)
        self.trail = [self.pos.copy()]

        self.trail_line, = ax.plot([], [], "-", color="orange", lw=1.5, zorder=4)
        self.poly = Polygon(rect_corners(self.pos, self.theta, BODY_HALF), closed=True,
                            fc="red", ec="darkred", lw=1.2, zorder=6, label="Rectangle body")
        ax.add_patch(self.poly)
        self.heading, = ax.plot([], [], "k-", lw=2, zorder=7)
        self.refresh()

    def move(self, new_pos, new_theta=None):
        self.pos = new_pos
        if new_theta is not None:
            self.theta = wrap_angle(new_theta)
        self.trail.append(self.pos.copy())

    def refresh(self):
        self.poly.set_xy(rect_corners(self.pos, self.theta, BODY_HALF))
        h = heading_segment(self.pos, self.theta, BODY_HALF[0])
        self.heading.set_data(h[:, 0], h[:, 1])
        t = np.array(self.trail)
        self.trail_line.set_data(t[:, 0], t[:, 1])


# ----------------------------------------------------------------------------
# Plot setup
# ----------------------------------------------------------------------------
def setup_plot(pb):
    plt.ion()
    fig, ax = plt.subplots(figsize=(11, 7.5))
    fig.subplots_adjust(left=0.07, right=0.70)
    xmin, xmax, ymin, ymax = WORKSPACE
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(alpha=0.2)

    # Final area used by RRT*: GREY rectangle, random orientation
    ax.add_patch(Polygon(rect_corners(pb.goal, pb.theta_goal, GOAL_HALF), closed=True,
                         fc="grey", ec="dimgray", alpha=0.45, lw=1.5, zorder=2,
                         label="Final area (RRT*)"))
    # Precision area: GREEN rectangle, same orientation, inside the grey one
    ax.add_patch(Polygon(rect_corners(pb.goal, pb.theta_goal, PRECISION_HALF), closed=True,
                         fc="limegreen", ec="darkgreen", alpha=0.6, lw=1.5, zorder=3,
                         label="Precision area"))
    ax.plot(*pb.goal, "g*", ms=13, zorder=4, label="Target position")

    # Target pose: dashed outline of the body at the target + heading line
    ax.add_patch(Polygon(rect_corners(pb.goal, pb.theta_goal, BODY_HALF), closed=True,
                         fill=False, ec="darkgreen", ls="--", lw=1.5, zorder=5,
                         label="Target pose"))
    th = heading_segment(pb.goal, pb.theta_goal, BODY_HALF[0])
    ax.plot(th[:, 0], th[:, 1], "--", color="darkgreen", lw=1.5, zorder=5)

    ax.plot(*pb.start, "ko", ms=5, label="Start")

    tree = LineCollection([], colors="grey", linewidths=0.6, alpha=0.7)   # grey tree
    ax.add_collection(tree)
    return fig, ax, tree


def put_legend(ax):
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=9)


# ----------------------------------------------------------------------------
# Phase 1: follow the RRT* path
# ----------------------------------------------------------------------------
def follow_path(fig, body, path):
    """Move the body along `path` (orientation is kept constant here)."""
    wp = 1
    step = 0
    while wp < len(path):
        err = path[wp] - body.pos
        dist = np.linalg.norm(err)
        is_last = wp == len(path) - 1

        move = min(V_MAX * DT, dist)
        new_pos = body.pos + err / dist * move if dist > 1e-9 else body.pos.copy()
        body.move(new_pos)

        if dist <= (1e-3 if is_last else WAYPOINT_TOL):
            wp += 1

        step += 1
        if step % 3 == 0:
            body.refresh()
            fig.canvas.draw_idle()
            plt.pause(0.001)

    body.refresh()
    fig.canvas.draw_idle()
    plt.pause(0.001)


# ----------------------------------------------------------------------------
# Phase 2: precision feature (proportional controller on pose error)
# ----------------------------------------------------------------------------
def precision_control(ax, fig, body, pb):
    """
    Position: v = KP_POS * (goal - pos)                 (saturated at V_MAX)
              -> proportional to the Euclidean distance error.
    Angle:    w = KP_ANG * wrap(theta_goal - theta)     (saturated at OMEGA_MAX)

    Stops when the body center is inside the green (precision) rectangle AND the
    orientation error is below ANGLE_TOL.
    """
    status = ax.text(1.02, 0.0, "", transform=ax.transAxes, fontsize=10, va="bottom",
                     bbox=dict(boxstyle="round", fc="white", ec="black", alpha=0.9))
    t = 0.0
    step = 0
    converged = False
    while t < PRECISION_MAX_TIME:
        err_vec = pb.goal - body.pos
        dist = np.linalg.norm(err_vec)
        ang_err = wrap_angle(pb.theta_goal - body.theta)

        if in_rect(body.pos, pb.goal, pb.theta_goal, PRECISION_HALF) and abs(ang_err) <= ANGLE_TOL:
            converged = True
            break

        v = KP_POS * err_vec
        speed = np.linalg.norm(v)
        if speed > V_MAX:
            v *= V_MAX / speed
        w = np.clip(KP_ANG * ang_err, -OMEGA_MAX, OMEGA_MAX)

        body.move(body.pos + v * DT, body.theta + w * DT)
        t += DT
        step += 1

        if step % 2 == 0:
            body.refresh()
            status.set_text(f"Precision phase\n"
                            f"position error: {dist:.3f}\n"
                            f"angle error: {np.rad2deg(ang_err):.1f} deg")
            fig.canvas.draw_idle()
            plt.pause(0.001)

    body.refresh()
    return status, converged, t


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    rng = np.random.default_rng(SEED)
    pb = make_problem(rng)
    planner = RRTStar(rng, pb)
    fig, ax, tree_lc = setup_plot(pb)
    title = ax.set_title("RRT* - growing tree")

    # The rectangle starts at the (random) start with a random orientation
    theta0 = rng.uniform(-np.pi, np.pi)
    body = RectBody(ax, pb.start, theta0)
    put_legend(ax)
    print(f"Start : ({pb.start[0]:.2f}, {pb.start[1]:.2f}), "
          f"initial orientation {np.rad2deg(theta0):.1f} deg")
    print(f"Target: ({pb.goal[0]:.2f}, {pb.goal[1]:.2f}), "
          f"target orientation {np.rad2deg(pb.theta_goal):.1f} deg")

    # ---- Build the tree ------------------------------------------------------
    first_goal_iter = None
    for it in range(1, MAX_ITER + 1):
        planner.extend()

        if first_goal_iter is None and planner.goal_nodes:
            first_goal_iter = it
            print(f"First path to the final area found at iteration {it}")

        if it % DRAW_EVERY == 0:
            tree_lc.set_segments(planner.edge_segments())
            best = planner.best_goal_node()
            cost_txt = f" | best cost: {planner.nodes[best].cost:.2f}" if best is not None else ""
            title.set_text(f"RRT* - iteration {it}, nodes: {len(planner.nodes)}{cost_txt}")
            fig.canvas.draw_idle()
            plt.pause(0.001)

        if first_goal_iter is not None and it - first_goal_iter >= EXTRA_ITER_AFTER_GOAL:
            break

    tree_lc.set_segments(planner.edge_segments())

    best = planner.best_goal_node()
    if best is None:
        title.set_text("RRT* - no path found (try more iterations)")
        print("No path found. Increase MAX_ITER or STEP_SIZE.")
        plt.ioff()
        plt.show()
        return

    path = planner.extract_path(best)
    path_len = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    print(f"RRT* path: {len(path)} waypoints, length = {path_len:.2f}")

    # Highlight the final path
    ax.plot(path[:, 0], path[:, 1], "-", color="royalblue", lw=3, zorder=5,
            label=f"RRT* path (length {path_len:.1f})")
    ax.plot(path[:, 0], path[:, 1], "o", color="royalblue", ms=4, zorder=5)
    put_legend(ax)
    title.set_text(f"RRT* - final path (length {path_len:.2f}), executing...")
    fig.canvas.draw_idle()
    plt.pause(1.0)

    # ---- Phase 1: follow the RRT* path ---------------------------------------
    follow_path(fig, body, path)
    err_rrt = float(np.linalg.norm(pb.goal - body.pos))
    ang_rrt = wrap_angle(pb.theta_goal - body.theta)
    print(f"End of RRT* path: position error = {err_rrt:.3f}, "
          f"angle error = {np.rad2deg(ang_rrt):.1f} deg")

    # ---- Phase 2: precision feature ------------------------------------------
    title.set_text("Precision feature: converging to target pose...")
    plt.pause(0.5)
    status, converged, t_prec = precision_control(ax, fig, body, pb)

    # ---- Final errors ----------------------------------------------------------
    err_vec = pb.goal - body.pos
    err = float(np.linalg.norm(err_vec))
    ang_err = float(wrap_angle(pb.theta_goal - body.theta))
    in_precise = in_rect(body.pos, pb.goal, pb.theta_goal, PRECISION_HALF)

    print(f"Precision phase: {'converged' if converged else 'TIMEOUT'} in {t_prec:.2f} s")
    print(f"Final position : ({body.pos[0]:.3f}, {body.pos[1]:.3f})")
    print(f"Target position: ({pb.goal[0]:.3f}, {pb.goal[1]:.3f})")
    print(f"Position error : {err:.3f} (dx={err_vec[0]:.3f}, dy={err_vec[1]:.3f})")
    print(f"Orientation error: {np.rad2deg(ang_err):.2f} deg")
    print(f"Inside precision area: {in_precise}")

    title.set_text("Target pose reached" if converged else "Precision phase: timeout")
    status.set_text(f"Final position error: {err:.3f}\n"
                    f"Final orientation error: {np.rad2deg(ang_err):.2f} deg\n"
                    f"Inside precision area: {'yes' if in_precise else 'no'}")
    fig.canvas.draw_idle()

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()