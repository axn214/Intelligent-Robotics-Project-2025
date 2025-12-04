from controller import Robot
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
import matplotlib.text as mtext

# ---------------------------------------------------------
# TURTLEBOT PARAMETERS
WHEEL_RADIUS = 0.033
WHEEL_BASE   = 0.160
TIME_STEP    = 32
# ---------------------------------------------------------

robot = Robot()

# ----------------- LiDAR --------------------
try:
    lidar = robot.getDevice("LDS-01")
except:
    lidar = robot.getDevice("laser")

lidar.enable(TIME_STEP)
lidar.enablePointCloud()

fov = lidar.getFov()
res = lidar.getHorizontalResolution()
angle_step = fov / res
max_range = lidar.getMaxRange()

# ----------------- Encoders -----------------
left_enc  = robot.getDevice("left wheel sensor")
right_enc = robot.getDevice("right wheel sensor")

left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

# ----------------- Motors (for visualization, robot stays still) ----
left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

prev_left  = 0.0
prev_right = 0.0

# ----------------- Odometry -----------------
x = 0.0
y = 0.0
theta = 0.0


# ----------------- Clustering -----------------
def detect_obstacles(xs, ys, cluster_dist=0.25, min_points=5):
    clusters = []
    if len(xs) < 2:
        return clusters

    current = [(xs[0], ys[0])]

    for i in range(1, len(xs)):
        x1, y1 = xs[i-1], ys[i-1]
        x2, y2 = xs[i], ys[i]

        d = math.dist((x1, y1), (x2, y2))

        if d < cluster_dist:
            current.append((x2, y2))
        else:
            if len(current) >= min_points:
                clusters.append(current)
            current = [(x2, y2)]

    if len(current) >= min_points:
        clusters.append(current)

    return clusters


# ----------------- Matplotlib Setup ----------------
plt.ion()
fig, ax = plt.subplots()

scat = ax.scatter([], [], s=4, color='blue')

robot_body = Circle((0, 0), 0.15, fill=False, color='red', linewidth=2)
ax.add_patch(robot_body)

heading_arrow = FancyArrowPatch(
    (0, 0), (0.3, 0),
    arrowstyle='->',
    color='red',
    mutation_scale=15,
    linewidth=2
)
ax.add_patch(heading_arrow)

ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect("equal")
ax.set_title("TurtleBot3: LiDAR + Obstacle Detection + IDs")

# Maintain lists of circles + labels for cleanup
obstacle_circles = []
obstacle_labels = []

# ======================================================
# MAIN LOOP
# ======================================================
while robot.step(TIME_STEP) != -1:

    # ---------------- ODOMETRY ----------------
    left_pos  = left_enc.getValue()
    right_pos = right_enc.getValue()

    dl = (left_pos  - prev_left)  * WHEEL_RADIUS
    dr = (right_pos - prev_right) * WHEEL_RADIUS

    prev_left  = left_pos
    prev_right = right_pos

    dc     = (dl + dr) / 2
    dtheta = (dr - dl) / WHEEL_BASE

    x += dc * math.cos(theta + dtheta/2)
    y += dc * math.sin(theta + dtheta/2)
    theta += dtheta

    # ---------------- LIDAR POINTS ----------------
    ranges = lidar.getRangeImage()
    xs = []
    ys = []

    angle = -fov / 2

    for r in ranges:
        if r < max_range:
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            gx = x + lx * math.cos(theta) - ly * math.sin(theta)
            gy = y + lx * math.sin(theta) + ly * math.cos(theta)

            xs.append(gx)
            ys.append(gy)

        angle += angle_step

    pts = np.column_stack((xs, ys)) if len(xs) > 0 else np.zeros((0, 2))
    scat.set_offsets(pts)

    # ---------------- OBSTACLE CLUSTERING ----------------
    clusters = detect_obstacles(xs, ys)

    # Remove old circles + labels
    for c in obstacle_circles:
        c.remove()
    for lbl in obstacle_labels:
        lbl.remove()

    obstacle_circles = []
    obstacle_labels = []

    print("\n--- Detected Obstacles ---")
    obstacle_id = 1

    # ---------------- DRAW NEW OBSTACLE CIRCLES + LABELS ----------------
    for cluster in clusters:
        pts_x = [p[0] for p in cluster]
        pts_y = [p[1] for p in cluster]

        cx = sum(pts_x) / len(pts_x)
        cy = sum(pts_y) / len(pts_y)

        radius = max(math.dist((cx, cy), (px, py)) for px, py in cluster)

        print(f"Obstacle {obstacle_id}:  x={cx:.2f}, y={cy:.2f}, radius={radius:.2f}")

        circ = Circle((cx, cy), radius, fill=False, color='green', linewidth=2)
        ax.add_patch(circ)
        obstacle_circles.append(circ)

        # ID label
        label = ax.text(cx, cy + radius + 0.1, f"ID {obstacle_id}",
                        color='black', fontsize=10, ha='center')
        obstacle_labels.append(label)

        obstacle_id += 1

    if len(clusters) == 0:
        print("No obstacles detected.")

    # ---------------- UPDATE ROBOT GRAPHICS ----------------
    robot_body.center = (x, y)

    heading_arrow.set_positions(
        (x, y),
        (x + 0.4 * math.cos(theta),
         y + 0.4 * math.sin(theta))
    )

    plt.draw()
    plt.pause(0.001)

