# controller_vo_with_viz.py
# Single-file Webots controller:
# - LiDAR -> clustering -> obstacle centers
# - Odometry
# - Go-to-goal + static VO avoidance
# - Matplotlib visualization of LiDAR, obstacles, VO cones, robot velocity and goal

from controller import Robot
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch

# ----------------- PARAMETERS -----------------
WHEEL_RADIUS = 0.033
WHEEL_BASE   = 0.160
TIME_STEP    = 32  # ms
ROBOT_RADIUS = 0.15  # used to inflate obstacles for safety
MAX_WHEEL_SPEED = 6.67

# Go-to-goal params
GOAL_X = 10.0
GOAL_Y = 0.0
V_NOMINAL = 0.25
W_GAIN = 2.0

# Clustering params
CLUSTER_DIST = 0.25
CLUSTER_MIN_POINTS = 5

# VO parameters
SLOWDOWN_FACTOR = 0.25
TURN_BIAS = 1.2   # angular bias when avoiding

# ----------------- ROBOT SETUP -----------------
robot = Robot()

# Try both names for different robots
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

left_enc  = robot.getDevice("left wheel sensor")
right_enc = robot.getDevice("right wheel sensor")
left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ----------------- ODOMETRY STATE -----------------
x = 0.0
y = 0.0
theta = 0.0
prev_left = 0.0
prev_right = 0.0

# ----------------- Matplotlib (interactive) -----------------
plt.ion()
fig, ax = plt.subplots(figsize=(7,7))
scat = ax.scatter([], [], s=6, color='blue', label='LiDAR points')

robot_body = Circle((0, 0), ROBOT_RADIUS, fill=False, color='red', linewidth=2, label='Robot')
ax.add_patch(robot_body)

heading_arrow = FancyArrowPatch((0,0),(0.4,0), arrowstyle='->', color='red', mutation_scale=15, linewidth=2)
ax.add_patch(heading_arrow)

vel_arrow = FancyArrowPatch((0,0),(0,0), arrowstyle='->', color='magenta', mutation_scale=12, linewidth=2, label='Velocity')
ax.add_patch(vel_arrow)

goal_marker, = ax.plot(GOAL_X, GOAL_Y, marker='*', color='orange', markersize=12, label='Goal')

ax.set_xlim(-6, 12)
ax.set_ylim(-8, 8)
ax.set_aspect("equal")
ax.set_title("TurtleBot: LiDAR - Clusters - VO Cones - GOAL")
ax.legend(loc='upper right')

obstacle_circles = []
obstacle_labels = []
cone_lines = []

# ----------------- Utility functions -----------------
def detect_obstacles(xs, ys, cluster_dist=CLUSTER_DIST, min_points=CLUSTER_MIN_POINTS):
    clusters = []
    if len(xs) < 2:
        return clusters

    current = [(xs[0], ys[0])]
    for i in range(1, len(xs)):
        x1, y1 = xs[i-1], ys[i-1]
        x2, y2 = xs[i], ys[i]
        d = math.dist((x1,y1), (x2,y2))
        if d < cluster_dist:
            current.append((x2,y2))
        else:
            if len(current) >= min_points:
                clusters.append(current)
            current = [(x2,y2)]
    if len(current) >= min_points:
        clusters.append(current)
    return clusters

def update_odometry():
    global x, y, theta, prev_left, prev_right
    left_pos = left_enc.getValue()
    right_pos = right_enc.getValue()

    dl = (left_pos - prev_left) * WHEEL_RADIUS
    dr = (right_pos - prev_right) * WHEEL_RADIUS
    prev_left = left_pos
    prev_right = right_pos

    dc = (dl + dr)/2.0
    dtheta = (dr - dl) / WHEEL_BASE

    x += dc * math.cos(theta + dtheta/2.0)
    y += dc * math.sin(theta + dtheta/2.0)
    theta += dtheta
    theta = math.atan2(math.sin(theta), math.cos(theta))

def is_collision_course_static(ox, oy, v_forward, robot_x, robot_y, robot_theta, inflated_R):
    """
    static obstacle VO test: returns True if (robot forward velocity) points into VO
    """
    dx = ox - robot_x
    dy = oy - robot_y
    d = math.hypot(dx, dy)
    # already overlapping or nearly zero motion -> treat as collision for safety
    if d <= inflated_R or v_forward < 1e-3:
        return True

    lam = math.asin(min(1.0, inflated_R / d))  # half-angle
    # robot velocity vector (world frame)
    vx = v_forward * math.cos(robot_theta)
    vy = v_forward * math.sin(robot_theta)
    dot = vx*dx + vy*dy
    mag_v = math.hypot(vx, vy)
    lhs = dot / (mag_v * d)

    return lhs > math.cos(lam)

def signed_angle_between_p_and_heading(dx, dy, theta):
    # signed angle from vector p=(dx,dy) to robot heading direction (cosθ, sinθ)
    # we'll compute angle(p) - heading so positive means p is left of heading
    ang_p = math.atan2(dy, dx)
    ang_h = theta
    diff = math.atan2(math.sin(ang_p - ang_h), math.cos(ang_p - ang_h))
    return diff

# ----------------- Main loop -----------------
print("Controller started. Goal:", (GOAL_X, GOAL_Y))
while robot.step(TIME_STEP) != -1:
    # --- ODOMETRY ---
    update_odometry()

    # --- LIDAR read + world transform ---
    ranges = lidar.getRangeImage()
    xs = []
    ys = []
    angle = -fov/2.0
    for r in ranges:
        if r < max_range:
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)
            # transform into world frame
            gx = x + lx * math.cos(theta) - ly * math.sin(theta)
            gy = y + lx * math.sin(theta) + ly * math.cos(theta)
            xs.append(gx)
            ys.append(gy)
        angle += angle_step

    pts = np.column_stack((xs, ys)) if len(xs) > 0 else np.zeros((0,2))
    scat.set_offsets(pts)

    # --- Clustering for obstacles (in world frame) ---
    clusters = detect_obstacles(xs, ys)

    # cleanup old drawing
    for c in obstacle_circles:
        try:
            c.remove()
        except:
            pass
    for lbl in obstacle_labels:
        try:
            lbl.remove()
        except:
            pass
    for ln in cone_lines:
        try:
            ln.remove()
        except:
            pass

    obstacle_circles = []
    obstacle_labels = []
    cone_lines = []

    # Build obstacle list for controller
    obstacles_for_vo = []

    # Print header
    print("\n--- Detected Obstacles ---")

    oid = 1
    for cluster in clusters:
        pts_x = [p[0] for p in cluster]
        pts_y = [p[1] for p in cluster]
        cx = sum(pts_x) / len(pts_x)
        cy = sum(pts_y) / len(pts_y)
        radius = max(math.dist((cx,cy),(px,py)) for px,py in cluster)

        print(f"ID {oid}: x={cx:.2f}, y={cy:.2f}, r={radius:.2f}")

        circ = Circle((cx, cy), radius, fill=False, color='green', linewidth=2)
        ax.add_patch(circ)
        obstacle_circles.append(circ)

        lbl = ax.text(cx, cy + radius + 0.08, f"ID {oid}", color='black', fontsize=9, ha='center')
        obstacle_labels.append(lbl)

        obstacles_for_vo.append((cx, cy, radius))
        oid += 1

    if len(clusters) == 0:
        print("No obstacles detected.")

    # --- Go-to-goal baseline control (compute v,w before VO) ---
    dx_goal = GOAL_X - x
    dy_goal = GOAL_Y - y
    dist_goal = math.hypot(dx_goal, dy_goal)
    heading_goal = math.atan2(dy_goal, dx_goal)

    if dist_goal < 0.15:
        # reached
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("Reached goal.")
        break

    heading_err = math.atan2(math.sin(heading_goal - theta), math.cos(heading_goal - theta))
    v_cmd = V_NOMINAL
    w_cmd = W_GAIN * heading_err

    # --- VO avoidance (static obstacles) ---
    # check each obstacle; if any returns True, apply avoidance adjustments
    for (ox, oy, oradius) in obstacles_for_vo:
        inflated_R = ROBOT_RADIUS + oradius
        collision = is_collision_course_static(ox, oy, v_cmd, x, y, theta, inflated_R)
        # visualize cone even if collision False (helps debugging)
        # compute lambda and cone boundary rays in world coords (start at robot pos)
        dx_o = ox - x
        dy_o = oy - y
        d_o = math.hypot(dx_o, dy_o)
        if d_o > 1e-6:
            lam = math.asin(min(1.0, inflated_R / d_o))
            p_hat = np.array([dx_o/d_o, dy_o/d_o])
            # left boundary
            rotL = np.array([[math.cos(+lam), -math.sin(+lam)],[math.sin(+lam), math.cos(+lam)]])
            rotR = np.array([[math.cos(-lam), -math.sin(-lam)],[math.sin(-lam), math.cos(-lam)]])
            bL = rotL.dot(p_hat)
            bR = rotR.dot(p_hat)
            cone_len = max(0.8, min(3.0, d_o))  # draw to reasonable length
            lineL, = ax.plot([x, x + bL[0]*cone_len], [y, y + bL[1]*cone_len], color='cyan', linewidth=1.5, alpha=0.8)
            lineR, = ax.plot([x, x + bR[0]*cone_len], [y, y + bR[1]*cone_len], color='cyan', linewidth=1.5, alpha=0.8)
            cone_lines.append(lineL); cone_lines.append(lineR)

        # if collision predicted, steer away (add angular bias + slow)
        if collision:
            print("VO: avoid -> obstacle at:", (ox,oy))
            alpha_signed = signed_angle_between_p_and_heading(dx_o, dy_o, theta)
            if alpha_signed > 0:
                w_cmd += TURN_BIAS
            else:
                w_cmd -= TURN_BIAS
            v_cmd *= SLOWDOWN_FACTOR

    # --- Convert (v_cmd, w_cmd) to wheel speeds and publish ---
    omega_l = (2.0*v_cmd - w_cmd*WHEEL_BASE) / (2.0*WHEEL_RADIUS)
    omega_r = (2.0*v_cmd + w_cmd*WHEEL_BASE) / (2.0*WHEEL_RADIUS)
    omega_l = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, omega_l))
    omega_r = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, omega_r))

    left_motor.setVelocity(omega_l)
    right_motor.setVelocity(omega_r)

    # --- Update robot drawing (body, heading, velocity arrow) ---
    robot_body.center = (x, y)
    heading_arrow.set_positions((x, y), (x + 0.35*math.cos(theta), y + 0.35*math.sin(theta)))
    vel_arrow.remove()  # remove old and add new (easier to manage)
    vel_arrow = FancyArrowPatch((x, y), (x + 0.5*v_cmd*math.cos(theta), y + 0.5*v_cmd*math.sin(theta)),
                                arrowstyle='->', color='magenta', mutation_scale=12, linewidth=2)
    ax.add_patch(vel_arrow)

    # re-draw goal marker (in case limits/scale change)
    goal_marker.set_data(GOAL_X, GOAL_Y)

    plt.draw()
    plt.pause(0.001)

# end loop
print("Controller finished.")

