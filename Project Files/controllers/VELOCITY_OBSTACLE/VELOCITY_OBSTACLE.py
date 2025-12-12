from controller import Robot, Keyboard
import math
import numpy as np
import random

# ----------------- ROBOT INITIALISATION -----------------
robot = Robot()

# ----------------- GENERIC PARAMETERS -----------------
WHEEL_RADIUS = 0.033
DISTANCE_BETWEEN_WHEELS = 0.160
ROBOT_RADIUS = 0.15  # used to inflate obstacles for safety
MAX_WHEEL_SPEED = 6.67
V_NOMINAL = 0.5
W_GAIN = 2.0
TIME_STEP = int(robot.getBasicTimeStep())

K_D = 0.2 # Linear Velocity Strength Constant
K_T = 10 # Angular Velocity Strength Constant

# ----------------- CLUSTERING PARAMETERS -----------------
CLUSTER_DIST = 0.25
CLUSTER_MIN_POINTS = 5

# ----------------- OBSTACLE AVOIDANCE PARAMETERS -----------------
SLOWDOWN_FACTOR = 0.25
TURN_BIAS = 1.5   # angular bias when avoiding

# Keyboard for testing in different cases
kb = Keyboard()
kb.enable(TIME_STEP)

# ----------------- DEVICE SETUP -----------------
lidar = robot.getDevice("LDS-01")
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
fov = lidar.getFov()
res = lidar.getHorizontalResolution()
angle_step = fov / res
max_range = lidar.getMaxRange()

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

# Simulate 1 time step for device initialisation
robot.step(TIME_STEP)

# Create an array to store robot pose ([x, y, theta])
pose = []

# Variables to store the old encoder values (previous timestep)
left_encoder_old = left_encoder.getValue()
right_encoder_old = right_encoder.getValue()

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

def calculate_accurate_pose():
    
    pos = gps.getValues()
    x = pos[0]
    y = pos[1]

    orientation = compass.getValues()

    theta = math.atan2(orientation[0], orientation[1])
    
    pose = [x, y, theta]

    return pose

def calculate_velocities(distance_to_goal, angular_displacement):
    
    # Compute the turning angle required to reach the goal
    turning_angle = angular_displacement - pose[2]
    turning_angle = (turning_angle + math.pi) % (2 * math.pi) - math.pi
    
    
    # Compute linear and angular velocities required to reach the goal
    linear_velocity = K_D * distance_to_goal * math.cos(turning_angle)
    # limit linear speed so it doesn't explode for far goals
    linear_velocity = min(V_NOMINAL, linear_velocity)
    
    angular_velocity = K_T * turning_angle

    # Compute and set the left and right wheel velocities 
    
    linear_velocity_left = linear_velocity - ((DISTANCE_BETWEEN_WHEELS / 2) * angular_velocity)
    linear_velocity_right = linear_velocity +  ((DISTANCE_BETWEEN_WHEELS / 2) * angular_velocity)
    
    # In rad/s
    angular_velocity_left = linear_velocity_left / WHEEL_RADIUS
    angular_velocity_right = linear_velocity_right / WHEEL_RADIUS
    
    return [angular_velocity_left, angular_velocity_right]


def compute_current_pose(angular_velocity_left, angular_velocity_right):
    
    # Update pose (pose = [x,y,theta])
    
    global left_encoder_old
    global right_encoder_old
    velocity_right = angular_velocity_right * WHEEL_RADIUS
    velocity_left = angular_velocity_left * WHEEL_RADIUS
    
    left_encoder_new = left_encoder.getValue()
    right_encoder_new = right_encoder.getValue()
    
    # First we need to calcualte the distance travelled by each wheel (chnage in distance)
    delta_left = (left_encoder_new - left_encoder_old) * WHEEL_RADIUS
    delta_right = (right_encoder_new - right_encoder_old) * WHEEL_RADIUS
    
    # Store current position sensor values for the next iteration
    left_encoder_old = left_encoder_new
    right_encoder_old = right_encoder_new
    
    #Calculate the robots avg displacement and orientation change
    delta_avg = (delta_left + delta_right) / 2
    delta_orientation = (delta_right - delta_left) / DISTANCE_BETWEEN_WHEELS
    orientation = pose[2] + delta_orientation

    # Normalise orientation to [-pi,pi]
    if orientation > math.pi:
        pose[2] = orientation - (2 * math.pi)
    elif orientation < -math.pi:
        pose[2] = orientation + (2 * math.pi)
    else:
        pose[2] = orientation
    
    theta_mid = pose[2] - delta_orientation / 2
    delta_x = delta_avg * math.cos(theta_mid)
    delta_y = delta_avg * math.sin(theta_mid)
    
    # Store new x and y coordinates
    pose[0] += delta_x
    pose[1] += delta_y

def generate_goals():
    # Initialise goal position array
    goal_positions = [(3.8, 3.99), (-3.06, 3.99), (1.17, 1.28), (3.65, -3.7), (1.45, -3.7), (1.45, 0.43), (3.29, -1.62), (0.15, -1.62), (1.09, 3.67), (-2.41, -1.12)]
    selected_goal_positions = []

    for i in range (5):
        # Ensure we get a unique goal position
        unique = False
        
        while not(unique):
            selection = random.choice(goal_positions)
            if not (selection in selected_goal_positions):
                unique = True
             
        selected_goal_positions.append(selection)
    
    return selected_goal_positions

def calculate_exact_coords_for_testing():
    
    pos = gps.getValues()
    x = pos[0]
    y = pos[1]

    return (x, y)

# Function implemented for testing
def update_path_length(path_length, prev_pos):
    pos = gps.getValues()
    if prev_pos is None:
        prev_pos = pos
        
    else:
        dx = pos[0] - prev_pos[0]
        dy = pos[1] - prev_pos[1]

        step_distance = math.sqrt(dx**2 + dy**2)
        path_length += step_distance
        prev_pos = pos

    return path_length, prev_pos

# ----------------- Main loop -----------------


# Create an array to store the robot's predicted finishing locations (for testing)
finishing_positions = []

# Create an array to store the robot's actual finishing locations (for testing)
actual_finishing_positions = []

# Create arrays to store the path length's to the goal and the time taken (for testing)
path_lengths = []
times_taken = []

# Gets the starting condition
valid = False
singleExecution = False

print('Please press \"1\" for \"Multiple Goal Positions\" or \"2\" for \"Single Simulation Execution\"')
while not (valid):

    # Get control key (for testing)
    key = kb.getKey()
    if key == ord('1'):
        print("Starting execution for multiple goal positions")
        # Generate the goal positions
        goal_positions = generate_goals()   
        valid = True
        
    elif key == ord('2'):
        print("Starting single execution")
        goal_positions = [(4,4)]
        valid = True
        singleExecution = True

    robot.step(TIME_STEP)

# Calculate initial pose using compass and gps as odoemtry won't work yet
pose = calculate_accurate_pose()

for goal_position in goal_positions:
    print('Navigating to goal', goal_position)
    
    GOAL_X = goal_position[0]
    GOAL_Y = goal_position[1]
    
    
    path_length = 0 # Length of the path travelled to goal (for testing) 
    prev_pos = None # Previous coordinate for path length calculatios (for testing)
    no_timesteps = 0 # Stores the number of timesteps executed to reach the destination (for testing)
    starting_position = (pose[0], pose[1]) # Stores the number of timesteps executed to reach the destination (for testing)

    while robot.step(TIME_STEP) != -1:
        
        # --- LIDAR read + world transform ---
        ranges = lidar.getRangeImage()
        xs = []
        ys = []
        angle = -fov/2.0
        
        x = pose[0]
        y = pose[1]
        theta = pose[2]
        
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

        # --- Clustering for obstacles (in world frame) ---
        clusters = detect_obstacles(xs, ys)

        # Build obstacle list for controller
        obstacles_for_vo = []

        oid = 1
        for cluster in clusters:
            pts_x = [p[0] for p in cluster]
            pts_y = [p[1] for p in cluster]
            cx = sum(pts_x) / len(pts_x)
            cy = sum(pts_y) / len(pts_y)
            radius = max(math.dist((cx,cy),(px,py)) for px,py in cluster)

            print(f"Obstacle {oid}: (x={cx:.2f}, y={cy:.2f}, r={radius:.2f})")

            obstacles_for_vo.append((cx, cy, radius))
            oid += 1

        if len(clusters) == 0:
            print("No Obstacles Detected.")

        # --- Go-to-goal baseline control (compute v,w before VO) ---
        dx_goal = GOAL_X - pose[0]
        dy_goal = GOAL_Y - pose[1]
        dist_goal = math.hypot(dx_goal, dy_goal)
        heading_goal = math.atan2(dy_goal, dx_goal)

        # Update the path_length
        path_length, prev_pos = update_path_length(path_length, prev_pos)
        no_timesteps += 1
        
        if dist_goal < 0.01:
            # reached
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("Goal Reached!")
            
            # Store the location the robot finished at for testing later
            finishing_positions.append((pose[0], pose[1]))
            actual_finishing_positions.append(calculate_exact_coords_for_testing())
            path_lengths.append(path_length)
            times_taken.append(no_timesteps * TIME_STEP)
            break

        heading_err = math.atan2(math.sin(heading_goal - theta), math.cos(heading_goal - theta))
        v_cmd = V_NOMINAL
        w_cmd = W_GAIN * heading_err

        # Odometric Movement Code (Defines what the robot will do when there are no detected obstacles)
                
        # Compute displacement from goal
        delta_x = goal_position[0] - pose[0]
        delta_y = goal_position[1] - pose[1]
                
        distance_to_goal = math.sqrt((delta_x**2) + (delta_y**2))
        
        angular_displacement = math.atan2(delta_y, delta_x)
        
        angular_velocity_left, angular_velocity_right = calculate_velocities(distance_to_goal, angular_displacement)

        # --- VO avoidance (static obstacles) ---
        # check each obstacle; if any returns True, apply avoidance adjustments
        for (ox, oy, oradius) in obstacles_for_vo:
            inflated_R = ROBOT_RADIUS + oradius
            collision = is_collision_course_static(ox, oy, v_cmd, pose[0], pose[1], pose[2], inflated_R)

            # compute lambda and cone boundary rays in world coords (start at robot pos)
            dx_o = ox - pose[0]
            dy_o = oy - pose[1]
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
            
            # if collision predicted, steer away (add angular bias + slow)
            if collision:
                print("Performing evasive manuevers... Obstacle's postion:", (ox,oy))

                alpha_signed = signed_angle_between_p_and_heading(dx_o, dy_o, theta)
                if alpha_signed > 0:
                    w_cmd += TURN_BIAS
                else:
                    w_cmd -= TURN_BIAS
                v_cmd *= SLOWDOWN_FACTOR

                # --- Convert (v_cmd, w_cmd) to wheel speeds and publish ---
                omega_l = (2.0*v_cmd - w_cmd*DISTANCE_BETWEEN_WHEELS) / (2.0*WHEEL_RADIUS)
                omega_r = (2.0*v_cmd + w_cmd*DISTANCE_BETWEEN_WHEELS) / (2.0*WHEEL_RADIUS)
                
                angular_velocity_left = omega_l
                angular_velocity_right = omega_r
        
        # Update Robot Pose
        compute_current_pose(angular_velocity_left, angular_velocity_right)

        # Sets the velocity for the left and right motors
        left_motor.setVelocity(angular_velocity_left)
        right_motor.setVelocity(angular_velocity_right)
    
        pass

print("Controller finished!")

if singleExecution:
    for i in range (len(goal_positions)):
        print(f'Goal {i+1}:\n')
        print(f'Starting Position: {starting_position}\n')
        print(f'\tTarget:', goal_positions[i])
        
        print(f'\tEstimate Destination:', finishing_positions[i])
        print(f'\tActual Destination:', actual_finishing_positions[i])
        print(f'\tError in x (estimated):', abs(finishing_positions[i][0] - goal_positions[i][0]))
        print(f'\tError in y (estimated):', abs(finishing_positions[i][1] - goal_positions[i][1]))
        print(f'\tError in x (true):', abs(actual_finishing_positions[i][0] - goal_positions[i][0]))
        print(f'\tError in y (true):', abs(actual_finishing_positions[i][1] - goal_positions[i][1]))
        print(f'\nPath Length = {path_lengths[i]} metres')
        print(f'\nTime Taken = {times_taken[i] / 1000}')
else:
    for i in range (len(goal_positions)):
        print(f'Goal {i+1}:\n')
        print(f'\tTarget:', goal_positions[i])
        print(f'\tEstimate Destination:', finishing_positions[i])
        print(f'\tActual Destination:', actual_finishing_positions[i])
        print(f'\tError in x (estimated):', abs(finishing_positions[i][0] - goal_positions[i][0]))
        print(f'\tError in y (estimated):', abs(finishing_positions[i][1] - goal_positions[i][1]))
        print(f'\tError in x (true):', abs(actual_finishing_positions[i][0] - goal_positions[i][0]))
        print(f'\tError in y (true):', abs(actual_finishing_positions[i][1] - goal_positions[i][1]))
        
        # Caclulate euclidean distance
        d = np.linalg.norm(np.array(finishing_positions[i]) - np.array(goal_positions[i]))
        print(f'\tEuclidean distance between Target and Estimated Destination = ', d)

        d = np.linalg.norm(np.array(actual_finishing_positions[i]) - np.array(goal_positions[i]))
        print(f'\tEuclidean distance between Target and Actual Destination = ', d)


