"""movement_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import random
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# Initialise an array for robot pose ([x, y, theta])
pose = []

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Declare Linear Velocity Strength Constant
K_D = 0.2

# Declaur Angular Velocity Strength Constant
K_T = 10

WHEEL_RADIUS = 0.033

DISTANCE_BETWEEN_WHEELS = 0.160

# Initialise sensors
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

# back_encoder = robot.getDevice('rear position sensor')
# back_encoder.enable(timestep)

# left_encoder = robot.getDevice('left position sensor')
# left_encoder.enable(timestep)

# right_encoder = robot.getDevice('right position sensor')
# right_encoder.enable(timestep)

left_encoder = robot.getDevice('left wheel sensor')
left_encoder.enable(timestep)
right_encoder = robot.getDevice('right wheel sensor')
right_encoder.enable(timestep)

# Initialise motors
left_motor = robot.getDevice('left wheel motor')

right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0)
right_motor.setVelocity(0)

robot.step(timestep)


# Initialise goal position to x, y coordinates
# goal_position = [2, -2]

# Create global variables to store the old encoder values (previous timestep)
left_encoder_old = left_encoder.getValue()
right_encoder_old = right_encoder.getValue()

def generate_random_goal_position():
    # Generates a list of 2 random floating point numbers for the goal position
    return [random.uniform(-2.5,2.5) for _ in range(2)]

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

    angular_velocity = K_T * turning_angle

    # Compute and set the left and right wheel velocities 
    
    linear_velocity_left = linear_velocity - ((DISTANCE_BETWEEN_WHEELS / 2) * angular_velocity)
    linear_velocity_right = linear_velocity +  ((DISTANCE_BETWEEN_WHEELS / 2) * angular_velocity)
    
    # In rad/s
    angular_velocity_left = linear_velocity_left / WHEEL_RADIUS
    angular_velocity_right = linear_velocity_right / WHEEL_RADIUS
    
    # Convert to rev/s
    angular_velocity_left = angular_velocity_left / (2 * math.pi)
    angular_velocity_right = angular_velocity_right / (2 * math.pi)

    print("Angular Velocity Left = ", angular_velocity_left)
    print("Angular Velocity Right = ", angular_velocity_right)

    return [angular_velocity_left, angular_velocity_right]

def compute_current_pose(angular_velocity_left, angular_velocity_right):
    
    global left_encoder_old
    global right_encoder_old
    velocity_right = angular_velocity_right * WHEEL_RADIUS
    velocity_left = angular_velocity_left * WHEEL_RADIUS

    angular_velocity = (velocity_right - velocity_left) / DISTANCE_BETWEEN_WHEELS
    # NEW 2
    
    left_encoder_new = left_encoder.getValue()
    right_encoder_new = right_encoder.getValue()

    # rev_left = (left_encoder_new - left_encoder_old) / (2 * math.pi) 
    # rev_right = (right_encoder_new - right_encoder_old) / (2 * math.pi) 
    
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
    
    otest = compass.getValues()
    theta = math.atan2(otest[0], otest[1])
    print(f'Current estimated pose is: {pose}')
    print(f'Compass: {theta}, GPS coordinates: ({gps.getValues()[0]},{gps.getValues()[1]})')

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

# Create an array to store the robot's predicted finishing locations (for testing)
finishing_position = None

# Create an array to store the robot's actual finishing locations (for testing)
actual_finishing_position = None

# Calculate initial pose using compass and gps as odoemtry won't work yet
pose = calculate_accurate_pose()

# Initialise goal position
goal_position = (4, 4)
print("Goal = ", goal_position)

no_timesteps = 0 # Store number of time steps for timing the journey (for testing)
starting_position = (pose[0], pose[1]) # Stores the number of timesteps executed to reach the destination (for testing)
path_length = 0 # Length of the path travelled to goal (for testing) 
prev_pos = None # Previous coordinate for path length calculatios (for testing)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # Compute displacement from goal
    delta_x = goal_position[0] - pose[0]
    delta_y = goal_position[1] - pose[1]
    
    distance_to_goal = math.sqrt((delta_x**2) + (delta_y**2))

    angular_displacement = math.atan2(delta_y, delta_x)
    
    angular_velocity_left, angular_velocity_right = calculate_velocities(distance_to_goal, angular_displacement)

    compute_current_pose(angular_velocity_left, angular_velocity_right)

    # Set wheel velocities
    left_motor.setVelocity(angular_velocity_left)
    right_motor.setVelocity(angular_velocity_right)
    
    # Update the path_length
    path_length, prev_pos = update_path_length(path_length, prev_pos)

    no_timesteps += 1 # Increment no. timesteps

    # Stopping condition
    if distance_to_goal <= 0.01:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
        # Store the location the robot finished at for testing later
        finishing_position = (pose[0], pose[1])
        actual_finishing_position = calculate_exact_coords_for_testing()
        time_taken = no_timesteps * timestep
        accuratePose = calculate_accurate_pose()
    
        break
        
    pass
        
print("Results:\n\n\n")

print(f'Starting Position: {starting_position}\n')
print(f'\tTarget:', goal_position)
    
print(f'\tEstimate Destination:', finishing_position)
print(f'\tActual Destination:', actual_finishing_position)
print(f'\tError in x (estimated):', abs(finishing_position[0] - goal_position[0]))
print(f'\tError in y (estimated):', abs(finishing_position[1] - goal_position[1]))
print(f'\tError in x (true):', abs(actual_finishing_position[0] - goal_position[0]))
print(f'\tError in y (true):', abs(actual_finishing_position[1] - goal_position[1]))
print(f'\nPath Length = {path_length} metres')
print(f'\nTime Taken = {time_taken / 1000}')
        

# Enter here exit cleanup code.
