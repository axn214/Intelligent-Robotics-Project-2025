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
K_T = 0.2

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
    
    #NEW 1
    # x = pose[0]
    # y = pose[1]
    # theta = pose[2]

    # R = (DISTANCE_BETWEEN_WHEELS / 2) * ((velocity_left + velocity_right) / (velocity_right - velocity_left))

    # icr = [x - R * math.sin(theta), y + R * math.cos(theta)]

    
    # delta_t = timestep / 1000
    
    # x_new = ((math.cos(angular_velocity * delta_t) * (x - icr[0])) - (math.sin(angular_velocity * delta_t) * (y - icr[1]))) + icr[0]
    # y_new = ((math.sin(angular_velocity * delta_t) * (x - icr[0])) + (math.cos(angular_velocity * delta_t) * (y - icr[1]))) + icr[1]
    # theta_new = theta + (angular_velocity * delta_t)
    
    # print('theta_new = ', theta_new)

    # Compare with compass
    # orientation = compass.getValues()
    # theta = math.atan2(orientation[0], orientation[1])
    # print("Compass: ", theta)

    # pose[0] = x_new
    # pose[1] = y_new
    # pose[2] = theta_new

    #NEW END 1
    
    # theta
    # pose[2] += (dr - dl) / 0.095
    
    # # normalise orientation pose 
    # pose[2] = (pose[2] + math.pi) % (2 * math.pi) - math.pi
    
    # theta = pose[2]
    # # x
    # pose[0] += (2*math.sin(theta/2)) * (db/theta + 0.0675)
    
    # # y
    # pose[1] += (2*math.sin(theta/2)) * (dr/theta + 0.0475)
    
    # print("dl = ", dl)
    # print("dr = ", dr)
    # print("Current Pose = ", pose)

# Calculate initial pose using compass and gps as odoemtry won't work yet
pose = calculate_accurate_pose()

goal_positions = []
for i in range (5):
    # Initialise goal position
    goal_position = generate_random_goal_position()
    print("Goal = ", goal_position)
    goal_positions.append(goal_position)
    
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
          
        # Stopping condition
        if distance_to_goal <= 0.01:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            accuratePose = calculate_accurate_pose()
            goal_positions[i] = [goal_positions[i], accuratePose]
            # print(f'\n\n\n\nCurrent location is {accuratePose}\n\nThe estimated location is {pose}')
            # print("Goal = ", goal_position)
            break
            
        pass
        
print("Results:\n\n\n")
for i in range (len(goal_positions)):
    goal = goal_positions[i][0]
    robotPose = goal_positions[i][1]
    print(f"Iteration {i+1}:\n\tGoal Position = {goal}\n\tRobot Pose = {robotPose}")
            
        

# Enter here exit cleanup code.
