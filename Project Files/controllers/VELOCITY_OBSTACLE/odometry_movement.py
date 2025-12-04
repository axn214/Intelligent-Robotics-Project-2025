# controller_odometry_gotogoal.py
# Team Controller 1: Wheel odometry + Position estimation + Go-to-goal

from controller import Robot
import math

class OdomGoToGoal:
    def __init__(self, robot,
                 wheel_radius=0.033,
                 wheel_base=0.160,
                 max_wheel_speed=6.67,
                 time_step=32):

        self.robot = robot
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_wheel_speed = max_wheel_speed
        self.time_step = time_step

        # Motors + Encoders
        self.left_motor = robot.getDevice("left wheel motor")
        self.right_motor = robot.getDevice("right wheel motor")
        self.left_enc  = robot.getDevice("left wheel sensor")
        self.right_enc = robot.getDevice("right wheel sensor")

        self.left_enc.enable(time_step)
        self.right_enc.enable(time_step)

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = 0.0
        self.prev_right = 0.0

    # -----------------------------
    #   Update odometry each step
    # -----------------------------
    def update_odometry(self):
        left_pos = self.left_enc.getValue()
        right_pos = self.right_enc.getValue()

        dl = (left_pos - self.prev_left) * self.wheel_radius
        dr = (right_pos - self.prev_right) * self.wheel_radius

        self.prev_left = left_pos
        self.prev_right = right_pos

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.wheel_base

        # small rotation midpoint approximation
        self.x += dc * math.cos(self.theta + dtheta/2.0)
        self.y += dc * math.sin(self.theta + dtheta/2.0)
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    # -----------------------------
    #  PUBLIC API #1: Get position
    # -----------------------------
    def get_position(self):
        """Return current (x, y, theta)."""
        return self.x, self.y, self.theta

    # -----------------------------
    #  PUBLIC API #2: Go to goal
    # -----------------------------
    def go_to_goal(self, gx, gy, v_max=0.25, w_gain=2.0, tol=0.15):
        """
        Drive toward (gx, gy) until within tolerance.
        Call inside main loop.
        """

        # update odometry first
        self.update_odometry()

        # compute goal heading + distance
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)
        theta_g = math.atan2(dy, dx)

        if dist < tol:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
            return True  # reached goal

        # heading error
        e = math.atan2(math.sin(theta_g - self.theta), math.cos(theta_g - self.theta))

        # simple controller
        v = v_max
        w = w_gain * e

        # convert to wheel speeds
        omega_l = (2*v - w*self.wheel_base) / (2*self.wheel_radius)
        omega_r = (2*v + w*self.wheel_base) / (2*self.wheel_radius)

        # saturate
        omega_l = max(-self.max_wheel_speed, min(self.max_wheel_speed, omega_l))
        omega_r = max(-self.max_wheel_speed, min(self.max_wheel_speed, omega_r))

        self.left_motor.setVelocity(omega_l)
        self.right_motor.setVelocity(omega_r)

        return False  # not yet reached
        
robot = Robot()
ctrl = OdomGoToGoal(robot)

goal_x = 10.0
goal_y = 0.0
TIME_STEP = 32

print(f"Goal set to: ({goal_x}, {goal_y})")

# Main loop
while robot.step(TIME_STEP) != -1:

    reached = ctrl.go_to_goal(goal_x, goal_y)
    x, y, th = ctrl.get_position()

    print(f"Pose: x={x:.2f}, y={y:.2f}, theta={th:.2f}")

    if reached:
        print("Reached goal!")
        break


