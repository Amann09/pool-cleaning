from controller import Robot, DistanceSensor, Motor, Gyro
import random
import math

# Time in [ms] of a simulation step
TIME_STEP = 64
MAX_SPEED = 6.28

# Initialize the Robot instance
robot = Robot()

# Initialize the Gyroscope
gyro = robot.getDevice('gyro')
gyro.enable(TIME_STEP)

# Initialize the Distance Sensors
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for name in psNames:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# Initialize the Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# State variables for normal behavior
turning = False
turn_direction = 1  # 1 for right, -1 for left

# State variables for the escape maneuver (when stuck in a corner)
escape_mode = False
escape_phase = None  # Can be "rotate" or "forward"
rotation_accumulated = 0.0
forward_timer = 0

while robot.step(TIME_STEP) != -1:
    if escape_mode:
        # During escape mode the robot ignores normal obstacle behavior.
        if escape_phase == "rotate":
            # Rotate in place: left motor backward, right motor forward.
            leftMotor.setVelocity(-0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
            
            # Integrate the gyro's yaw rate over time.
            # (Assuming gyro.getValues()[2] corresponds to the yaw angular velocity in rad/s)
            angular_velocity = gyro.getValues()  # [wx, wy, wz]
            delta_rotation = abs(angular_velocity[2]) * (TIME_STEP / 1000.0)
            rotation_accumulated += delta_rotation
            
            # Check if the robot has rotated approximately 180° (pi radians).
            if rotation_accumulated >= math.pi:
                escape_phase = "forward"
                forward_timer = 1000  # Move forward for 1000ms (adjust as needed)
                print("Completed 180° rotation. Moving forward to escape the corner.")
        elif escape_phase == "forward":
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
            forward_timer -= TIME_STEP
            if forward_timer <= 0:
                # Exit escape mode and reset state variables.
                escape_mode = False
                escape_phase = None
                rotation_accumulated = 0.0
                print("Escape maneuver completed. Resuming normal operation.")
    else:
        # Normal operation mode.
        psValues = [sensor.getValue() for sensor in ps]
        print("Proximity sensor values:", psValues)
        
        # Obstacle detection using sensor thresholds.
        right_obstacle = psValues[0] > 100.0 or psValues[1] > 100.0 or psValues[2] > 80.0
        front_obstacle = psValues[3] > 100.0 or psValues[4] > 100.0
        left_obstacle = psValues[5] > 100.0 or psValues[6] > 100.0 or psValues[7] > 80.0

        # If obstacles are detected on the front, left, and right, consider the robot stuck in a corner.
        if front_obstacle and left_obstacle and right_obstacle:
            escape_mode = True
            escape_phase = "rotate"
            rotation_accumulated = 0.0
            print("Corner detected: initiating 180° rotation escape maneuver.")
        elif front_obstacle or left_obstacle or right_obstacle:
            # Normal random turning behavior when an obstacle is detected.
            if not turning:
                turning = True
                turn_direction = random.choice([1, -1])
                print("Obstacle detected: starting random", ("right" if turn_direction == 1 else "left"), "turn.")
            leftMotor.setVelocity(turn_direction * 0.5 * MAX_SPEED)
            rightMotor.setVelocity(-turn_direction * 0.5 * MAX_SPEED)
        else:
            turning = False
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
