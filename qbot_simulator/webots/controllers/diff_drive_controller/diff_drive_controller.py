"""Square path controller with odometry and 300ms stop between motions."""

from controller import Robot
import math

class SquarePathController:
    def __init__(self, robot):
        self.robot = robot
        self.dt = int(robot.getBasicTimeStep()) / 1000.0  # seconds

        # Robot physical parameters
        self.wheel_radius = 0.034
        self.wheel_base = 0.1934
        self.side_length = 3.0
        self.num_sides = 4

        # Motion parameters
        rpm = 100
        self.linear_speed = self.wheel_radius * rpm * 2 * math.pi / 60
        self.angular_speed = (2 * self.linear_speed) / self.wheel_base

        # Motors
        self.left_motor = robot.getDevice("left_wheel_joint")
        self.right_motor = robot.getDevice("right_wheel_joint")
        self.left_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)
        self.right_motor.setPosition(float("inf"))
        self.right_motor.setVelocity(0)

        # Sensors
        self.left_sensor = self.left_motor.getPositionSensor()
        self.right_sensor = self.right_motor.getPositionSensor()
        self.left_sensor.enable(int(robot.getBasicTimeStep()))
        self.right_sensor.enable(int(robot.getBasicTimeStep()))

        # Odometry
        self.x = self.y = self.theta = 0.0
        self.prev_left = self.prev_right = 0.0
        self.start_pos = (0.0, 0.0)
        self.start_theta = 0.0

        # State
        self.state = "forward"
        self.waiting = False
        self.side_count = 0
        self.time = self.wait_time = 0.0

    def update_odometry(self):
        left_pos = self.left_sensor.getValue()
        right_pos = self.right_sensor.getValue()
        d_left = left_pos - self.prev_left
        d_right = right_pos - self.prev_right
        self.prev_left = left_pos
        self.prev_right = right_pos

        d_center = 0.5 * self.wheel_radius * (d_right + d_left)
        d_theta = (self.wheel_radius / self.wheel_base) * (d_right - d_left)

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta = (self.theta + d_theta + math.pi) % (2 * math.pi) - math.pi

    def set_velocity(self, left, right):
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def angle_diff(self, a, b):
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def handle_wait(self):
        if self.time - self.wait_time >= 0.3:
            self.waiting = False
        else:
            self.set_velocity(0, 0)

    def step_logic(self):
        if self.waiting:
            self.handle_wait()
            return

        if self.state == "forward":
            dist = math.hypot(self.x - self.start_pos[0], self.y - self.start_pos[1])
            if dist < self.side_length:
                speed = self.linear_speed / self.wheel_radius
                self.set_velocity(speed, speed)
            else:
                self.set_velocity(0, 0)
                self.state = "turn"
                self.start_theta = self.theta
                self.waiting = True
                self.wait_time = self.time

        elif self.state == "turn":
            angle_turned = abs(self.angle_diff(self.theta, self.start_theta))
            if angle_turned < math.pi / 2:
                w = self.angular_speed * (self.wheel_base / (2 * self.wheel_radius))
                self.set_velocity(-w, w)
            else:
                self.set_velocity(0, 0)
                self.side_count = (self.side_count + 1) % self.num_sides
                self.state = "forward"
                self.start_pos = (self.x, self.y)
                self.waiting = True
                self.wait_time = self.time

    def run(self):
        while self.robot.step(int(self.dt * 1000)) != -1:
            self.time += self.dt
            self.update_odometry()
            self.step_logic()
            print(f"[{self.state.upper()}] x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.1f}°")

if __name__ == "__main__":
    SquarePathController(Robot()).run()
