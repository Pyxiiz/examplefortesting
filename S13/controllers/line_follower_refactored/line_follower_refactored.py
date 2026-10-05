# NOTE: This is a constructed calibration fixture, not a real student
# submission. It implements the same control logic and thresholds as
# S9/controllers/line_following/line_following.py, but rewritten as a
# class with a different decision structure (precomputed booleans
# instead of inline threshold checks), different odometry formulation,
# and consolidated logging. No token-level overlap with S9 is expected -
# this specifically tests whether semantic (LLM-based) comparison can
# catch logic-equivalent paraphrase that structural fingerprinting
# cannot. See LABELS.json at the repo root for ground truth.

from controller import Robot
import numpy as np


class LineFollowerRobot:
    """E-puck line follower with dead-reckoning position tracking."""

    WHEEL_RADIUS_M = 0.0201
    WHEEL_BASE_M = (0.026 + 0.005 / 2) * 2
    CRUISE_SPEED = 3.14 / 2
    ON_LINE_THRESHOLD = 500

    def __init__(self):
        self.robot = Robot()
        self.tick_ms = int(self.robot.getBasicTimeStep())
        self.tick_s = self.tick_ms / 1000.0

        self.left_wheel = self.robot.getDevice('left wheel motor')
        self.right_wheel = self.robot.getDevice('right wheel motor')
        self.left_wheel.setPosition(float('Inf'))
        self.right_wheel.setPosition(float('Inf'))

        self.sensors = {
            'left': self.robot.getDevice('gs0'),
            'center': self.robot.getDevice('gs1'),
            'right': self.robot.getDevice('gs2'),
        }
        for sensor in self.sensors.values():
            sensor.enable(self.tick_ms)

        self.distance_traveled_m = 0.0
        self.heading_accumulated_rad = 0.0

        self.world_x = 0.0
        self.world_y = 0.028
        self.world_heading_rad = 1.5708

        self.is_recovering = False

    def _sensor_on_line(self, name):
        return self.sensors[name].getValue() < self.ON_LINE_THRESHOLD

    def _choose_wheel_speeds(self):
        on_left, on_center, on_right = (
            self._sensor_on_line('left'),
            self._sensor_on_line('center'),
            self._sensor_on_line('right'),
        )

        if not self.is_recovering and on_left and on_center and on_right:
            # All three on the line at once and we weren't already
            # recovering - treat as the start/finish marker, stop.
            return 0.0, 0.0

        if on_center and not on_left and not on_right:
            self.is_recovering = False
            return self.CRUISE_SPEED, self.CRUISE_SPEED

        if on_right:
            self.is_recovering = True
            return 0.25 * self.CRUISE_SPEED, -0.1 * self.CRUISE_SPEED

        if on_left:
            self.is_recovering = True
            return -0.1 * self.CRUISE_SPEED, 0.25 * self.CRUISE_SPEED

        # Fallback: hold previous behavior rather than stall silently.
        return 0.0, 0.0

    def _integrate_odometry(self, left_speed, right_speed):
        left_linear = self.WHEEL_RADIUS_M * left_speed
        right_linear = self.WHEEL_RADIUS_M * right_speed

        step_distance = (left_linear + right_linear) / 2 * self.tick_s
        step_rotation = (right_linear - left_linear) / self.WHEEL_BASE_M * self.tick_s

        self.distance_traveled_m += step_distance
        self.heading_accumulated_rad += step_rotation

        self.world_heading_rad += step_rotation
        self.world_x += np.cos(self.world_heading_rad) * step_distance
        self.world_y += np.sin(self.world_heading_rad) * step_distance

    def _log_tick(self, left_speed, right_speed):
        distance_from_start = np.sqrt(self.world_x ** 2 + self.world_y ** 2)
        print(
            f"[tick] wheels=({left_speed:.3f}, {right_speed:.3f}) "
            f"recovering={self.is_recovering} "
            f"pos=({self.world_x:.4f}, {self.world_y:.4f}) "
            f"heading={self.world_heading_rad:.4f} "
            f"dist_total={self.distance_traveled_m:.4f} "
            f"dist_from_start={distance_from_start:.4f}"
        )

    def run(self):
        while self.robot.step(self.tick_ms) != -1:
            left_speed, right_speed = self._choose_wheel_speeds()
            self.left_wheel.setVelocity(left_speed)
            self.right_wheel.setVelocity(right_speed)
            self._integrate_odometry(left_speed, right_speed)
            self._log_tick(left_speed, right_speed)


if __name__ == '__main__':
    LineFollowerRobot().run()
