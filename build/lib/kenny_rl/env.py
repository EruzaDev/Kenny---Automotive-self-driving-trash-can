"""Gymnasium differential-drive navigation with partial observations and map routing."""
from collections import deque
from dataclasses import asdict
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from .config import RobotConfig, EnvConfig
from .geometry import circle_boxes, circle_rects, wrap_angle
from .planner import GridPlanner
from .world import generate_world
from . import sensors


FEATURES = [("lidar_range", 72), ("lidar_valid", 72),
            ("depth_range", 36), ("depth_valid", 36),
            ("floor_drop_range", 12), ("floor_valid", 12),
            ("downward_hazard", 3), ("downward_valid", 3),
            ("route_points_xy", 6), ("route_valid", 3),
            ("goal_relative_xy", 2), ("measured_velocity", 2),
            ("previous_requested_action", 2), ("previous_executed_action", 2),
            ("localization_health", 3), ("sensor_age_and_guard", 3)]
FRAME_SIZE = sum(n for _, n in FEATURES)
SCHEMA_VERSION = "kenny-geometric-v1"


class KennyEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 10}

    def __init__(self, robot=None, config=None, render_mode=None):
        super().__init__()
        self.robot, self.config = robot or RobotConfig(), config or EnvConfig()
        if render_mode not in (None, *self.metadata["render_modes"]):
            raise ValueError("Unsupported render mode")
        self.render_mode = render_mode
        self.action_space = spaces.Box(-1., 1., shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(-1., 1., shape=(FRAME_SIZE*self.config.history,), dtype=np.float32)
        self.history = deque(maxlen=self.config.history)
        self.figure = None
        self.finished = True

    def contract(self):
        return {"version": SCHEMA_VERSION, "features": FEATURES,
                "frame_size": FRAME_SIZE, "history": self.config.history,
                "robot": asdict(self.robot), "normalization": "fixed bounds in env._frame",
                "actions": "v=(a0+1)/2*max_speed; omega=a1*max_turn_rate"}

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            # Independent PRNG streams per split, even if the numeric seed is reused.
            split_id = {"train": 0, "validation": 1, "test": 2, "stress": 3}[self.config.split]
            seed = int(np.random.SeedSequence([seed, split_id]).generate_state(1)[0])
        super().reset(seed=seed)
        options = options or {}
        if set(options) - {"start", "goal", "heading"}:
            raise ValueError("Supported reset options: start, goal, heading")
        self.world = generate_world(self.np_random, self.config, self.robot)
        for name in ("start", "goal"):
            if name not in options:
                continue
            point = np.asarray(options[name], dtype=float)
            if (point.shape != (2,) or not np.isfinite(point).all() or
                    np.any(point <= self.robot.radius) or
                    np.any(point >= self.world.size-self.robot.radius) or
                    circle_boxes(point, self.robot.radius+.05, self.world.all_boxes(), self.robot.height) or
                    circle_rects(point, self.robot.radius+.05, self.world.cliffs)):
                raise ValueError(f"{name} must be a finite, collision-free x/y point inside the room")
            setattr(self.world, name, point.copy())
        heading = options.get("heading", self.np_random.uniform(-np.pi, np.pi))
        if not np.isscalar(heading) or not np.isfinite(heading):
            raise ValueError("heading must be a finite angle in radians")
        self.pose = np.array([*self.world.start, wrap_angle(heading)])
        self.estimate = self.pose.copy()  # initialized by a surveyed start marker/pose
        self.velocity = np.zeros(2)
        self.measured_velocity = np.zeros(2)
        self.requested = np.array([-1., 0.])
        self.executed = np.array([-1., 0.])
        self.steps = self.marker_age = self.interventions = 0
        self.path_length = 0.
        self.clutter_changes = 0
        self.uncertainty = .02
        self.intervened = False
        randomize = self.config.domain_randomization
        self.gain = self.np_random.uniform(.9, 1.1, 2) if randomize else np.ones(2)
        self.slip = self.np_random.uniform(.94, 1.02, 2) if randomize else np.ones(2)
        self.accel_scale = self.np_random.uniform(.7, 1.1) if randomize else 1.
        self.delay = int(self.np_random.integers(0, 3)) if randomize else 0
        self.commands = deque([np.array([-1., 0.]) for _ in range(self.delay)])
        self.dropout = self.config.dropout * (2 if self.config.split == "stress" else 1)
        self.planner = GridPlanner(self.world.size, self.config.grid_resolution, self.robot.radius)
        self.planner.set_walls(self.world.boxes[np.array(self.world.kinds) == "wall"])
        self.route = np.empty((0, 2))
        self.last_lidar_step = -100
        self._sense(force=True)
        self._replan()
        self.previous_remaining = self._remaining()
        self.history.clear()
        frame = self._frame()
        # Repeat initial frame rather than introduce artificial all-zero distances.
        for _ in range(self.config.history):
            self.history.append(frame.copy())
        self.finished = False
        return self._observation(), self._info("running")

    def _sense(self, force=False):
        r, c = self.robot, self.config
        period = max(1, round(1/(r.lidar_hz*c.dt)))
        if force or self.steps-self.last_lidar_step >= period:
            self.lidar = sensors.lidar(self.world, self.pose, r, self.np_random, c.sensor_noise, self.dropout)
            self.last_lidar_step = self.steps
            self.lidar_estimate = self.estimate.copy()
        self.depth = sensors.depth(self.world, self.pose, r, self.np_random, c.sensor_noise, self.dropout)
        self.floor = sensors.floor_scan(self.world, self.pose, r, self.np_random, self.dropout)
        self.down_hazard, self.down_valid = sensors.downward(self.world, self.pose, r, self.np_random, self.dropout)
        # Project using estimated acquisition pose, not simulator truth.
        for scan, acquisition in ((self.lidar, self.lidar_estimate), (self.depth, self.estimate)):
            if scan is self.lidar and not (force or self.steps == self.last_lidar_step):
                continue
            angle = scan.angles[scan.hits]+acquisition[2]
            points = acquisition[:2] + scan.ranges[scan.hits, None]*np.column_stack((np.cos(angle), np.sin(angle)))
            self.planner.observe(points, self.steps)
        angle = self.floor.angles[self.floor.hits] + self.estimate[2]
        points = self.estimate[:2] + self.floor.ranges[self.floor.hits, None]*np.column_stack((np.cos(angle), np.sin(angle)))
        self.planner.observe(points, self.steps, ttl=10000)

    def _replan(self):
        self.route = self.planner.path(self.estimate[:2], self.world.goal, self.steps)

    def _remaining(self):
        if not len(self.route):
            return None
        index = np.argmin(np.linalg.norm(self.route-self.estimate[:2], axis=1))
        return float(np.linalg.norm(self.route[index]-self.estimate[:2]) +
                     np.linalg.norm(np.diff(self.route[index:], axis=0), axis=1).sum())

    def _local(self, points):
        d = np.asarray(points)-self.estimate[:2]
        theta = self.estimate[2]
        return d @ np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

    def _lookahead(self):
        if not len(self.route):
            return np.zeros((3, 2)), np.zeros(3)
        index = np.argmin(np.linalg.norm(self.route-self.estimate[:2], axis=1))
        indices = np.minimum(index + np.array([2, 4, 8]), len(self.route)-1)
        return self._local(self.route[indices]), np.ones(3)

    def _frame(self):
        r = self.robot
        points, valid = self._lookahead()
        values = [self.lidar.ranges/r.lidar_range, self.lidar.valid,
                  self.depth.ranges/r.camera_range, self.depth.valid,
                  self.floor.ranges/r.camera_range, self.floor.valid,
                  self.down_hazard, self.down_valid, points.ravel()/3., valid,
                  self._local(self.world.goal[None])[0]/self.world.size,
                  self.measured_velocity/np.array([r.max_speed, r.max_turn_rate]),
                  self.requested, self.executed,
                  [self.uncertainty/.5, min(self.marker_age*self.config.dt/30., 1), float(self.uncertainty < .35)],
                  [(self.steps-self.last_lidar_step)*self.config.dt, 0., float(self.intervened)]]
        return np.clip(np.concatenate([np.asarray(v).ravel() for v in values]), -1, 1).astype(np.float32)

    def _observation(self):
        return np.concatenate(self.history).astype(np.float32)

    def _guard(self, target):
        """Stop using sensor evidence only, never a simulator collision query."""
        if not self.config.shield:
            return target, False
        r, c = self.robot, self.config
        speed = max(abs(self.velocity[0]), abs(target[0]))
        braking = speed**2/(2*r.acceleration*min(self.accel_scale, 1.))
        margin = r.radius + .05 + braking + speed*(c.dt + 1/r.lidar_hz)
        moving = target[0] > 0 or abs(target[1]) > 0
        hazards = np.any(self.down_hazard | ~self.down_valid) or self.uncertainty > .35
        # Sensor unknowns in the direction of travel cause a conservative stop.
        front = np.abs(self.lidar.angles) < np.deg2rad(40)
        hazards |= not np.all(self.lidar.valid[front])
        hazards |= np.count_nonzero(self.depth.valid) < len(self.depth.valid)*.8
        hazards |= np.count_nonzero(self.floor.valid) < len(self.floor.valid)*.5
        for scan in (self.lidar, self.depth):
            lateral = np.abs(scan.ranges*np.sin(scan.angles))
            ahead = scan.ranges*np.cos(scan.angles)
            collision = scan.hits & (lateral < r.radius+.05) & (ahead > 0) & (ahead < margin)
            hazards |= bool(np.any(collision))
        hazards |= bool(np.any(self.floor.hits & (self.floor.ranges < margin)))
        if moving and hazards:
            return np.zeros(2), True
        return target, False

    def step(self, action):
        if self.finished:
            raise RuntimeError("Call reset() before stepping a finished episode")
        action = np.asarray(action, dtype=float)
        if action.shape != (2,) or not np.isfinite(action).all():
            raise ValueError("Expected two finite action values")
        r, c = self.robot, self.config
        previous_action = self.requested.copy()
        self.requested = np.clip(action, -1, 1)
        self.commands.append(self.requested.copy())
        delayed = self.commands.popleft()
        target = np.array([(delayed[0]+1)/2*r.max_speed, delayed[1]*r.max_turn_rate])
        if not len(self.route):
            target[:] = 0  # no blind recovery motion
        previously_intervened = self.intervened
        target, self.intervened = self._guard(target)
        self.interventions += int(self.intervened)
        self.executed = np.array([2*target[0]/r.max_speed-1, target[1]/r.max_turn_rate])
        max_delta = np.array([r.acceleration*self.accel_scale, r.angular_acceleration])*c.dt
        # Wheel target saturation respects the actual motor/wheel maximum.
        wheels = np.array([target[0]-target[1]*r.track_width/2, target[0]+target[1]*r.track_width/2])
        wheels *= self.gain
        wheels /= max(1., np.max(np.abs(wheels))/r.wheel_surface_speed)
        actual_target = np.array([wheels.mean(), (wheels[1]-wheels[0])/r.track_width])
        self.velocity += np.clip(actual_target-self.velocity, -max_delta, max_delta)
        self.measured_velocity = self.velocity.copy()
        start = self.pose[:2].copy()
        event = "running"
        # Substeps prevent jumping through thin boxes/edges; people share substeps.
        substeps = max(5, int(np.ceil(abs(self.velocity[0])*c.dt/.02)))
        for _ in range(substeps):
            dt = c.dt/substeps
            self.world.move_people(dt, self.np_random)
            self.pose[2] = wrap_angle(self.pose[2]+self.velocity[1]*self.slip[1]*dt)
            self.pose[:2] += self.velocity[0]*self.slip[0]*dt*np.array([np.cos(self.pose[2]), np.sin(self.pose[2])])
            if circle_boxes(self.pose[:2], r.radius, self.world.all_boxes(), r.height):
                event = "collision"; break
            if circle_rects(self.pose[:2], r.radius, self.world.cliffs):
                event = "cliff"; break
        self.path_length += float(np.linalg.norm(self.pose[:2]-start))
        self.steps += 1
        if self.config.stage in ("dynamic", "full") and self.steps % 200 == 0:
            self.clutter_changes += int(self.world.change_clutter(self.np_random, self.pose, r))
        self.marker_age += 1
        self.estimate[2] = wrap_angle(self.estimate[2]+self.measured_velocity[1]*c.dt)
        self.estimate[:2] += self.measured_velocity[0]*c.dt*np.array([np.cos(self.estimate[2]), np.sin(self.estimate[2])])
        self.uncertainty += .0005 + .003*abs(self.velocity[0])*c.dt
        predicted_remaining = self._remaining()
        corrected = sensors.marker_visible(self.world, self.pose, r, self.np_random, c.marker_dropout)
        if corrected:
            # Surrogate of a calibrated marker pose solution; no perfect pose observation.
            self.estimate = self.pose + self.np_random.normal(0, [.015, .015, .02])
            self.estimate[2] = wrap_angle(self.estimate[2])
            self.uncertainty, self.marker_age = .02, 0
        self._sense()
        replanned = self.steps % 10 == 0
        if replanned:
            self._replan()
        remaining = self._remaining()
        progress = 0.
        if predicted_remaining is not None and self.previous_remaining is not None:
            # Motion progress before landmark correction/replanning prevents reward jumps.
            progress = float(np.clip(self.previous_remaining-predicted_remaining, -.1, .1))
        self.previous_remaining = remaining
        reward = 5*progress-.01-.05*float(np.square(self.requested-previous_action).sum())
        reward -= .02*int(self.intervened) + .1*int(self.intervened and not previously_intervened)
        if event in ("collision", "cliff"):
            reward -= 50
        # Simulator truth checks success; it is never exposed as policy pose input.
        elif (np.linalg.norm(self.pose[:2]-self.world.goal) < .25 and
              np.linalg.norm(self.estimate[:2]-self.world.goal) < .25 and
              abs(self.velocity[0]) < .05 and abs(self.velocity[1]) < .1):
            event, reward = "success", reward+20
        terminated = event != "running"
        truncated = not terminated and self.steps >= c.max_steps
        if truncated:
            event = "timeout"
        self.finished = terminated or truncated
        self.history.append(self._frame())
        return self._observation(), float(reward), terminated, truncated, self._info(event)

    def _info(self, event):
        return {"event": event, "is_success": event == "success", "steps": self.steps,
                "interventions": self.interventions, "path_length": self.path_length,
                "clutter_changes": self.clutter_changes,
                "localization_error": float(np.linalg.norm(self.pose[:2]-self.estimate[:2])),
                "route_available": bool(len(self.route)), "stage": self.config.stage,
                "split": self.config.split}

    def render(self):
        from .render import render
        return render(self)

    def close(self):
        if self.figure is not None:
            import matplotlib.pyplot as plt
            plt.close(self.figure)
            self.figure = None
