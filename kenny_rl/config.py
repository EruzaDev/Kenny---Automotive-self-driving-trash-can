"""Physical assumptions are explicit and serialized with every checkpoint."""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import math


@dataclass(frozen=True)
class RobotConfig:
    # User approximate body dimensions; track assumes wheel-center distance.
    width: float = 0.23
    length: float = 0.23
    height: float = 0.485
    track_width: float = 0.235
    wheel_radius: float = 0.04
    motor_rpm: float = 333.0
    max_speed: float = 0.30
    max_turn_rate: float = 0.60
    acceleration: float = 0.50
    angular_acceleration: float = 1.50
    lidar_height: float = 0.22
    lidar_range: float = 6.0
    lidar_hz: float = 5.0
    camera_height: float = 0.40
    camera_pitch_deg: float = -15.0
    camera_hfov_deg: float = 60.0
    camera_vfov_deg: float = 50.0
    camera_min_range: float = 0.40
    camera_range: float = 4.0
    marker_range: float = 2.5
    # Assumes added downward sensors; not present on the user's original parts list.
    cliff_lookahead: float = 0.12

    @property
    def radius(self):
        return math.hypot(self.width, self.length) / 2

    @property
    def wheel_surface_speed(self):
        return self.motor_rpm * 2 * math.pi * self.wheel_radius / 60

    def __post_init__(self):
        for name in ("width", "length", "height", "track_width", "wheel_radius",
                     "motor_rpm", "max_speed", "max_turn_rate", "acceleration",
                     "angular_acceleration", "lidar_range", "lidar_hz", "camera_range",
                     "marker_range", "cliff_lookahead"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not 0 < self.camera_hfov_deg < 180 or not 0 < self.camera_vfov_deg < 180:
            raise ValueError("Camera field of view must be between 0 and 180 degrees")
        if not 0 < self.camera_height <= self.height:
            raise ValueError("Camera height must be within robot height")
        if not 0 < self.camera_min_range < self.camera_range:
            raise ValueError("Invalid camera range")
        if self.max_speed > self.wheel_surface_speed:
            raise ValueError("max_speed exceeds nominal wheel surface speed")


@dataclass(frozen=True)
class EnvConfig:
    stage: str = "full"
    split: str = "train"
    world_size: float = 12.0
    dt: float = 0.1
    max_steps: int = 1200
    grid_resolution: float = 0.25
    domain_randomization: bool = True
    shield: bool = True
    sensor_noise: float = 0.015
    dropout: float = 0.015
    marker_dropout: float = 0.15
    history: int = 4
    # Provisional ranges; replace with measurements from empty/full-bin trials.
    motor_gain_range: tuple = (0.9, 1.1)
    slip_range: tuple = (0.94, 1.02)
    acceleration_scale_range: tuple = (0.7, 1.1)
    command_delay_max_steps: int = 2
    sensor_outage_probability: float = 0.0
    sensor_outage_steps: tuple = (2, 5)

    def __post_init__(self):
        if self.stage not in ("empty", "static", "mixed", "dynamic", "cliffs", "full"):
            raise ValueError("Unknown curriculum stage")
        if self.split not in ("train", "validation", "test", "stress"):
            raise ValueError("Unknown split")
        if self.world_size < 5 or self.dt <= 0 or self.max_steps < 1:
            raise ValueError("Invalid world size, dt or episode length")
        if self.sensor_noise < 0:
            raise ValueError("sensor_noise must be nonnegative")
        if self.grid_resolution <= 0 or self.history < 1:
            raise ValueError("Invalid grid resolution or history")
        if not all(0 <= x < 1 for x in (self.dropout, self.marker_dropout)):
            raise ValueError("Dropout must be in [0, 1)")
        for name in ("motor_gain_range", "slip_range", "acceleration_scale_range"):
            bounds = getattr(self, name)
            if len(bounds) != 2 or not all(math.isfinite(x) for x in bounds) or not 0 < bounds[0] <= bounds[1]:
                raise ValueError(f"Invalid {name}")
        if not isinstance(self.command_delay_max_steps, int) or self.command_delay_max_steps < 0:
            raise ValueError("command_delay_max_steps must be a nonnegative integer")
        if not 0 <= self.sensor_outage_probability < 1:
            raise ValueError("sensor_outage_probability must be in [0, 1)")
        if (len(self.sensor_outage_steps) != 2 or
                not all(isinstance(x, int) for x in self.sensor_outage_steps) or
                not 1 <= self.sensor_outage_steps[0] <= self.sensor_outage_steps[1]):
            raise ValueError("Invalid sensor_outage_steps")


def load_config(path):
    data = json.loads(Path(path).read_text())
    for key in data:
        if key not in ("robot", "environment", "training"):
            raise ValueError(f"Unknown config section: {key}")
    return (RobotConfig(**data.get("robot", {})),
            EnvConfig(**data.get("environment", {})), data.get("training", {}))


def serialize(robot, environment, training=None):
    return {"robot": asdict(robot), "environment": asdict(environment),
            "training": training or {}}
