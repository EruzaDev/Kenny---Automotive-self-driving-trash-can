"""Procedural blocks, maze walls, low bags, elevated seats, people and floor gaps."""
from dataclasses import dataclass
import numpy as np
from .geometry import circle_boxes, circle_rects
from .planner import GridPlanner


@dataclass
class World:
    size: float
    boxes: np.ndarray
    kinds: list
    cliffs: np.ndarray
    markers: np.ndarray
    people: np.ndarray  # x, y, vx, vy, radius
    start: np.ndarray
    goal: np.ndarray
    width: float | None = None
    height: float | None = None

    def __post_init__(self):
        self.width = self.size if self.width is None else self.width
        self.height = self.size if self.height is None else self.height

    def people_boxes(self):
        if not len(self.people):
            return np.empty((0, 6))
        p = self.people
        return np.column_stack((p[:, 0]-p[:, 4], p[:, 1]-p[:, 4], np.zeros(len(p)),
                                p[:, 0]+p[:, 4], p[:, 1]+p[:, 4], np.full(len(p), 1.75)))

    def all_boxes(self):
        if not len(self.people):
            return self.boxes
        return np.concatenate((self.boxes, self.people_boxes()))

    def change_clutter(self, rng, robot_pose, robot):
        """Move/place a bag between steps, outside the robot's immediate footprint."""
        for _ in range(30):
            point = rng.uniform(.7, np.asarray([self.width, self.height])-1.)
            if (np.linalg.norm(point-robot_pose[:2]) < 1. or
                    np.linalg.norm(point-self.goal) < .6 or
                    circle_boxes(point, .35, self.all_boxes(), robot.height) or
                    circle_rects(point, .35, self.cliffs)):
                continue
            bag = [point[0], point[1], 0., point[0]+.3, point[1]+.3, .12]
            indices = [i for i, kind in enumerate(self.kinds) if kind == "bag"]
            if indices:
                self.boxes[int(rng.choice(indices))] = bag
            else:
                self.boxes = np.vstack((self.boxes, bag))
                self.kinds.append("bag")
            return True
        return False

    def move_people(self, dt, rng, robot_position=None, robot_radius=0., cooperative=True):
        """Advance pedestrians with a predictable, single-file yielding convention.

        Cooperative pedestrians keep moving in their intended direction.  When
        Kenny is on a collision course they yield to their own right with a
        small outward component, rather than reversing directly toward the
        robot.  Nearby pedestrians travelling in the same direction also
        queue behind one another instead of occupying a side-by-side barrier.
        """
        for index, p in enumerate(self.people):
            if rng.random() < dt * .25:
                theta = rng.uniform(-np.pi, np.pi)
                speed = rng.uniform(.15, .8)
                p[2:4] = speed * np.array([np.cos(theta), np.sin(theta)])
            velocity = p[2:4].copy()
            speed = np.linalg.norm(velocity)
            travel = velocity / speed if speed > 1e-9 else np.array([1., 0.])
            intended_travel = travel.copy()
            if cooperative and robot_position is not None:
                robot_delta = np.asarray(robot_position)-p[:2]
                # Predict the next closest approach, not merely current range.
                approaching = np.dot(robot_delta, travel) > -.05
                clearance = p[4]+robot_radius+.32
                if approaching and np.linalg.norm(robot_delta) < clearance+.55:
                    # Standard pedestrian convention: yield to the pedestrian's
                    # right. This gives Kenny a stable left-side passing cue
                    # when a pedestrian is travelling to Kenny's right.
                    right = np.array([travel[1], -travel[0]])
                    outward = -robot_delta / max(np.linalg.norm(robot_delta), 1e-9)
                    direction = .80*right + .35*outward
                    direction /= max(np.linalg.norm(direction), 1e-9)
                    velocity = max(speed, .15)*direction
                    travel = direction

                # People with similar headings queue in single file.  The
                # later pedestrian yields behind the earlier one if they would
                # otherwise create a side-by-side obstacle across Kenny's path.
                for other in self.people[:index]:
                    other_speed = np.linalg.norm(other[2:4])
                    # The lead may already be yielding, so accept broadly
                    # aligned headings rather than requiring near-parallel
                    # instantaneous velocities.
                    if other_speed < 1e-9 or np.dot(travel, other[2:4]/other_speed) < .2:
                        continue
                    delta = other[:2]-p[:2]
                    lateral = abs(np.cross(travel, delta))
                    longitudinal = np.dot(delta, travel)
                    if lateral < p[4]+other[4]+.18 and abs(longitudinal) < .9:
                        velocity = -intended_travel*min(max(speed, .15), other_speed)
                        break
            candidate = p[:2] + dt * velocity
            if (circle_boxes(candidate, p[4], self.boxes, 1.75) or
                    circle_rects(candidate, p[4], self.cliffs)):
                p[2:4] *= -1
                continue
            if cooperative and robot_position is not None:
                # The steering phase already keeps a generous social margin.
                # Only pause here for an actual near-contact, otherwise the
                # pedestrian would freeze instead of completing its side-step.
                clearance = p[4]+robot_radius+.02
                if np.linalg.norm(candidate-robot_position) < clearance:
                    # A blocked side-step pauses rather than walking into Kenny.
                    p[2:4] = np.zeros(2)
                    continue
            p[2:4] = velocity
            p[:2] = candidate


def generate_world(rng, config, robot):
    """Rejection sample connected missions; truth is used only for generation/labels."""
    for attempt in range(100):
        width = config.world_width or config.world_size
        height = config.world_height or config.world_size
        if config.domain_randomization:
            scale = rng.uniform(.85, 1.15) if config.split != "stress" else rng.uniform(1.1, 1.4)
            width *= scale
            height *= scale
        size = max(width, height)  # legacy scalar used by older callers
        boxes, kinds, cliffs = [], [], []
        def box(x, y, w, d, low=0., high=1.8, kind="wall"):
            boxes.append([x, y, low, x+w, y+d, high]); kinds.append(kind)
        box(0, 0, width, .15); box(0, height-.15, width, .15)
        box(0, 0, .15, height); box(width-.15, 0, .15, height)
        if config.stage != "empty":
            # Alternating openings create detours instead of a direct goal attractor.
            if rng.random() < .65:
                for i, x in enumerate(np.linspace(width*.3, width*.7, 2)):
                    gap_y = rng.uniform(height*.25, height*.75)
                    gap = max(1.3, robot.radius * 2 + .7)
                    box(x, .15, .18, max(.1, gap_y-gap/2-.15))
                    box(x, gap_y+gap/2, .18, max(.1, height-.15-gap_y-gap/2))
            if config.split == "stress":
                # Held-out U-shaped cul-de-sac topology, beyond mere seed changes.
                x, y = width*.42, height*.42
                box(x, y, width*.2, .18)
                box(x, y, .18, height*.18)
                box(x+width*.2, y, .18, height*.18)
            for _ in range(int(rng.integers(3, 8))):
                x, y = rng.uniform(1., np.asarray([width, height])-2.)
                box(x, y, rng.uniform(.3, 1.), rng.uniform(.3, 1.))
        if config.stage in ("mixed", "dynamic", "cliffs", "full"):
            for _ in range(int(rng.integers(3, 7))):
                x, y = rng.uniform(1., np.asarray([width, height])-1.8)
                if rng.random() < .5:
                    # Seat elevated above the scan; legs are narrower than LiDAR bins.
                    box(x, y, .55, .55, .48, .56, "chair_seat")
                    for dx, dy in ((0, 0), (.50, 0), (0, .50), (.50, .50)):
                        box(x+dx, y+dy, .05, .05, 0., .48, "chair_leg")
                    box(x, y+.5, .55, .05, .56, 1., "chair_back")
                else:
                    box(x, y, rng.uniform(.2, .6), rng.uniform(.2, .5),
                        0., rng.uniform(.07, .18), "bag")
            for _ in range(2):
                x, y = rng.uniform(1., np.asarray([width, height])-2.)
                box(x, y, .8, .4, robot.height*.65, robot.height*.9, "overhang")
        if config.stage in ("cliffs", "full"):
            for _ in range(int(rng.integers(1, 4))):
                x, y = rng.uniform(1., np.asarray([width, height])-2.)
                cliffs.append([x, y, x+rng.uniform(.4, 1.2), y+rng.uniform(.5, 1.5)])
        boxes = np.asarray(boxes, dtype=float).reshape(-1, 6)
        cliffs = np.asarray(cliffs, dtype=float).reshape(-1, 4)
        def free(p, radius=robot.radius+.15):
            return not circle_boxes(p, radius, boxes, robot.height) and not circle_rects(p, radius, cliffs)
        candidates = [p for p in rng.uniform(.7, np.asarray([width, height])-.7, (150, 2)) if free(p)]
        if len(candidates) < 3:
            continue
        start = candidates[0]
        goal_candidates = [p for p in candidates[1:]
                           if (1.0 if config.stage == "empty" else 2.) < np.linalg.norm(p-start)
                           < (3.5 if config.stage == "empty" else size*1.2)]
        if not goal_candidates:
            continue
        goal = goal_candidates[0]
        validator = GridPlanner([width, height], config.grid_resolution, robot.radius)
        full = boxes[(boxes[:, 2] < robot.height) & (boxes[:, 5] > 0)]
        if len(cliffs):
            full = np.concatenate((full, np.column_stack((cliffs[:, :2], np.zeros(len(cliffs)),
                                                         cliffs[:, 2:], np.ones(len(cliffs))))))
        validator.set_walls(full)
        if not len(validator.path(start, goal)):
            continue
        markers = [p for p in rng.uniform(.5, np.asarray([width, height])-.5, (60, 2)) if free(p, .1)]
        markers.extend([start.copy(), goal.copy()])
        people = []
        if config.stage in ("dynamic", "full"):
            for p in candidates[2:2+int(rng.integers(2, 5))]:
                if np.linalg.norm(p-start) > 1.:
                    theta = rng.uniform(-np.pi, np.pi)
                    speed = rng.uniform(.2, .8)
                    people.append([*p, speed*np.cos(theta), speed*np.sin(theta), .23])
        return World(size, boxes, kinds, cliffs, np.asarray(markers),
                     np.asarray(people).reshape(-1, 5), start, goal, width, height)
    raise RuntimeError("Could not sample a reachable world; check dimensions/density")
