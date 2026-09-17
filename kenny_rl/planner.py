"""Grid A*: map walls plus sensor-observed hazards, not hidden scene geometry."""
import heapq
import numpy as np


class GridPlanner:
    def __init__(self, size, resolution, radius, progressive=False, clearance_weight=0.):
        self.size, self.resolution, self.radius = size, resolution, radius
        self.n = int(np.ceil(size / resolution))
        self.static = np.zeros((self.n, self.n), dtype=bool)
        self.expires = np.zeros((self.n, self.n), dtype=np.int32)
        self.progressive = progressive
        self.known = np.full((self.n, self.n), not progressive, dtype=bool)
        self.occupied = np.zeros((self.n, self.n), dtype=bool)
        self.exploration_target = None
        self.navigation_mode = "goal"
        self.clearance_weight = clearance_weight

        k = int(np.ceil((radius + .06) / resolution))
        self.offsets = np.array([(x, y) for x in range(-k, k + 1)
                                 for y in range(-k, k + 1)
                                 if np.hypot(x, y) * resolution <= radius + resolution], dtype=int)

    def cell(self, point):
        return tuple(np.clip(np.asarray(point) / self.resolution, 0, self.n - 1).astype(int))

    def point(self, cell):
        return (np.asarray(cell) + .5) * self.resolution

    def set_walls(self, boxes):
        x, y = np.meshgrid((np.arange(self.n) + .5) * self.resolution,
                           (np.arange(self.n) + .5) * self.resolution, indexing="ij")
        self.static[:] = False
        margin = self.radius + .04
        for b in boxes:
            self.static |= ((x >= b[0] - margin) & (x <= b[3] + margin) &
                            (y >= b[1] - margin) & (y <= b[4] + margin))

    def observe(self, points, step, ttl=50):
        """Finite memory for movable objects; longer TTL supplied for floor drops."""
        for point in points:
            cell = self.cell(point)
            # A known wall is already footprint-inflated by set_walls(). Adding
            # the same range return again would inflate it a second time and can
            # incorrectly close valid doorways. Unknown obstacle endpoints land
            # outside the static mask and still enter temporary memory.
            lo = np.maximum(np.asarray(cell)-1, 0)
            hi = np.minimum(np.asarray(cell)+2, self.n)
            if self.static[lo[0]:hi[0], lo[1]:hi[1]].any():
                continue
            cells = np.asarray(cell) + self.offsets
            cells = cells[np.all((cells >= 0) & (cells < self.n), axis=1)]
            self.expires[cells[:, 0], cells[:, 1]] = np.maximum(
                self.expires[cells[:, 0], cells[:, 1]], step + ttl)

    def observe_scan(self, origin, heading, scan):
        """Reveal only cells traversed by valid planar rays at estimated pose.

        No wall list, true pose, or interpolation across rays is used. Hit
        endpoints remain obstacles through observe(); invalid rays reveal nothing.
        This is a mapping surrogate, not scan matching or loop closure.
        """
        if not self.progressive:
            return
        endpoints = []
        for distance, angle, valid, hit in zip(scan.ranges, scan.angles, scan.valid, scan.hits):
            if not valid or not np.isfinite(distance) or distance <= 0:
                continue
            limit = max(0., distance - (self.resolution if hit else 0.))
            samples = np.arange(0., limit, self.resolution / 3)
            direction = np.array([np.cos(angle+heading), np.sin(angle+heading)])
            points = np.asarray(origin) + samples[:, None]*direction
            cells = np.floor(points/self.resolution).astype(int)
            cells = cells[np.all((cells >= 0) & (cells < self.n), axis=1)]
            self.known[cells[:, 0], cells[:, 1]] = True
            self.occupied[cells[:, 0], cells[:, 1]] = False
            if hit:
                endpoint = np.asarray(origin) + distance*direction
                if np.all(endpoint >= 0) and np.all(endpoint < self.size):
                    endpoints.append(self.cell(endpoint))
        # Hits take precedence over crossing free rays within the same scan.
        for cell in endpoints:
            self.known[cell] = True
            self.occupied[cell] = True

    def blocked(self, step):
        blocked = self.static | (self.expires > step) | ~self.known
        if self.progressive:
            for cell in np.argwhere(self.occupied):
                cells = cell + self.offsets
                cells = cells[np.all((cells >= 0) & (cells < self.n), axis=1)]
                blocked[cells[:, 0], cells[:, 1]] = True
        return blocked

    def navigation_path(self, start, goal, step=0):
        """Prefer a known route to the goal, otherwise route to a frontier.

        Frontiers are reachable free cells adjacent to unknown cells. Retain a
        chosen frontier until reached or blocked to avoid oscillating targets.
        Goal position is assumed supplied in the localization/map frame.
        """
        route = self.path(start, goal, step)
        if len(route) or not self.progressive:
            self.navigation_mode = "goal" if len(route) else "blocked"
            self.exploration_target = None
            return route
        blocked = self.blocked(step)
        penalties = self.clearance_cost(blocked)
        unknown = ~self.known
        adjacent = np.zeros_like(unknown)
        adjacent[1:] |= unknown[:-1]
        adjacent[:-1] |= unknown[1:]
        adjacent[:, 1:] |= unknown[:, :-1]
        adjacent[:, :-1] |= unknown[:, 1:]
        frontier = adjacent & ~blocked
        if self.exploration_target is not None:
            target = self.exploration_target
            if frontier[target] and np.linalg.norm(self.point(target)-start) > self.resolution*1.5:
                route = self.path(start, self.point(target), step)
                if len(route):
                    self.navigation_mode = "explore"
                    return route
        # One reachable-space search rather than a separate A* per candidate.
        a = self.cell(start)
        queue = [(0., a)]
        costs, previous = {a: 0.}, {}
        while queue:
            cost, current = heapq.heappop(queue)
            if cost > costs[current]:
                continue
            for nxt, distance in self.neighbors(current, blocked):
                candidate = cost + distance*(1+penalties[nxt])
                if candidate < costs.get(nxt, np.inf):
                    costs[nxt], previous[nxt] = candidate, current
                    heapq.heappush(queue, (candidate, nxt))
        candidates = [c for c in costs if frontier[c] and
                      np.linalg.norm(self.point(c)-start) > self.resolution*1.5]
        if not candidates:
            self.navigation_mode = "blocked"
            self.exploration_target = None
            return np.empty((0, 2))
        target = min(candidates, key=lambda c: costs[c]*self.resolution +
                     np.linalg.norm(self.point(c)-goal))
        self.exploration_target = target
        self.navigation_mode = "explore"
        cells = [target]
        while cells[-1] != a:
            cells.append(previous[cells[-1]])
        return np.array([self.point(c) for c in reversed(cells)])

    def neighbors(self, current, blocked):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = current[0]+dx, current[1]+dy
            if not (0 <= nxt[0] < self.n and 0 <= nxt[1] < self.n) or blocked[nxt]:
                continue
            if dx and dy and (blocked[current[0]+dx, current[1]] or
                              blocked[current[0], current[1]+dy]):
                continue
            yield nxt, np.hypot(dx, dy)

    def path(self, start, goal, step=0):
        blocked = self.blocked(step)
        penalties = self.clearance_cost(blocked)
        a, b = self.cell(start), self.cell(goal)
        if blocked[b]:
            return np.empty((0, 2))
        frontier = [(0., a)]
        costs, previous = {a: 0.}, {}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == b:
                cells = [b]
                while cells[-1] != a:
                    cells.append(previous[cells[-1]])
                route = np.array([self.point(c) for c in reversed(cells)])
                # Preserve the precise destination instead of stopping at a
                # nearby cell center, potentially outside the arrival radius.
                return np.vstack((route, np.asarray(goal)))
            for nxt, distance in self.neighbors(current, blocked):
                cost = costs[current] + distance*(1+penalties[nxt])
                if cost >= costs.get(nxt, np.inf):
                    continue
                costs[nxt], previous[nxt] = cost, current
                heapq.heappush(frontier, (cost + np.hypot(nxt[0] - b[0], nxt[1] - b[1]), nxt))
        return np.empty((0, 2))

    def clearance_cost(self, blocked):
        """Soft preference for room to turn/brake; narrow passages remain usable.

        Uses only the current planner map, including unknown cells. Two grid
        rings outside already inflated obstacles are penalized, not forbidden.
        """
        penalty = np.zeros_like(blocked, dtype=float)
        if self.clearance_weight == 0:
            return penalty
        expanded = blocked.copy()
        for scale in (1., .5):
            padded = np.pad(expanded, 1, constant_values=True)
            grown = np.zeros_like(blocked)
            for x in range(3):
                for y in range(3):
                    grown |= padded[x:x+self.n, y:y+self.n]
            penalty[grown & ~expanded] = scale*self.clearance_weight
            expanded = grown
        return penalty
