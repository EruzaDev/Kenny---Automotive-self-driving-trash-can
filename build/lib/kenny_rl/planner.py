"""Grid A*: map walls plus sensor-observed hazards, not hidden scene geometry."""
import heapq
import numpy as np


class GridPlanner:
    def __init__(self, size, resolution, radius):
        self.size, self.resolution, self.radius = size, resolution, radius
        self.n = int(np.ceil(size / resolution))
        self.static = np.zeros((self.n, self.n), dtype=bool)
        self.expires = np.zeros((self.n, self.n), dtype=np.int32)
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
            cells = np.asarray(self.cell(point)) + self.offsets
            cells = cells[np.all((cells >= 0) & (cells < self.n), axis=1)]
            self.expires[cells[:, 0], cells[:, 1]] = np.maximum(
                self.expires[cells[:, 0], cells[:, 1]], step + ttl)

    def path(self, start, goal, step=0):
        blocked = self.static | (self.expires > step)
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
                return np.array([self.point(c) for c in reversed(cells)])
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = current[0] + dx, current[1] + dy
                if not (0 <= nxt[0] < self.n and 0 <= nxt[1] < self.n) or blocked[nxt]:
                    continue
                if dx and dy and (blocked[current[0] + dx, current[1]] or
                                  blocked[current[0], current[1] + dy]):
                    continue
                cost = costs[current] + np.hypot(dx, dy)
                if cost >= costs.get(nxt, np.inf):
                    continue
                costs[nxt], previous[nxt] = cost, current
                heapq.heappush(frontier, (cost + np.hypot(nxt[0] - b[0], nxt[1] - b[1]), nxt))
        return np.empty((0, 2))
