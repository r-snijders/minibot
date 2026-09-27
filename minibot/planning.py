"""Small, ROS-independent occupancy-grid planner for a slow indoor rover."""
import heapq
import math
from collections import deque


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Grid:
    def __init__(self, width, height, resolution, origin, data, radius=0.16):
        self.w, self.h, self.res, self.origin = width, height, resolution, origin
        self.data = list(data)
        self.free = {i for i, v in enumerate(data) if 0 <= v < 35}
        # Unknown cells cannot be driven through. Inflate obstacles and unknown
        # cells by the robot's circumscribed radius (plus a small margin).
        cells = math.ceil(radius / resolution)
        offsets = [(dx, dy) for dx in range(-cells, cells + 1)
                   for dy in range(-cells, cells + 1)
                   if math.hypot(dx, dy) * resolution <= radius + resolution / 2]
        blocked = set()
        for i, v in enumerate(data):
            if v < 0 or v >= 35:
                x, y = i % width, i // width
                for dx, dy in offsets:
                    xx, yy = x + dx, y + dy
                    if 0 <= xx < width and 0 <= yy < height:
                        blocked.add(yy * width + xx)
        self.free -= blocked
        # Treat the edge of the available map as unknown too.
        self.free = {i for i in self.free
                     if min(i % width, width-1-i % width,
                            i // width, height-1-i // width) >= cells}

    def cell(self, x, y):
        ox, oy, yaw = self.origin
        dx, dy = x-ox, y-oy
        col = math.floor((math.cos(yaw)*dx + math.sin(yaw)*dy) / self.res)
        row = math.floor((-math.sin(yaw)*dx + math.cos(yaw)*dy) / self.res)
        return row*self.w+col if 0 <= col < self.w and 0 <= row < self.h else None

    def point(self, i):
        x, y = (i % self.w + .5)*self.res, (i // self.w + .5)*self.res
        ox, oy, yaw = self.origin
        return ox+math.cos(yaw)*x-math.sin(yaw)*y, oy+math.sin(yaw)*x+math.cos(yaw)*y

    def neighbors(self, i):
        x, y = i % self.w, i // self.w
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            xx, yy = x+dx, y+dy
            n = yy*self.w+xx
            if 0 <= xx < self.w and 0 <= yy < self.h and n in self.free:
                yield n

    def route(self, start, goal):
        if start not in self.free or goal not in self.free:
            return []
        queue, cost, parent = [(0, start)], {start: 0}, {start: None}
        while queue:
            _, i = heapq.heappop(queue)
            if i == goal:
                path = []
                while i is not None:
                    path.append(self.point(i))
                    i = parent[i]
                return path[::-1]
            for n in self.neighbors(i):
                new = cost[i] + 1
                if new < cost.get(n, float('inf')):
                    cost[n], parent[n] = new, i
                    heuristic = abs(n % self.w-goal % self.w)+abs(n // self.w-goal // self.w)
                    heapq.heappush(queue, (new+heuristic, n))
        return []

    def explore(self, start, visited):
        """Prefer reachable cells near unknown space; otherwise patrol new cells."""
        if start not in self.free:
            return None
        reachable, queue = {start: 0}, deque([start])
        while queue:
            i = queue.popleft()
            for n in self.neighbors(i):
                if n not in reachable:
                    reachable[n] = reachable[i]+1
                    queue.append(n)
        candidates = []
        ring = max(1, math.ceil(.35/self.res))
        for i, distance in reachable.items():
            if distance*self.res < .35:
                continue
            px, py = self.point(i)
            if any(math.hypot(px-x, py-y) < .3 for x, y in visited):
                continue
            x, y = i % self.w, i // self.w
            unknown = sum(0 <= x+dx < self.w and 0 <= y+dy < self.h
                          and self.data[(y+dy)*self.w+x+dx] < 0
                          for dx, dy in ((ring, 0), (-ring, 0), (0, ring), (0, -ring)))
            candidates.append((unknown*10 - distance*self.res, i))
        return max(candidates)[1] if candidates else None


class Greeting:
    """One 10 s greeting; re-arm only after cooldown and an absence."""
    def __init__(self, duration=10.0, cooldown=60.0):
        self.duration, self.cooldown = duration, cooldown
        self.until, self.next_allowed = -1.0, -1.0
        self.absent_since = None
        self.armed = True

    def observe(self, present, now):
        if present:
            self.absent_since = None
        elif self.absent_since is None:
            self.absent_since = now
        if self.absent_since is not None and now-self.absent_since >= 3 and now >= self.next_allowed:
            self.armed = True

    def start(self, now):
        if not self.armed or now < self.next_allowed:
            return False
        self.until, self.next_allowed, self.armed = now+self.duration, now+self.cooldown, False
        return True
