"""Find and rank legal moves. This file never needs a RabbitMQ connection."""

import math
from time import monotonic


def point(report):
    """Read the x, y and heading of a waypoint or robot report."""
    # Turns the dicitonary wich the robot creates of its state at every 
    # tik into a three-tuple of floats. 
    # point({"x": "2.5", "y": 3, "theta": 0}) -->  Returns (2.5, 3.0, 0.0)

    result = (float(report["x"]), float(report["y"]), float(report["theta"]))
    if not all(math.isfinite(value) for value in result):
        raise ValueError("A waypoint must have finite coordinates")
    return result


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


class Footprint:
    """A fixed-heading rectangle, or the area it sweeps between two waypoints."""

    def __init__(self, start, end):
        self.start, self.end = start, end
        self.length, self.width = 1.430 / 2, 0.630 / 2  # Half-dimensions in meters as position is assumed to be the center.
        angle = start[2] # this is theta. 
        self.forward = (math.cos(angle), math.sin(angle)) #Direction is turned from angle to vector with x, y components.
        self.side = (-math.sin(angle), math.cos(angle)) #Direction of width.
        # These directions ("axes") are used to determine possible collisions later on.  
        self.axes = [self.forward, self.side]
        
    # shadow detrmines the area that is needed to be free for a move. 
    # f.e if x=0 is starting point and x=2 is endpoint. The whole corridor (inclduing length and widthof robot)
    # needs to be free to move legally. 
    def shadow(self, axis):
        """The two endpoints of the shape's shadow on a line."""
        a, b = dot(self.start, axis), dot(self.end, axis)
        margin = abs(dot(self.forward, axis)) * self.length + abs(dot(self.side, axis)) * self.width
        return min(a, b) - margin, max(a, b) + margin

    
    def overlaps(self, other):
        for axis in self.axes + other.axes:
            left, right = self.shadow(axis)
            other_left, other_right = other.shadow(axis)
            if right < other_left - 1e-8 or other_right < left - 1e-8:
                return False  # We want a nonzero gap between robots in any direction.
        return True
        #False means--> No overlap, with this no ccollision will occur.

    # Now we need to define moving/stopping and then a way to avoid deadlocks.
class Move:
    def __init__(self, robot, stop_at, crossings_cleared):
        self.robot = robot
        self.stop_at = stop_at
        self.crossings_cleared = crossings_cleared

    def result(self, positions):
        after = list(positions)
        after[self.robot] = self.stop_at
        return tuple(after)


class CrossingPlanner:
    def __init__(self, routes):
        self.goals = tuple(len(path) - 1 for path in routes)
        self.parked = [[Footprint(p, p) for p in path] for path in routes]
        self.steps = []
        for robot, path in enumerate(routes):
            sweeps = [Footprint(path[i], path[i + 1]) for i in range(len(path) - 1)]
            # A robot still occupies its last waypoint after reaching its goal.
            self.steps.append(sweeps + [self.parked[robot][-1]])

        # For each robot and each neighbor, record the neighbor's last
        # conflicting step for every step on this robot's route.
        self.conflicts = [{} for path in routes]
        for a in range(len(routes)):
            for b in range(a + 1, len(routes)):
                a_steps = [-1] * len(routes[a])
                b_steps = [-1] * len(routes[b])
                for i, step_a in enumerate(self.steps[a]):
                    for j, step_b in enumerate(self.steps[b]):
                        if step_a.overlaps(step_b):
                            a_steps[i], b_steps[j] = j, i
                if max(a_steps) >= 0:
                    self.conflicts[a][b] = a_steps
                    self.conflicts[b][a] = b_steps

        self.goals_are_clear = self.overlap_at(self.goals) is None
        self.answers = {self.goals: self.goals_are_clear}
        self.checked_advances = {}

    def overlap_at(self, positions):
        for a in range(len(positions)):
            for b in range(a + 1, len(positions)):
                if self.parked[a][positions[a]].overlaps(self.parked[b][positions[b]]):
                    return a, b
        return None

    def crossings_left(self, robot, positions):
        """Return each pending crossing's entry step and clearance waypoint."""
        crossings = []
        for other, steps in self.conflicts[robot].items():
            pending = []
            for step in range(positions[robot], len(steps)):
                if steps[step] >= positions[other]:
                    pending.append(step)
            if pending:
                crossings.append((pending[0], pending[-1] + 1))
        return crossings

    def counts(self, positions):
        counts = [len(self.crossings_left(robot, positions)) for robot in range(len(positions))]
        return sum(counts) // 2, counts  # Each pair appears on two robots' lists.

    def clear_to_stop(self, robot, start, stop, positions):
        for other, waypoint in enumerate(positions):
            if robot == other:
                continue
            key = (robot, start, stop, other, waypoint)
            if key not in self.checked_advances:
                parked = self.parked[other][waypoint]
                self.checked_advances[key] = not any(
                    self.steps[robot][step].overlaps(parked) for step in range(start, stop)
                )
            if not self.checked_advances[key]:
                return False
        return True

    def possible_moves(self, positions, deadline):
        for robot, goal in enumerate(self.goals):
            if monotonic() >= deadline:
                raise TimeoutError("Legal-move search reached its time budget")
            start = positions[robot]
            if start == goal:
                continue

            crossings = self.crossings_left(robot, positions)
            stop = goal
            if crossings:
                crossings.sort(key=lambda crossing: crossing[1])
                clear_at = crossings[0][1]
                if clear_at > goal:
                    continue
                next_entry = min((entry for entry, _ in crossings[1:]), default=goal)
                # If two crossings overlap, stop as soon as the first one clears.
                stop = max(clear_at, next_entry)

            if self.clear_to_stop(robot, start, stop, positions):
                cleared = sum(clear_at <= stop for _, clear_at in crossings)
                yield Move(robot, stop, cleared)

    def can_finish(self, positions, deadline):
        if positions in self.answers:
            return self.answers[positions]
        for move in self.possible_moves(positions, deadline):
            if self.can_finish(move.result(positions), deadline):
                self.answers[positions] = True
                return True
        self.answers[positions] = False
        return False

    def legal_moves(self, positions, search_seconds=0.4):
        if not self.goals_are_clear or self.overlap_at(positions) is not None:
            return []
        deadline = monotonic() + search_seconds
        legal = []
        for move in self.possible_moves(positions, deadline):
            if self.can_finish(move.result(positions), deadline):
                legal.append(move)
        return legal



#weights used in the scheduling problem. This can be heavily improved. 
# The upside is that changes should be conflict free
def weight(report, now_ms):
    """The provisional weight. The final urgency formula can go here."""
    seconds_left = max(1, (report["deadline"] - now_ms) / 1000)
    battery = max(1, report["battery_level"])
    return 1 / (seconds_left * battery)



# The rank is determined through considering the urgency to be a weighted
# scheduling problem 1||sum wjCj, where the weight is determined by the battery level and the time left to complete the task
# loaded or unloaded is NOT taking into consideration as 
# I couldnt really find a reason why loaded would be prioritized over unloaded. 
# It might be that the unloaded robot is prioritized since it has to pick up a important load. 

def rank_moves(moves, robots, now_ms):
    ranked = []
    for move in moves:
        robot = robots[move.robot]
        count = robot.report["remaining_crossings"]
        score = weight(robot.report, now_ms) / count if count else 0 # This is basically Smiths rule.
        ranked.append((move, score))
    # Bigger scores go first. Use the device ID when scores are tied.
    return sorted(ranked, key=lambda item: (-item[1], robots[item[0].robot].device_id))
