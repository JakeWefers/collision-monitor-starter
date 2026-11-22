"""Receive robot reports from RabbitMQ and send Pause or Resume."""

import argparse
import json
import math
import os
from time import monotonic, time

from decision import CrossingPlanner, point, rank_moves


STATE_QUEUE = "robot.state"
PAUSE = "Pause"
RESUME = "Resume"


def action_queue(device_id):
    return f"robot.{device_id}.actions"


def ensure_queue(connection, name):
    """Create a shared durable queue or use one the robot simulator already made."""
    import pika

    try:
        with connection.channel() as channel:
            channel.queue_declare(queue=name, durable=True)
    except pika.exceptions.ChannelClosedByBroker as error:
        if error.reply_code != 406:
            raise
        # An existing queue can have different settings. Check it on a fresh channel.
        with connection.channel() as channel:
            channel.queue_declare(queue=name, passive=True)


def read_report(message):
    """Check the fields used by the monitor and keep a copy of the report."""
    report = message.copy()
    if not isinstance(report["device_id"], str) or not report["device_id"].strip():
        raise ValueError("Expected a device ID")
    if not isinstance(report["loaded"], bool) or not report["path"]:
        raise ValueError("Expected a load status and a path")
    report["timestamp"] = int(report["timestamp"])
    report["deadline"] = int(report["deadline"])
    report["battery_level"] = float(report["battery_level"])
    if not 0 <= report["battery_level"] <= 100:
        raise ValueError("Expected battery level from 0 to 100")
    point(report)
    for waypoint in report["path"]:
        point(waypoint)
    return report


class Robot:
    def __init__(self, report):
        self.device_id = report["device_id"]
        self.path = tuple(point(waypoint) for waypoint in report["path"])
        heading = self.path[0][2]
        for waypoint in self.path[1:]:
            if abs(math.remainder(waypoint[2] - heading, 2 * math.pi)) > 1e-8:
                raise ValueError(f"{self.device_id} must keep the same heading along its path")
        self.waypoint = 0
        self.still_reports = 0
        self.update(report)

    def update(self, report):
        x, y, heading = point(report)
        path = tuple(point(waypoint) for waypoint in report["path"])
        for index in range(self.waypoint, len(self.path)):
            px, py, planned_heading = self.path[index]
            distance = math.hypot(x - px, y - py)
            angle_error = abs(math.remainder(heading - planned_heading, 2 * math.pi))
            if distance <= 1e-5 and angle_error <= 1e-5:
                if path != self.path and path != self.path[index:]:
                    raise ValueError(f"{self.device_id} changed its route")
                self.still_reports = self.still_reports + 1 if index == self.waypoint else 1
                self.waypoint = index
                self.report = report.copy()
                self.report["remaining_crossings"] = None  # The planner fills this in each tick.
                self.last_seen = monotonic()
                return
        raise ValueError(f"{self.device_id} reported a position outside its remaining route")


class CollisionMonitor:
    def __init__(self, robot_ids, send_action, search_seconds=0.4, show_trace=True):
        self.robot_ids = robot_ids
        self.send_action = send_action
        self.search_seconds = search_seconds
        self.show_trace = show_trace
        self.robots = {}
        self.planner = None
        self.remaining_crossings = None
        self.active = None
        self.stopping = False
        self.fault = None

    def pause_all(self):
        for device_id in self.robot_ids:
            self.send_action(device_id, PAUSE)

    def fail(self, reason):
        self.fault = reason
        self.pause_all()
        print(reason, flush=True)

    def receive_state(self, report):
        """Handle one report immediately, especially at the planned stopping point."""
        device_id = report["device_id"]
        if device_id not in self.robot_ids:
            self.fail(f"Unexpected robot {device_id}")
            return
        robot = self.robots.get(device_id)
        if robot and report["timestamp"] <= robot.report["timestamp"]:
            return  # A duplicate or older report cannot confirm a stop.

        previous = robot.waypoint if robot else None
        try:
            if robot:
                robot.update(report)
            else:
                robot = Robot(report)
                self.robots[device_id] = robot
        except ValueError as error:
            self.fail(str(error))
            return

        if self.planner is None or self.fault:
            return
        if self.active is None or device_id != self.robot_ids[self.active.robot]:
            if robot.waypoint != previous:
                self.fail(f"{device_id} moved while expected to be paused")
            return

        if robot.waypoint > self.active.stop_at:
            self.fail(f"{device_id} moved past its planned stopping waypoint")
        elif robot.waypoint == self.active.stop_at:
            if not self.stopping:
                self.send_action(device_id, PAUSE)
                self.stopping = True
            elif robot.still_reports >= 3:  # The arrival report and two newer reports.
                self.active = None
                self.stopping = False

    def decide(self, robots, positions, now_ms):
        if self.fault:
            return self.fault
        if len(robots) < len(self.robot_ids):
            return "Waiting for reports from every robot"
        if any(monotonic() - robot.last_seen > 3 for robot in robots):
            if self.active:
                self.fail("A robot stopped reporting during a move")
                return self.fault
            if self.planner is None:
                for robot in robots:
                    robot.still_reports = 0
            return "Waiting for fresh reports"
        if self.planner is None:
            if any(robot.still_reports < 2 for robot in robots):
                return "Confirming starting positions"
            self.planner = CrossingPlanner([robot.path for robot in robots])
            self.update_counts(robots, positions)

        if self.active:
            return "Confirming stop" if self.stopping else "Moving to the stopping waypoint"
        if self.planner.overlap_at(positions) is not None:
            self.fail("Two robots are already overlapping")
            return self.fault
        if positions == self.planner.goals:
            return "All robots reached their goals"

        if not self.planner.goals_are_clear:
            return "No legal crossing sequence reaches all goals"

        deadline = monotonic() + self.search_seconds
        try:
            candidates = self.planner.possible_moves(positions, deadline)
            ranked = rank_moves(candidates, robots, now_ms)

            for move, _ in ranked:
                next_positions = move.result(positions)
                if self.planner.can_finish(next_positions, deadline):
                    self.active = move
                    return "Selected the highest-ranked legal move"
        except (TimeoutError, RecursionError):
            return "Legal-move search reached its limit"

        return "No legal crossing sequence reaches all goals"

    def tick(self):
        """Make a decision, send the actions, and print one trace entry."""
        now_ms = int(time() * 1000)
        robots = [self.robots[name] for name in self.robot_ids if name in self.robots]
        positions = tuple(robot.waypoint for robot in robots)
        had_planner = self.planner is not None
        if had_planner:
            self.update_counts(robots, positions)
        reason = self.decide(robots, positions, now_ms)

        actions = {}
        for name in self.robot_ids:
            actions[name] = PAUSE
        if self.active and not self.stopping and not self.fault:
            actions[self.robot_ids[self.active.robot]] = RESUME
        for name, action in actions.items():
            self.send_action(name, action)

        if self.show_trace:
            trace = {
                "timestamp": now_ms,
                "robots": [
                    {
                        "device_id": robot.device_id,
                        "waypoint": robot.waypoint,
                        "x": robot.report["x"],
                        "y": robot.report["y"],
                        "theta": robot.report["theta"],
                        "report_timestamp": robot.report["timestamp"],
                        "battery_level": robot.report["battery_level"],
                        "deadline": robot.report["deadline"],
                        "still_reports": robot.still_reports,
                    }
                    for robot in robots
                ],
                "remaining_crossings": self.remaining_crossings,
                "active_move": {
                    "device_id": self.robot_ids[self.active.robot],
                    "stop_at_waypoint": self.active.stop_at,
                    "waiting_for_stop": self.stopping,
                } if self.active else None,
                "actions": actions,
                "reason": reason,
            }
            if not had_planner and self.planner is not None:
                trace["routes"] = {robot.device_id: robot.path for robot in robots}
            print(json.dumps(trace), flush=True)

    def update_counts(self, robots, positions):
        self.remaining_crossings, counts = self.planner.counts(positions)
        for index, robot in enumerate(robots):
            robot.report["remaining_crossings"] = counts[index]


def run_service(args):
    import pika

    settings = pika.URLParameters(args.amqp_url)
    settings.blocked_connection_timeout = 5
    with pika.BlockingConnection(settings) as connection:
        ensure_queue(connection, args.state_queue)
        for name in args.robot_ids:
            ensure_queue(connection, action_queue(name))
        channel = connection.channel()
        channel.confirm_delivery()

        def send_action(device_id, action):
            body = json.dumps({"device_id": device_id, "action": action})
            channel.basic_publish(exchange="", routing_key=action_queue(device_id),
                                  body=body, mandatory=True,
                                  properties=pika.BasicProperties(content_type="application/json"))

        monitor = CollisionMonitor(args.robot_ids, send_action, args.search_seconds)
        monitor.pause_all()
        next_tick = monotonic()
        try:
            for delivery, _, body in channel.consume(args.state_queue, inactivity_timeout=0.1):
                if delivery is not None:
                    try:
                        report = read_report(json.loads(body))
                    except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as error:
                        print(f"Invalid state report: {error}", flush=True)
                    else:
                        monitor.receive_state(report)
                    channel.basic_ack(delivery_tag=delivery.delivery_tag)
                now = monotonic()
                if now >= next_tick:
                    next_tick = now + 1.0
                    monitor.tick()
        except KeyboardInterrupt:
            pass
        finally:
            if channel.is_open:
                monitor.pause_all()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot_ids", nargs="*", default=os.getenv("ROBOT_IDS", "").split())
    parser.add_argument("--amqp-url", default=os.getenv("AMQP_URL", "amqp://guest:guest@localhost:5672/%2F"))
    parser.add_argument("--state-queue", default=STATE_QUEUE)
    parser.add_argument("--search-seconds", type=float, default=0.4)
    args = parser.parse_args()
    if not args.robot_ids or len(set(args.robot_ids)) != len(args.robot_ids):
        parser.error("Supply a list of unique robot IDs")
    if not 0 < args.search_seconds < 1:
        parser.error("--search-seconds must be between 0 and 1")
    run_service(args)


if __name__ == "__main__":
    main()
