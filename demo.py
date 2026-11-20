"""Optional mock robots loaded from test_instances.json."""

import argparse
import json
import os
from pathlib import Path
from time import time

from decision import Footprint, point
from monitor import CollisionMonitor, PAUSE, RESUME, STATE_QUEUE, action_queue, ensure_queue


class MockRobot:
    def __init__(self, device_id, path, battery):
        self.device_id = device_id
        self.path = path
        self.battery = battery
        self.waypoint = 0
        self.action = PAUSE

    def at_goal(self):
        return self.waypoint == len(self.path) - 1

    def report(self, timestamp, deadline):
        x, y, theta = self.path[self.waypoint]
        return {
            "device_id": self.device_id, "timestamp": timestamp,
            "x": x, "y": y, "theta": theta, "battery_level": self.battery,
            "loaded": False, "deadline": deadline,
            "path": [{"x": x, "y": y, "theta": theta} for x, y, theta in self.path],
        }


def run_example(robots, send_reports, wait=None):
    started = int(time() * 1000)
    goal_reports = 0
    tick_limit = 5 * (1 + sum(len(robot.path) for robot in robots))
    for tick in range(tick_limit):
        moving = [robot for robot in robots if robot.action == RESUME and not robot.at_goal()]
        if len(moving) > 1:
            raise RuntimeError("More than one mock robot was resumed")
        for robot in moving:
            sweep = Footprint(robot.path[robot.waypoint], robot.path[robot.waypoint + 1])
            for other in robots:
                if other is not robot:
                    parked = other.path[other.waypoint]
                    if sweep.overlaps(Footprint(parked, parked)):
                        raise RuntimeError("The next mock move would collide")
            robot.waypoint += 1
            x, y, _ = robot.path[robot.waypoint]
            print(f"{robot.device_id} moved to ({x}, {y})", flush=True)

        timestamp = started + tick * 1000
        send_reports([robot.report(timestamp, started + 60_000) for robot in robots])
        if all(robot.at_goal() for robot in robots):
            goal_reports += 1
            if goal_reports == 4:  # Allow time for the monitor to confirm the stop.
                print("Demo complete. All robots reached their goals.", flush=True)
                return
        if wait:
            wait()
    raise RuntimeError(f"The demo reached {tick_limit} ticks before all robots finished")


def run_locally(robots):
    by_id = {robot.device_id: robot for robot in robots}

    def apply_action(device_id, action):
        by_id[device_id].action = action

    monitor = CollisionMonitor(list(by_id), apply_action, show_trace=False)

    def send_reports(reports):
        for report in reports:
            monitor.receive_state(report)
        monitor.tick()

    run_example(robots, send_reports)


def run_with_rabbitmq(robots):
    import pika

    url = os.getenv("AMQP_URL", "amqp://guest:guest@localhost:5672/%2F")
    with pika.BlockingConnection(pika.URLParameters(url)) as connection:
        ensure_queue(connection, STATE_QUEUE)
        for robot in robots:
            ensure_queue(connection, action_queue(robot.device_id))
        channel = connection.channel()

        def send_reports(reports):
            for report in reports:
                channel.basic_publish(exchange="", routing_key=STATE_QUEUE, body=json.dumps(report))

        def wait_for_next_step():
            connection.sleep(1)
            for robot in robots:
                queue = action_queue(robot.device_id)
                while True:
                    delivery, _, body = channel.basic_get(queue, auto_ack=True)
                    if delivery is None:
                        break
                    robot.action = json.loads(body)["action"]

        run_example(robots, send_reports, wait_for_next_step)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local", action="store_true", help="Run without RabbitMQ")
    parser.add_argument("--instance", type=int, default=1, help="Test instance number, starting at 1")
    args = parser.parse_args()
    with Path(__file__).with_name("test_instances.json").open() as file:
        instances = json.load(file)
    # Each instance is a list of robot state messages. Each mock starts at its first waypoint.
    if not instances:
        parser.error("test_instances.json is empty. Add a test instance before running the demo.")
    if not 1 <= args.instance <= len(instances):
        parser.error(f"Choose an instance number from 1 to {len(instances)}.")
    robots = []
    for report in instances[args.instance - 1]:
        path = [point(waypoint) for waypoint in report["path"]]
        robots.append(MockRobot(report["device_id"], path, report["battery_level"]))
    if not robots:
        parser.error("This test instance has no robots.")
    if args.local:
        run_locally(robots)
    else:
        run_with_rabbitmq(robots)


if __name__ == "__main__":
    main()
