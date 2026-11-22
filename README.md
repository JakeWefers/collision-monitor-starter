# Collision Monitor



## Run the demo

From the project folder, run this with Python

```sh
python3 demo.py --local
```

 This demo uses only Python's standard library. The mock robots pass their reports directly to the monitor and print their waypoint advances.

The scenario in `test_instances.json` gives robots A through H eight distinct routes through a shared area.

## Run the RabbitMQ service

To check the message exchange, use a running RabbitMQ broker with empty test queues. The commands below use a virtual environment on macOS or Linux. On Windows, use `.venv\Scripts\python.exe` for the environment's Python.

Create the environment and start the monitor in one terminal.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python monitor.py A B C D E F G H
```

In a second terminal in the same project folder, start the mock robots.

```sh
.venv/bin/python demo.py
```

Both programs default to a local RabbitMQ broker on port 5672. The monitor reads `robot.state` and publishes to each robot's action queue. Its terminal shows the per-tick trace. The mock robots report once a second, so this run takes longer than the local demo. Stop the monitor with Ctrl+C after the demo finishes.

Supporting documentation (onpaper to be uploaded)


Explanation of how the logic is set up: 

What is bad about this: 