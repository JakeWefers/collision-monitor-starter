# Collision Monitor



## Quick start (Docker-free)

From the project folder, run the eight-robot demo with Python 3.

```sh
python3 demo.py --local
```

On Windows, use `py -3 demo.py --local` if `python3` is unavailable. The demo uses only Python's standard library. The mock robots pass their reports directly to the monitor and print their waypoint advances. The last line is `Demo complete. All robots reached their goals.`

The scenario in `test_instances.json` gives robots A through H eight distinct routes through a shared area.

## Test the live RabbitMQ service without Docker

To test the message exchange, start a RabbitMQ broker with empty test queues. The commands below use a virtual environment on macOS or Linux. On Windows, use `.venv\Scripts\python.exe` for the environment's Python.

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

WHAT IS BAD?:

- Collision avoidance is NOT guaranteed. The stop command is sent once the intended stoping point is arrived. Until RabbitMQ sends the pause command, at least one second has passed so the robot overshoots and might crash in this time. 
    --> Stopping command must be done while the robot is still moving just before the points where it should stop. 

- One deadlocked position stops the entire fleet. If f.e A,B,C are in deadlock but D has a free path and could complete its task, it wont because can_finish() only is true if all robots can finish their trip. 
    --> Can finish should not be a global variable but for every robot indivudally. 

- Smiths rule is only a crude heuristic. I have no expermintal data on how important to actually 
make the number of crossing, battery status or the number of crossings, my weight is just made up by feel. 

- As currently implemented, a move is only allowed if we can simulate all robots to finish their entire route after this move has happened. Although this avoids deadlock/unfinished state, it is increadibly expensive in something like O(n^n). 
        --> Instead of cheking if all robots could completely finsih after the propsed move by A, 
            just check if 2 or 3 moves are possible for each robot. Then we move to polynomial time. 

        --> Even better: If we propose A to move next, and f.e robot D is never crossing its path, 
            we probably dont need to check if the move deadlocks D. There is alot of redundacy here 
            that once could get rid of. 

- I use no parallelzation. 
    Only one robot moves at a time, This is for "control" and "safety" but If robots A and B are on 
    different corners of the map, then they should be able to move at the same time, 
     --> We would then model as a 2||sum(wjCj) sheduling problem. 

     This is also the main change I would perform to scale this to 100/1000 robots. 
     We would additionally need logic to seperate these groups. 

- Instances in which simulatenous moves are necessary are not considered at all. 
My solution would leave these instances in constant PAUSE state. 

- My implementation of information exhange using RabbitMQ is done with Codex. I didnt know how to do this myself so I had an LLM write this part of the Code. Unfortunately I am not able to verify if everything that happens here is sensible or not myself. 





HOW IT WORKS:

- Receive robot states through RabbitMQ. Each robot publishes its state about once per second to the shared robot.state queue. The monitor reads the reports as they arrive and updates each robot’s current waypoint. The state uses the json format provided in the task requirements. 

- Find candidate moves. When the fleet is ready for another move (which is after the previous move is finished), the monitor checks which robots can drive to their next stopping waypoint without their footprints intersecting a parked robot. 
    - About stopping wypoints: 
       - If the crossings are separate, the robot’s whole footprint clears the first crossing, then    the robot continues to the waypoint just before it would enter the next one.
        - If crossings overlap, the robot stops at the first waypoint where its whole footprint has cleared the first crossing. IT CAN STILL BLOCK ANOTHER WAYPOINT HERE.
        If there’s no next crossing, the stopping waypoint is the robot’s goal (so last waypoint).

- As we perform only one move at a time, we consider this to be a scheduling problem of type 
    1||sum(wj Cj). For this problem, I know that Smiths rule performs quite well and is VERY CHEAP.  The monitor sorts candidates by descending weight / remaining_crossings. The remaining crossings estimate processing time, and a closer deadline or lower battery increases the weight. A robot with no crossings left gets a score of zero and can still take its final leg.


- Check for deadlocks in ranked order. For each candidate, the monitor simulates the move and uses dynamic programming to check whether all robots can still reach their goals after this proposed move has been executed. It saves answers by waypoint positions so different move orders can reuse them. It selects the first candidate that passes.
        - If the priority queue chooses A before B, then we check moves of A, and then to avoid deadlock how B would move. If it determined that at one postional state (f.e A at waypoint 3 
        B at waypoint 7) there is no way to finish, it saves the state FALSE in this position. This 
        can be reused everytime in later checks that A and B might be in this constellation. 

- It is beacuse of this check, that every approved move preserves a way to finish, and every completed move advances a waypoint, so the fleet cannot cycle back to an earlier set of positions. This holds when the planner can find a feasible one-robot-at-a-time sequence and the robots execute it as planned. The monitor can still leave everyone paused if the search times out or the layout requires simultaneous moves.

- Send commands and confirm the stop through RabbitMQ. The monitor sends Resume to the selected robot and Pause to the others through their individual robot.<device_id>.actions queues. When the moving robot reports that it has reached its stopping waypoint, the monitor immediately sends it Pause. It waits for two newer reports showing the same waypoint (so, if after two ticks the positon (x,y) hasnt changed, this tells us that indeed the robot has successfully stopped) before choosing the next move.

