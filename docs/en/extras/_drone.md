# Drone Module Description

The drone client is in the `drone/` folder and is started by the `client.py` file. It works on top of ROS (Clover) and MAVROS. This page is for those who want to understand how the client works inside. If you only need to use it, see [Working with the Drone](../drone.md).

## Structure

```
drone/
├── client.py            # entry point, network part
├── update.py            # client update
├── config.example.ini   # settings template
└── modules/
    ├── animation.py     # animation parsing, preparation and frame execution
    ├── checks.py        # preflight check
    ├── commander.py     # task queue (TaskManager)
    ├── config.py        # reads drone.ini
    ├── failsafe.py      # emergency landing on loss of position
    ├── flight.py        # flight commands through Clover services
    ├── led.py           # LED strip control
    ├── network.py       # time: chrony and NTP
    ├── remote.py        # files, commands and services on the server's request
    ├── utils.py         # drone name, logging
    └── version.py       # client version
```

## client.py

On startup the client:

1. creates the ROS node `drone_swarm_client`;
2. takes the drone name from `hostname` — this is the `copter_id`;
3. reads `drone/drone.ini`;
4. creates a `TaskManager` (executes commands) and a `Watchdog` (failsafe), which is checked every 0.5 s;
5. starts the network layer — four threads.

The network layer is the `NetworkManager` class. Its threads:

| Thread | Period | What it does |
|---|---|---|
| commands | continuous | looks for the server by UDP broadcast, connects over TCP, receives and executes commands |
| telemetry | 0.5 s | sends the state to the server over UDP |
| check | 5 s | runs `checks.self_check()` |
| time | 5 s | measures the clock offset |

### Protocol

All messages are JSON.

**Discovery (UDP, port 9000).** The client sends a broadcast request, the server replies:

```json
{"type": "discover", "copter_id": "clover-1"}
{"type": "discover_reply", "tcp_port": 9010}
```

**Commands (TCP).** Each message: 4 bytes of length (big-endian) + JSON. The client first sends a greeting, then receives commands:

```json
{"type": "hello", "copter_id": "clover-1"}
{"action": "takeoff", "params": {"height": 1.5}}
{"type": "log", "text": "config updated (modify)"}
```

The last one is a client reply: the text appears in the server console.

**Telemetry (UDP, port 9001).** A single JSON with all fields: `copter_id`, `version`, `x/y/z/yaw`, `frame_id`, `bat`, `armed`, `mode`, `connected`, `states` (the state of each column: `ok`/`warn`/`fail`), `animation_id`, `animation_state`, `start_pos`, `checks`, `config`, `errors`, `time_offset`.

### Server commands

| Command (`action`) | Parameters | What happens |
|---|---|---|
| `check` | — | runs the check in a separate thread |
| `set_config` | `ini_text`, `mode` (`modify`/`rewrite`) | writes `drone.ini` and reloads the settings |
| `set_animation` | `csv_text` | writes `animation.csv` and reloads the animation |
| `write_file` | `path`, `data` (base64), `restart` | writes a file, restarts a service if needed |
| `run_command` | `command` | runs a shell command |
| `restart_service` | `name` | restarts `chrony`, `ros` or `swarm` |
| `reboot` | — | reboots the system |
| `load_fcu_params` | `path`, `data` | saves the file and loads the parameters into PX4 through `rosrun mavros mavparam` |
| `play`, `takeoff`, `land`, `stop`, ... | depend on the command | passed to the `TaskManager` |

If the drone's motors are running (`armed`), the client refuses `reboot`, service restarts (except `chrony`) and replacing the animation. The `play` command is not executed until the last check passes (unless the parameters contain `ignore_checks`).

> The command channel is not protected: the server can run any command on the drone. This is a deliberate simplification for a closed show network.

## modules/commander.py

`TaskManager` runs **one task at a time**. The `do_action()` method first raises the `interrupter` flag, waits up to 2 seconds for the current task to stop, and then queues the new one. All wait loops in `flight` and `animation` check this flag every 0.05–0.2 s, so landing or pause interrupt everything else.

The `_play_animation()` function:

1. requires `animation.state == "OK"` and a known position in `aruco_map`;
2. chooses the action: `takeoff` or `fly` (see the [animation module](animation.md));
3. if `start_time` is given, converts it to local time taking the clock offset into account and waits;
4. walks through the frames: waits for the frame's moment and calls `execute_frame()`.

## modules/flight.py

A thin wrapper over the Clover services: `navigate`, `land`, `get_telemetry`, `mavros/cmd/arming`, `mavros/cmd/command`.

* All flights are in the `aruco_map` coordinate frame (`FRAME_ID`).
* `navto()` does not block: it sends a point and returns immediately. It is used for animation frames, whose timing is set by the schedule.
* `reach_point()` blocks until the drone arrives (0.2 m tolerance) or the time runs out (20 s).
* `takeoff()` climbs up relative to `body`, then checks the height in `aruco_map`.
* `land()` calls landing and waits for disarm; if time runs out and the height is below 0.3 m, it stops the motors forcibly.
* `stop()` sets the target at the current position, i.e. hovering.

## modules/checks.py

`self_check()` returns `{"ok": bool, "problems": [...], "warnings": [...]}`. The checks: FCU, battery, ArUco map visibility, availability of ROS services (`navigate`, `land`, `get_telemetry`, `led/set_effect`). The battery is evaluated per cell; a warning appears only when there are no problems. The `battery_state()` function is also used for the `battery` column on the server.

## modules/failsafe.py

`Watchdog` subscribes to the position `mavros/vision_pose/pose`, the FCU state and the rangefinder.

* If the drone is armed, in `OFFBOARD` mode, and the position has not been updated for more than 2 seconds, `AUTO.LAND` mode is switched on.
* `sensors_ok()` is the freshness of the position regardless of whether the drone is flying (the `sensors` column).
* `rangefinder_ok()` — whether rangefinder data arrives (gives the yellow color of the column).

## modules/remote.py

Service actions on the server's request: writing a file (atomic, with `sudo tee` for system paths), merging `.ini` in `modify` mode, running a command (60 s timeout, output truncated to 4000 characters), restarting services. Service names map like this:

| Name in the server menu | systemd service |
|---|---|
| `chrony` | `chrony` |
| `ros` | `clover` |
| `swarm` | `droneswarm` |

## modules/network.py

Time handling. `get_time_offset()` first asks `chronyc tracking`; if `chrony` is not synchronized, it tries an NTP request to the server. `set_chrony_server()` adds the found server to `chrony` with `chronyc add server`, so no IP is needed in the config. The drone's `chrony.conf` (`makestep 1 3`) only allows quick time adjustment after boot.

## modules/version.py and update.py

The client version is `branch@commit` (if installed from git; an asterisk at the end means local changes) or the version of the `drone-swarm` package (if installed from apt). `update.py` runs `git pull --ff-only` or `apt-get install --only-upgrade`, and then restarts the service.

## modules/utils.py and led.py

`utils.py` sets up logging: the file `<hostname>.log`, the console and a ring buffer of the last 30 warnings and errors (it is sent to the server). `led.py` calls the Clover service `led/set_effect`; `test()` turns on red, green and blue in turn.
