# Server Module Description

The server is in the `server/` folder. It is a PyQt5 application: a window with a drone table and a network part that runs in background threads. If you only need to use it, see [Working with the Server](../server.md).

## Structure

```
server/
├── core.py                  # window, table, menus, buttons
├── config/
│   └── server.example.ini   # settings template
├── chrony/
│   └── chrony-server.conf   # chrony config for the server computer
└── modules/
    ├── config.py            # reads server.ini
    ├── network.py           # network: discovery, commands, telemetry
    └── version.py           # server version and git pull
```

## modules/network.py

The `NetworkManager` class starts three threads. They listen on the ports set in the [configuration](config.md).

| Thread | Port | What it does |
|---|---|---|
| discovery | UDP 9000 | answers `discover` with a `discover_reply` message carrying the TCP port |
| commands | TCP 9010 | accepts drone connections, reads the `hello` handshake and `log` replies |
| telemetry | UDP 9001 | receives JSON with the drone state and passes it to the window |

The message format is described in the [drone module description](_drone.md#protocol).

Details:

* each drone gets its own `DroneConnection` object with its own sender thread and queue. A stuck drone does not block the window or commands to the others;
* if a drone with the same name connects again, the old connection is closed;
* the socket enables `TCP_NODELAY` and `SO_KEEPALIVE`;
* if a message cannot be sent in full, the connection is closed: a partially sent frame would corrupt the stream. The drone will find the server again by itself;
* `_last_seen` (receive time) and `_addr` (the drone's IP) are added to the telemetry;
* `send_command()` sends a command to one drone, `broadcast_command()` to a list. If there is no connection, the command is not sent and a warning is printed to the console.

Data is passed to the window through Qt signals: `telemetry_received`, `drone_disconnected`, `log_message`.

## modules/config.py

Reads `server/config/server.ini` and substitutes the defaults when a key is missing. Keys are joined as `section_key`: the `tcp_port` parameter from the `[network]` section is available as `config.network_tcp_port`. The full parameter list: [Configuration](config.md).

## modules/version.py

`get_version()` returns `branch@commit`, and an asterisk means local changes. The server compares this string with the drone versions in the `version` column. `git_pull()` runs `git fetch` and `git pull --rebase`.

## core.py

The main class is `DroneDashboard`.

**Table.** One row per drone; the columns are listed in the [server description](../server.md#drone-table). The state of each cell is computed in `update_row()`:

* `version_state()` — the version matches the server's;
* `animation_state()` — the animation is loaded and has a name;
* `dt_state()` — clock offset (< 0.05 s — `ok`, < 0.2 s — `warn`);
* `start_state()` — distance from the current position to the animation's start point (≤ 0.3 m — `ok`, ≤ 1 m — `warn`);
* the other states (`battery`, `system`, `sensors`, `mode`, `position`) come from the drone itself in the `states` field.

A row is considered disconnected if there is no TCP connection or no telemetry for more than `drone_timeout` seconds (the `check_stale` timer).

**Sending files.** Animations, calibrations and other files are read on the server and sent to the drone inside the command as text or base64. For folders (`Animations...`, `Camera calibrations...`) the file for a drone is found by name: the file name must contain the `copter_id` as a separate fragment. So `clover-1.csv` suits drone `clover-1`, but not drone `clover-10`.

**Updating.** The **Update** item sends the drone a `run_command` with one shell command: `git pull --ff-only` if the drone has a git copy, otherwise `apt-get install --only-upgrade drone-swarm`; then after 3 seconds the `droneswarm` service restarts. Thanks to this, drones with an old client version can be updated too.

## Not implemented

* **Play music** and **Визуальная посадка** (visual landing) are placeholders without logic;
* **Flip** sends a command, but the client ignores it.
