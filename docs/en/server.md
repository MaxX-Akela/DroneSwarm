# Working with the Server

The server is a window on the operator's computer. It shows the state of all drones in a table and sends them commands. The server is written in Python with PyQt5 and runs on Windows and Ubuntu.

## Starting

Install the dependencies and run `core.py`:

```bash
pip install PyQt5
python3 server/core.py
```

On startup the server opens three ports:

| Port | Protocol | Purpose |
|---|---|---|
| 9000 | UDP | drones look for the server |
| 9010 | TCP | commands from server → drones |
| 9001 | UDP | telemetry from drones → server |

These ports must be open in the computer's firewall. If a port is busy, `ERROR: can't bind ...` appears in the server console. The ports and other settings are changed in the [server configuration](extras/config.md).

> The computer and the drones must be on the same network. Drones find the server with a broadcast request, so there is no need to enter the server's IP address anywhere.

## Time synchronization

For the drones to start simultaneously, their clocks must match the server's clock. To achieve this, run `chrony` on the server computer with the config `server/chrony/chrony-server.conf` (also available as `examples/chrony/server.conf`):

```
local stratum 10
allow
```

This config declares the server a time source without Internet access and allows any client to connect. The drones add the server to their own `chrony` after they find it.

## Drone table

A drone appears in the table as soon as its telemetry arrives. Each cell is colored by state: green — everything is fine, yellow — warning, red — error. The color of the name cell is the worst state in the row. If there has been no telemetry for more than 5 seconds (`drone_timeout`) or the TCP link is lost, the whole row turns red and the `mode` column shows `OFFLINE`.

| Column | What it shows |
|---|---|
| copter ID | drone name (`hostname`). The checkbox selects drones that will receive commands |
| version | client version. Yellow if it differs from the server version or the code has local changes |
| animation ID | name of the loaded animation. Red if the file is missing or cannot be parsed |
| battery | battery voltage |
| system | whether there is a link to the flight controller |
| sensors | freshness of the camera-based position; yellow if the rangefinder is not responding |
| mode | flight mode and the `armed` flag |
| checks | preflight check result: `OK`, `WARN (n)` or `FAIL (n)`. Details are in the tooltip |
| current x y z yaw frame_id | the drone's position in `aruco_map` |
| start x y z | the first point of the animation |
| dt | clock offset between drone and server in seconds, and the source (`chrony` or `ntp`) |

The `start` column compares the current position with the animation's first point in the XY plane: up to 0.3 m — green, up to 1 m — yellow, further — red. This shows that a drone is not standing in its place. Once the drone has taken off, the column is not colored.

Double-clicking a cell opens a window with the drone's active config and its last 30 errors and warnings.

## Control panel

Most buttons act on the drones checked in the table.

| Button | What it does |
|---|---|
| **Проверка (Preflight check)** | runs the check on the selected drones immediately |
| **Start animation** | starts the animation on the selected drones after the time in the **Start after** field |
| **Pause** | the drone hovers at its current point |
| **Посадка (Land selected)** | lands the selected drones |
| **Land ALL** | lands all drones in the table |
| **Emergency land** | the same, but with confirmation, for all at once |
| **Взлет (Takeoff)** | takes off to the height in the **Z** field (asks for confirmation) |
| **Disarm selected / Disarm ALL** | stops the motors (asks for confirmation) |
| **Test leds** | tests the LED strip |
| **Reboot FCU** | reboots the flight controller |
| **Calibrate gyro / level** | calibrates the gyroscope and level |

The **Визуальная посадка** (visual landing) and **Flip** buttons and the **Play music** field do nothing yet; they are placeholders.

> **Disarm** in flight will make the drone fall. Use it only on the ground or in the most extreme situation.

### Starting an animation

1. Check the drones.
2. Press **Проверка** and make sure the `checks`, `start` and `dt` columns are green.
3. Set the time before the start (in seconds) in **Start after** so that you have time to step back.
4. Press **Start animation**.

If a drone fails its check, it refuses to play the animation and prints `REFUSED play` in the console. For bench tests, enable **Developer mode (ignore self-check)**.

## Menus

### Selected drones

Everything in this menu applies to the checked drones.

**Send** — sending files to the drones:

| Item | What it sends |
|---|---|
| Animations... | a folder of animations: for each drone, the file whose name contains its name is used (`clover-1.csv` for `clover-1`) |
| Animation... | a single animation file to all selected drones |
| Camera calibrations... | a folder of camera calibrations (`.yaml`); the file is chosen the same way, by drone name |
| Aruco map... | the `map.txt` marker map; Clover is restarted after upload |
| Configuration... | an `.ini` file with drone settings. **Modify** updates only the listed keys, **Rewrite** replaces the whole config |
| Launch files folder... | all `.launch` and `.yaml` files from a folder |
| FCU parameters file... | a PX4 parameter file (`.params`), loaded into the flight controller right away |
| File... | any file; a dialog asks for the path on the drone |
| Command... | any shell command; the result comes back to the console |

Where the calibration, the map and the launch files go on the drone is set in the `[paths]` section of the [server configuration](extras/config.md).

Other items:

* **Restart service** — restart `chrony`, `ros` (Clover) or `swarm` (the DroneSwarm client);
* **Update (git pull + restart)** — update the client and restart it;
* **Reload animation** — re-read the animation file;
* **Reboot** — reboot the companion computer (30–60 seconds).

> Uploading an animation, restarting services, rebooting and updating are **not performed** while the drone is airborne (`armed`). The exception is restarting `chrony`.

### Server

* **Edit server config** — edit `server/config/server.ini`;
* **Edit any config** — edit any `.ini` file;
* **Update server git** — `git fetch` and `git pull --rebase` in the server folder;
* **Restart server** — restart the server.

### Table

**Select all** and **Deselect all** — check or uncheck all drones.

## Console

The console of the server is at the bottom of the window. It shows drones connecting and disconnecting, replies to commands (`file written`, `config updated`, the result of `run_command`, etc.) and errors. Drone messages are prefixed with the drone's name in square brackets.

> The command channel is not password-protected: anyone on the network can send a command to a drone. Run shows only on a dedicated network, as recommended in the [quick start](fast_start.md).

**More about the server modules: [extras/_server.md](extras/_server.md)**
