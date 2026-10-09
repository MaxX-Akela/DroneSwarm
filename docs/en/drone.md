# Working with the Drone (Client)

The client is the `client.py` program that runs on the drone's companion computer (Raspberry Pi 4 or Orange Pi 5 Pro; a ready-made image for Orange Pi 5 Pro and Technic 6S is still in development) and controls the drone on the server's commands. How to install the client is described in the [quick start](fast_start.md). This page describes what the client does after it starts and how to work with it.

## How the client starts

The client runs as the systemd service `droneswarm` and starts together with the system. It runs as user `pi` and waits for Clover (`clover.service`) to come up.

Check the status and view the log:

```bash
sudo systemctl status droneswarm
journalctl -u droneswarm -f
```

The client also writes a log to the file `<drone-name>.log` in its working folder.

> The drone name (`copter ID`) is the `hostname`. It is what you set with the `drone-setup` command, and it is what the table on the server shows.

## How the client finds the server

There is no need to enter the server's IP address in advance:

1. Every 2 seconds the client sends a broadcast UDP request to port `9000`.
2. The server replies with the port for commands (`9010` by default).
3. The client connects to that port over TCP and introduces itself by name.
4. The client then adds the server to `chrony` as a time source (see [time synchronization](#time-synchronization)).

If the connection drops, the client looks for the server again and reconnects by itself. The drone and the server must be on the same network (the same router subnet).

While connected, the client sends telemetry to the server twice a second over UDP to port `9001`: battery, mode, position, check results, the active config and the latest errors.

## What the drone needs to fly

The whole flight happens in the `aruco_map` coordinate frame. So to start an animation the drone must:

* see the ArUco marker map (otherwise it has no position and the client refuses to play the animation);
* be connected to the flight controller;
* have the animation file `animation.csv` loaded.

The marker map and the animation are made in Blender: [Creating an Animation](blender_addon.md).

## Preflight check

The client checks itself every 5 seconds. The **Проверка (Preflight check)** button on the server simply runs the check immediately. The check covers:

| Check | What must be true |
|---|---|
| FCU | `mavros/state` messages arrive and MAVROS sees the flight controller |
| Battery | voltage per cell is not below `battery_min_voltage` (3.5 V by default) |
| ArUco | the drone sees the marker map and knows its position in `aruco_map` |
| ROS services | `navigate`, `land`, `get_telemetry` and `led/set_effect` are available |

If everything is fine but the charge is below `battery_warn_voltage` (3.7 V per cell), a warning (`WARN`) appears. The number of cells is set by `battery_cells` in the [config](extras/config.md).

> If the check fails, the client **refuses** to run the `play` command and prints `REFUSED play` in the server console. You can override this with the **Developer mode** checkbox on the server — use it only deliberately.

## What the client can do

The client runs one task at a time. A new command interrupts the current one, so **Land** and **Pause** work at any moment.

| Command | Action |
|---|---|
| `takeoff` | take off to the given height (the **Z** field on the server) |
| `play` | play the animation, optionally with a delayed start |
| `stop` | hold the current position (the **Pause** button) |
| `land` | land |
| `disarm` | stop the motors |
| `test_leds` | cycle the LED strip through red, green and blue |
| `reboot_fcu` | reboot the flight controller |
| `calibrate_gyro`, `calibrate_level` | calibrate the gyroscope and level |
| `reload_animation` | re-read the animation file |
| `check` | run the check immediately |

The **Flip** button exists on the server, but it is not implemented on this firmware: the client just ignores the command.

### Takeoff

The drone climbs straight up from where it stands and then checks its height in `aruco_map`. If the target height is not reached within `takeoff_time` seconds, the takeoff counts as failed.

### Animation

On `play` the client:

1. checks that the animation is loaded and the drone sees the marker map;
2. chooses how to start — with a takeoff (`takeoff`) or directly in flight (`fly`);
3. waits for the start time and then sends points to the drone and colors to the LED strip frame by frame.

The start time is passed as an absolute time, so all drones start simultaneously if their clocks are synchronized. More details: [Animation Module Description](extras/animation.md).

## Time synchronization

Starting all drones at the same moment depends on clock accuracy. For this, the server acts as a time source (`chrony`) and the drones adjust to it. The `dt` column in the server table shows how many seconds the drone's clock differs from the server's:

* under 0.05 s — good (green);
* 0.05 to 0.2 s — warning (yellow);
* over 0.2 s or no synchronization — error (red).

If `chrony` is unavailable, the client tries to measure the offset directly against the server over NTP.

## Failsafe

While the drone is airborne in `OFFBOARD` mode, the client watches the camera-based position. If the position has not been updated for more than 2 seconds, the client switches the drone to `AUTO.LAND` mode by itself, and the drone lands. On the server, such a drone is highlighted in the `mode` column.

> This only protects against loss of position. It does not replace an attentive operator: read the [safety rules](safety.md).

## Updating

You can update the client with a button on the server (**Selected drones → Update**) or directly on the drone:

```bash
cd /opt/droneswarm/drone    # for the image: /home/pi/DroneSwarm/drone
python3 update.py
```

If the client is installed from git (the image), `git pull` is run; if from the package, `apt-get install --only-upgrade drone-swarm`. After a successful update the service restarts; add `--no-restart` to skip that. The server skips updating an armed drone.

The `version` column on the server shows which version is installed on each drone. If it differs from the server's version, the cell turns yellow.

**More about the client modules: [extras/_drone.md](extras/_drone.md)**
