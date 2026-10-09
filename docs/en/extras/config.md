# Configuration

The drone and the server each have their own settings files in `.ini` format. If the file is missing or lacks some parameters, the defaults are used. Unknown keys and values of the wrong format are written to the log and ignored, and the default stays.

Ready-made examples are in the `examples/configs/` folder.

## Drone configuration

The file `drone/drone.ini` (next to `client.py`). The template is `drone/config.example.ini`. Changes apply after the client restarts, or immediately if you send the config from the server: **Selected drones → Send → Configuration...** There are two modes:

* **Modify** — update only the keys present in the sent file and keep the rest;
* **Rewrite** — replace the file completely.

> If the drone is airborne, it applies the new animation settings only on the next animation reload.

The drone's active settings are visible on the server: double-click a cell in the table.

### [animation]

| Parameter | Default | Description |
|---|---|---|
| `frame_delay` | `0.1` | time between frames in seconds, unless the animation file says otherwise |
| `yaw` | `animation` | `animation` — take the angle from the file; a number — a fixed angle in degrees |
| `common_offset` | `0, 0, 0` | common animation offset along X, Y, Z in meters (for all drones) |
| `private_offset` | `0, 0, 0` | offset for this drone only |
| `ratio` | `1, 1, 1` | scale per axis; applied before the offset |
| `takeoff_level` | `0.3` | height (m) below which the drone is considered to be on the ground |
| `output_static_begin` | `false` | play the initial section where the drone stands still |
| `output_takeoff` | `true` | play the takeoff section |
| `output_route` | `true` | play the main route |
| `output_land` | `true` | play the landing section |
| `output_static_end` | `false` | play the final section where the drone stands still |
| `check_ground` | `true` | check that the animation does not go below the ground |
| `ground_level` | `current` | ground height: `current` — the drone's current height, or a number in meters |
| `start_action` | `auto` | how to start: `takeoff` — with a takeoff, `fly` — directly in flight, `auto` — decide by the first point |

Values with coordinates are written separated by commas. Boolean values: `true`/`false`, `yes`/`no`, `on`/`off`, `1`/`0`.

Frame coordinates are recalculated as `x' = ratio_x · x + common_offset_x + private_offset_x`, and the same for each axis. This lets you shift the whole show or a single drone without recreating the animation.

### [led]

| Parameter | Default | Description |
|---|---|---|
| `use` | `true` | enable LED strip control |
| `takeoff_indication` | `true` | red indication at takeoff and green blinking afterwards |
| `land_indication` | `true` | red blinking at landing |

### [flight]

| Parameter | Default | Description |
|---|---|---|
| `takeoff_height` | `1.5` | takeoff height in the animation, m |
| `takeoff_time` | `5.0` | time allowed for takeoff, s |
| `land_timeout` | `5.0` | time allowed for landing, s |
| `land_delay` | `0.0` | delay before the landing command at the end of the animation |
| `arming_time` | `5.0` | time allowed for motor start before the flight begins |
| `reach_first_point_time` | `5.0` | time given to the drone to reach the first point after takeoff |

### [checks]

Voltages are given **per cell**: the client divides the battery voltage by `battery_cells`.

| Parameter | Default | Description |
|---|---|---|
| `battery_cells` | `4` | number of battery cells |
| `battery_min_voltage` | `3.5` | below this the check fails, the column is red |
| `battery_warn_voltage` | `3.7` | below this — a warning, the column is yellow |
| `fcu_timeout` | `3.0` | how long to wait for a message from the flight controller, s |
| `service_timeout` | `2.0` | how long to wait for each ROS service, s |

Example for a 6S battery:

```ini
[checks]
battery_cells = 6
```

## Server configuration

The file `server/config/server.ini`. The template is `server/config/server.example.ini`. You can edit it from the **Server → Edit server config** menu; changes apply after the server restarts (**Server → Restart server**).

### [network]

| Parameter | Default | Description |
|---|---|---|
| `discovery_port` | `9000` | UDP port on which the server answers discovery requests |
| `telemetry_port` | `9001` | UDP port for receiving telemetry |
| `tcp_port` | `9010` | TCP port for commands |
| `bind_address` | `0.0.0.0` | address to listen on; `0.0.0.0` — on all interfaces |
| `drone_timeout` | `5.0` | after how many seconds without telemetry a drone is considered disconnected |

> The discovery port (`9000`) is hard-coded in the drone client, and the client computes the telemetry port as the discovery port plus one. So it is better not to change `discovery_port` and `telemetry_port`. You can change `tcp_port`: the server tells the drones its value during discovery.

### [server]

| Parameter | Default | Description |
|---|---|---|
| `log_dir` | `.` | folder for logs |

### [paths]

Where files sent from the **Send** menu are placed on the drone.

| Parameter | Default |
|---|---|
| `camera_calibration` | `/home/pi/catkin_ws/src/clover/clover/camera_info/fisheye_cam_320x240.yaml` |
| `aruco_map` | `/home/pi/catkin_ws/src/clover/aruco_pose/map/map.txt` |
| `launch_dir` | `/home/pi/catkin_ws/src/clover/clover/launch` |
