# Animation Module Description

The module `drone/modules/animation.py` reads the animation file, prepares the frames for a specific drone and executes them. The animation file lives on the drone as `drone/animation.csv` and usually arrives from the server: **Selected drones → Send → Animation...** The files themselves are created in Blender: [Creating an Animation](../blender_addon.md).

## File format

```
my_show
1,0.0,0.0,0.0,0.0,255,0,0
2,0.0,0.0,0.0,0.0,255,0,0
3,0.0,0.0,0.1,0.0,255,0,0
...
```

* **The first line** with a single value is the animation name (`animation ID` on the server). If it is missing, the name is `No animation id` and the cell on the server turns yellow.
* **A frame line** has eight fields: `number, x, y, z, yaw, red, green, blue`. Coordinates are in meters, the angle in radians, color 0–255.
* **A line with two values** changes the delay between the following frames. The second value is the time in seconds.

If the file is empty, cannot be opened, has no frames or contains an unreadable line, the animation gets an error state (visible in the tooltip of the `animation ID` column) and cannot be started.

If the `yaw` parameter in the [config](config.md) is not `animation`, the given angle in degrees is used instead of the angle from the file.

## What an animation consists of

The module splits the frames into five sections. The split is based on how the drone moves (the movement threshold is 1 cm between frames):

| Section | What is in it |
|---|---|
| static start | the drone stands still |
| takeoff | the drone moves up without shifting in the plane |
| route | the main part of the show |
| landing | descent at the end, without shifting in the plane |
| static end | the drone stands still again |

Which sections are played is set by the `output_*` parameters in the config. By default takeoff, route and landing are played, while the static start and end are skipped.

## Preparing the frames

When the file is loaded and on every config change, the frames go through this chain:

1. **`transform`** — scale and offset: `x' = ratio · x + common_offset + private_offset`.
2. **`mark_stand_frames`** — static sections are marked `stand` (if the height is below `takeoff_level`) or `fly`.
3. **`apply_flags`** — sections are selected by `output_*`. At the same time a second list of frames is built, for starting "from takeoff".
4. **`mark_flight`** — service frames are inserted:
   * `arm` — before the first flight frame, for `arming_time`, starts the motors;
   * `takeoff` and `reach` — in the second list: takeoff to `takeoff_height` and then flying to the first point (`takeoff_time` and `reach_first_point_time` are allowed);
   * `land` — before the last flight frame, with the `land_delay` delay.

## How the animation starts

The module has two ways to start:

* **`takeoff`** — the drone stands on the ground, takes off by itself to `takeoff_height` and then flies to the first point of the animation;
* **`fly`** — the animation begins right at the first point; this is used when the takeoff is already part of the animation itself.

The `start_action` parameter sets the choice. In `auto` mode `takeoff` is chosen if the first point is above `takeoff_level`, otherwise `fly`.

If `check_ground` is enabled, the module checks before the start that the lowest point of the animation does not go underground. The ground height is the drone's current height (`ground_level = current`) or a given number. If the difference is more than 0.2 m, the client prints an error like `animation is lower than ground level for 0.35m` and does not start.

## Executing the frames

For each frame `execute_frame()` does the following:

| Frame action | What happens |
|---|---|
| `arm` | sends a point with motor auto-start |
| `fly`, `reach` | sends the point `x, y, z, yaw` in the `aruco_map` frame and does not wait for arrival |
| `takeoff` | red LED, takeoff, then green blinking |
| `land` | landing, red blinking, waiting for the motors to stop, then the LED goes off |

For `arm`, `fly` and `reach` frames the LED color from the frame is also set. The time between frames comes from the delay field: each frame runs at the moment `start + sum of previous delays`, rather than "0.1 seconds after the previous one". So the smoothness of the motion does not depend on how long sending a command took.

If a `land`, `stop` or other command arrives, playback is interrupted immediately.
