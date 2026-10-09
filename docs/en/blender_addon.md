# Creating an Animation

Show animations are made in [Blender](https://www.blender.org). The DroneSwarm add-on is used for this. It does two things:

* exports the movement of drone objects to `.csv` files that the client understands;
* builds, imports and exports an ArUco marker map for Clover.

## Installing the add-on

1. Take the `droneswarm.zip` file from the `blender_addon` folder (or zip the `droneswarm` folder yourself — it must be at the root of the archive).
2. In Blender open **Edit → Preferences → Add-ons → Install**, select the archive and enable the **DroneSwarm** add-on.

The add-on supports Blender 2.80 and newer.

## Drone animation

### Preparing the scene

1. Each drone is a separate object (for example, a cube or an empty). The object's name becomes the file name and must match the drone's `hostname`. For example, object `clover-1` → file `clover-1.csv`.
2. Move the objects and set keyframes as usual. Blender units are meters. The scene origin matches the origin of the ArUco map.
3. To set the LED color, add a material to the object and name its slot so that the name contains `led_color`. The color is taken from an `Emission`, `Diffuse` or `Principled BSDF` node. The color can be animated just like the position. Without such a material the LED strip will be black.
4. The animation range is set by the usual **Start** and **End** on the timeline.

> The client plays frames at 0.1-second steps (10 frames per second). Set Blender to 10 fps, otherwise the motion will run faster or slower than intended.

### Export

**File → Export → Swarm animation (.csv)**, choose a folder. A file `<object name>.csv` is created for every drone.

Export parameters:

| Parameter | Meaning |
|---|---|
| Use name filter for objects | export only objects whose name contains the given substring. Otherwise **all visible** objects in the scene are exported |
| Name identifier | that substring (`clever` by default) |
| Speed limit | warn if the speed is higher (3 m/s by default) |
| Distance limit | warn if drones are closer to each other (1.5 m by default) |
| Show detailed animation warnings | print a warning for every frame, not just the summary |

Speed and distance are checked only as warnings: the file is created anyway. See them in the Blender console (**Window → Toggle System Console**) or in the status bar.

> If the scene has a camera, a light or a marker map, enable the name filter, otherwise files are created for them too.

### What is inside the file

The first line is the animation name (the name of the `.blend` file). It is shown on the server in the `animation ID` column. Then one line per frame:

```
frame_number,x,y,z,yaw,red,green,blue
```

Coordinates are in meters, `yaw` in radians, color from 0 to 255. More about how the client reads the file: [Animation Module Description](extras/animation.md).

## ArUco map

The drone determines its position from the marker map on the floor. The map file is a plain text Clover `map.txt`. The add-on lets you draw it in the same scene where you make the animation, so you can see right away where the markers are relative to the drones.

The panel is in **3D View → Sidebar (N key) → DroneSwarm**. The same commands are in **File → Import / Export → ArUco map (.txt)**.

| Button | What it does |
|---|---|
| Import map | load `map.txt`. You can replace the current map and create a backplate |
| Export map | save the scene's markers to `map.txt` (optionally only the selected ones) |
| Generate grid | create a grid of markers |
| Add marker | add one marker at the 3D cursor position |
| Update backplate | recreate the white backplate under the markers |

### Generating a grid

Parameters:

* marker size (0.33 m by default);
* number of markers along X and along Y (2 × 4);
* distance between centers along X and Y (1 m);
* number of the first marker;
* "Start at bottom left" — number from the bottom-left corner (top-left by default);
* "Replace current map" — delete the previous markers;
* "Create backplate" and the backplate margin.

Numbers go in rows along the X axis. The center of the first column and the bottom row is at `(0, 0)`.

### map.txt format

```
# id    length  x    y    z    rot_z  rot_y  rot_x
0       0.33    0    3    0    0      0      0
```

Each line is one marker: number, side length, center position, rotation angles in radians. Blank lines and comments after `#` are skipped. A line may contain just the first two numbers (`id size`): the rest become zeros.

The add-on uses the **ORIGINAL 5×5** ArUco dictionary, ids 0 to 1023 — this is Clover's default dictionary. The marker images and materials are created automatically. All markers are in the `ArUco map` collection, and the backplate is named `aruco_backplate`.

### How to send the map to the drones

Export `map.txt` and send it from the server: **Selected drones → Send → Aruco map...** After the upload, Clover restarts on the drones.

## Typical workflow

1. Generate or import the marker map.
2. Place the drone objects and make the animation.
3. Export the animation and check the speed and distance warnings.
4. Export the marker map.
5. Send the map and the animations to the drones through the server: [Working with the Server](server.md).
