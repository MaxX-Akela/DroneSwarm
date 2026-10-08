"""File > Export > Swarm animation (.csv): one csv per drone object, one row per frame."""
import csv
import math
import os

import bpy
from bpy.props import BoolProperty, FloatProperty, StringProperty
from bpy.types import Operator
from bpy_extras.io_utils import ExportHelper

FRAME_TIME = 0.1  # seconds between frames, as the drone client plays them


def distance(a, b):
    return math.dist(a, b)


def speed(a, b):
    return distance(a, b) / FRAME_TIME


def led_color(obj):
    """[r, g, b] (0..255) from the object's material slot whose name contains 'led_color'."""
    rgb = [0, 0, 0]
    for slot in obj.material_slots:
        if "led_color" not in slot.name.lower() or slot.material is None:
            continue
        material = slot.material
        if material.use_nodes:
            for node in material.node_tree.nodes:
                if node.type in ("EMISSION", "BSDF_DIFFUSE", "BSDF_PRINCIPLED"):
                    color = node.inputs[0].default_value
                    rgb = [int(color[i] * color[3] * 255) for i in range(3)]
        else:
            rgb = [int(material.diffuse_color[i] * 255) for i in range(3)]
        break
    return rgb


class ExportCsv(Operator, ExportHelper):
    bl_idname = "export_animation.folder"
    bl_label = "Export Swarm animation"
    filename_ext = ""
    use_filter_folder = True

    use_namefilter: BoolProperty(name="Use name filter for objects", default=False)
    drones_name: StringProperty(
        name="Name identifier",
        description="Name identifier for all drone objects",
        default="clever",
    )
    show_warnings: BoolProperty(name="Show detailed animation warnings", default=False)
    speed_warning_limit: FloatProperty(
        name="Speed limit", description="Limit of drone movement speed (m/s)",
        unit="VELOCITY", default=3, min=0,
    )
    drone_distance_limit: FloatProperty(
        name="Distance limit", description="Closest possible distance between drones (m)",
        unit="LENGTH", default=1.5, min=0,
    )
    filepath: StringProperty(
        name="File Path", description="Folder the csv files are written to",
        maxlen=1024, subtype="DIR_PATH", default="",
    )

    def _drones(self, context):
        objects = context.visible_objects
        if not self.use_namefilter:
            return list(objects)
        name = self.drones_name.lower()
        return [o for o in objects if name in o.name.lower()]

    def _warn(self, message, detailed=False):
        if not detailed or self.show_warnings:
            self.report({"WARNING"}, message)

    def _check_frame(self, drone, others, position, previous, frame):
        """Returns (speed_exceeded, distance_exceeded) for one frame."""
        speed_exceeded = distance_exceeded = False
        if previous is not None:
            v = speed(position, previous)
            if v > self.speed_warning_limit:
                speed_exceeded = True
                self._warn(f"Speed of drone '{drone.name}' is greater than "
                           f"{round(self.speed_warning_limit, 5)} m/s ({round(v, 5)} m/s) on frame {frame}", True)
        for other in others:
            if other is drone:
                continue
            d = distance(position, other.matrix_world.to_translation())
            if d < self.drone_distance_limit:
                distance_exceeded = True
                self._warn(f"Distance between drones '{drone.name}' and '{other.name}' is less than "
                           f"{round(self.drone_distance_limit, 5)} m ({round(d, 5)} m) on frame {frame}", True)
        return speed_exceeded, distance_exceeded

    def _export_drone(self, context, drone, drones):
        scene = context.scene
        speed_exceeded = distance_exceeded = False
        previous = None
        path = os.path.join(self.filepath, f"{drone.name.lower()}.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL)
            writer.writerow([os.path.splitext(bpy.path.basename(bpy.data.filepath))[0]])
            for frame in range(scene.frame_start, scene.frame_end + 1):
                scene.frame_set(frame)
                x, y, z = drone.matrix_world.to_translation()
                rot_z = drone.matrix_world.to_euler("XYZ")[2]
                s, d = self._check_frame(drone, drones, (x, y, z), previous, frame)
                speed_exceeded |= s
                distance_exceeded |= d
                previous = (x, y, z)
                writer.writerow([str(frame), round(x, 5), round(y, 5), round(z, 5), round(rot_z, 5),
                                 *led_color(drone)])
        if speed_exceeded:
            self._warn(f"Drone '{drone.name}' speed limits exceeded")
        if distance_exceeded:
            self._warn(f"Drone '{drone.name}' distance limits exceeded")
        self.report({"INFO"}, f"Animation file exported for drone '{drone.name}'")

    def execute(self, context):
        os.makedirs(self.filepath, exist_ok=True)
        drones = self._drones(context)
        for drone in drones:
            self._export_drone(context, drone, drones)
        return {"FINISHED"}


def _menu(self, context):
    self.layout.operator(ExportCsv.bl_idname, text="Swarm animation (.csv)")


def register():
    bpy.utils.register_class(ExportCsv)
    bpy.types.TOPBAR_MT_file_export.append(_menu)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(_menu)
    bpy.utils.unregister_class(ExportCsv)
