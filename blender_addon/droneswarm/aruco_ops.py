"""Operators and panel for building, importing and exporting Clover ArUco maps."""
import bpy
from bpy.props import BoolProperty, FloatProperty, IntProperty, StringProperty
from bpy.types import Operator, Panel
from bpy_extras.io_utils import ExportHelper, ImportHelper

from . import aruco_blender as ab
from .aruco_dictionary import MAX_ID
from .aruco_map import MapMarker, MapParseError, format_map, grid_map, parse_map


def _create_all(context, markers, backplate, margin):
    for marker in markers:
        ab.create_marker(context, marker)
    if backplate:
        ab.update_backplate(context, margin)


class ARUCO_OT_import_map(Operator, ImportHelper):
    bl_idname = "droneswarm.aruco_import_map"
    bl_label = "Import ArUco map"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".txt"
    filter_glob: StringProperty(default="*.txt", options={"HIDDEN"})
    replace: BoolProperty(name="Replace current map", default=True)
    backplate: BoolProperty(name="Create backplate", default=True)
    margin: FloatProperty(name="Backplate margin", unit="LENGTH", default=0.1, min=0)

    def execute(self, context):
        try:
            with open(self.filepath, "r", encoding="utf-8") as f:
                markers = parse_map(f.read())
        except (OSError, UnicodeDecodeError, MapParseError) as e:
            self.report({"ERROR"}, f"Cannot import map: {e}")
            return {"CANCELLED"}
        too_big = [m.id for m in markers if m.id > MAX_ID]
        if too_big:
            self.report({"ERROR"}, f"Marker ids above {MAX_ID} are not in the ArUco dictionary: {too_big[:5]}")
            return {"CANCELLED"}
        if self.replace:
            ab.remove_markers(context)
        _create_all(context, markers, self.backplate, self.margin)
        self.report({"INFO"}, f"Imported {len(markers)} markers")
        return {"FINISHED"}


class ARUCO_OT_export_map(Operator, ExportHelper):
    bl_idname = "droneswarm.aruco_export_map"
    bl_label = "Export ArUco map"

    filename_ext = ".txt"
    filter_glob: StringProperty(default="*.txt", options={"HIDDEN"})
    selected_only: BoolProperty(name="Selected only", default=False)

    def execute(self, context):
        objects = ab.marker_objects(context, self.selected_only)
        if not objects:
            self.report({"ERROR"}, "No ArUco markers to export")
            return {"CANCELLED"}
        markers = sorted((ab.marker_from_object(o) for o in objects), key=lambda m: m.id)
        try:
            with open(self.filepath, "w", encoding="utf-8", newline="\n") as f:
                f.write(format_map(markers))
        except OSError as e:
            self.report({"ERROR"}, f"Cannot write map: {e}")
            return {"CANCELLED"}
        self.report({"INFO"}, f"Exported {len(markers)} markers")
        return {"FINISHED"}


class ARUCO_OT_add_marker(Operator):
    bl_idname = "droneswarm.aruco_add_marker"
    bl_label = "Add ArUco marker"
    bl_options = {"REGISTER", "UNDO"}

    marker_id: IntProperty(name="ID", default=0, min=0, max=MAX_ID)
    size: FloatProperty(name="Size", unit="LENGTH", default=0.33, min=0.001)

    def execute(self, context):
        x, y, z = context.scene.cursor.location
        obj = ab.create_marker(context, MapMarker(self.marker_id, self.size, x, y, z))
        for other in context.selected_objects:
            other.select_set(False)
        obj.select_set(True)
        context.view_layer.objects.active = obj
        return {"FINISHED"}


class ARUCO_OT_generate_grid(Operator):
    bl_idname = "droneswarm.aruco_generate_grid"
    bl_label = "Generate ArUco grid map"
    bl_options = {"REGISTER", "UNDO"}

    size: FloatProperty(name="Marker size", unit="LENGTH", default=0.33, min=0.001)
    count_x: IntProperty(name="Markers along X", default=2, min=1)
    count_y: IntProperty(name="Markers along Y", default=4, min=1)
    dist_x: FloatProperty(name="Distance X", unit="LENGTH", default=1.0, min=0)
    dist_y: FloatProperty(name="Distance Y", unit="LENGTH", default=1.0, min=0)
    first_id: IntProperty(name="First ID", default=0, min=0, max=MAX_ID)
    bottom_left: BoolProperty(name="Start at bottom left", default=False)
    replace: BoolProperty(name="Replace current map", default=True)
    backplate: BoolProperty(name="Create backplate", default=True)
    margin: FloatProperty(name="Backplate margin", unit="LENGTH", default=0.1, min=0)

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        last_id = self.first_id + self.count_x * self.count_y - 1
        if last_id > MAX_ID:
            self.report({"ERROR"}, f"Last marker id {last_id} is above {MAX_ID}")
            return {"CANCELLED"}
        markers = grid_map(self.size, self.count_x, self.count_y, self.dist_x, self.dist_y,
                           self.first_id, self.bottom_left)
        if self.replace:
            ab.remove_markers(context)
        _create_all(context, markers, self.backplate, self.margin)
        return {"FINISHED"}


class ARUCO_OT_update_backplate(Operator):
    bl_idname = "droneswarm.aruco_update_backplate"
    bl_label = "Update backplate"
    bl_options = {"REGISTER", "UNDO"}

    margin: FloatProperty(name="Margin", unit="LENGTH", default=0.1, min=0)

    def execute(self, context):
        if ab.update_backplate(context, self.margin) is None:
            self.report({"ERROR"}, "No ArUco markers in the scene")
            return {"CANCELLED"}
        return {"FINISHED"}


class ARUCO_PT_panel(Panel):
    bl_idname = "ARUCO_PT_panel"
    bl_label = "ArUco map"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "DroneSwarm"

    def draw(self, context):
        col = self.layout.column(align=True)
        col.operator(ARUCO_OT_import_map.bl_idname, text="Import map (.txt)", icon="IMPORT")
        col.operator(ARUCO_OT_export_map.bl_idname, text="Export map (.txt)", icon="EXPORT")
        col = self.layout.column(align=True)
        col.operator(ARUCO_OT_generate_grid.bl_idname, text="Generate grid", icon="MESH_GRID")
        col.operator(ARUCO_OT_add_marker.bl_idname, text="Add marker", icon="ADD")
        col.operator(ARUCO_OT_update_backplate.bl_idname, text="Update backplate", icon="MESH_PLANE")


def _menu_import(self, context):
    self.layout.operator(ARUCO_OT_import_map.bl_idname, text="ArUco map (.txt)")


def _menu_export(self, context):
    self.layout.operator(ARUCO_OT_export_map.bl_idname, text="ArUco map (.txt)")


CLASSES = (ARUCO_OT_import_map, ARUCO_OT_export_map, ARUCO_OT_add_marker, ARUCO_OT_generate_grid,
           ARUCO_OT_update_backplate, ARUCO_PT_panel)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.TOPBAR_MT_file_import.append(_menu_import)
    bpy.types.TOPBAR_MT_file_export.append(_menu_export)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(_menu_export)
    bpy.types.TOPBAR_MT_file_import.remove(_menu_import)
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
