"""Blender side of ArUco maps: marker objects, generated textures and materials, map backplate."""
import bpy
from mathutils import Euler, Vector

from .aruco_dictionary import marker_grid
from .aruco_map import MapMarker

COLLECTION_NAME = "ArUco map"
BACKPLATE_NAME = "aruco_backplate"
ID_PROP = "aruco_id"
SIZE_PROP = "aruco_size"
CELL_PIXELS = 16
BACKPLATE_GAP = 0.001  # keeps the backplate just below the markers to avoid z-fighting


def map_collection(context):
    collection = bpy.data.collections.get(COLLECTION_NAME)
    if collection is None:
        collection = bpy.data.collections.new(COLLECTION_NAME)
    if collection.name not in context.scene.collection.children:
        context.scene.collection.children.link(collection)
    return collection


def marker_objects(context, selected_only=False):
    objects = context.selected_objects if selected_only else context.scene.objects
    return [o for o in objects if ID_PROP in o]


def _marker_image(marker_id):
    name = f"aruco_{marker_id}"
    image = bpy.data.images.get(name)
    if image is not None:
        return image
    grid = marker_grid(marker_id)
    side = len(grid) * CELL_PIXELS
    pixels = []
    for row in reversed(grid):  # Blender image rows run bottom to top
        line = []
        for cell in row:
            line.extend(((float(cell), float(cell), float(cell), 1.0)) * CELL_PIXELS)
        pixels.extend(line * CELL_PIXELS)
    image = bpy.data.images.new(name, side, side, alpha=False)
    image.pixels.foreach_set(pixels)
    try:
        image.pack()  # generated images are lost on save otherwise
    except RuntimeError:
        pass
    return image


def _new_material(name):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Roughness"].default_value = 1.0
    material.node_tree.links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])
    return material, bsdf


def _marker_material(marker_id):
    name = f"aruco_{marker_id}"
    material = bpy.data.materials.get(name)
    if material is not None:
        return material
    material, bsdf = _new_material(name)
    texture = material.node_tree.nodes.new("ShaderNodeTexImage")
    texture.image = _marker_image(marker_id)
    texture.interpolation = "Closest"  # keep the cells sharp
    material.node_tree.links.new(texture.outputs["Color"], bsdf.inputs["Base Color"])
    return material


def _backplate_material():
    material = bpy.data.materials.get(BACKPLATE_NAME)
    if material is None:
        material, bsdf = _new_material(BACKPLATE_NAME)
        bsdf.inputs["Base Color"].default_value = (1.0, 1.0, 1.0, 1.0)
    return material


def _plane_mesh(name, half_x, half_y, uv=False):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata([(-half_x, -half_y, 0), (half_x, -half_y, 0), (half_x, half_y, 0), (-half_x, half_y, 0)],
                     [], [(0, 1, 2, 3)])
    if uv:
        layer = mesh.uv_layers.new(name="UVMap")
        for loop in mesh.loops:
            co = mesh.vertices[loop.vertex_index].co
            layer.data[loop.index].uv = (1.0 if co.x > 0 else 0.0, 1.0 if co.y > 0 else 0.0)
    mesh.update()
    return mesh


def create_marker(context, marker):
    """Add an object for `marker` (a MapMarker) to the map collection."""
    name = f"aruco_{marker.id}"
    obj = bpy.data.objects.new(name, _plane_mesh(name, marker.size / 2, marker.size / 2, uv=True))
    obj.data.materials.append(_marker_material(marker.id))
    obj.rotation_mode = "XYZ"  # Blender "XYZ" applies x, then y, then z: R = Rz*Ry*Rx, as in ROS rpy
    obj.rotation_euler = (marker.rot_x, marker.rot_y, marker.rot_z)
    obj.location = (marker.x, marker.y, marker.z)
    obj[ID_PROP] = marker.id
    obj[SIZE_PROP] = marker.size
    map_collection(context).objects.link(obj)
    return obj


def marker_from_object(obj):
    """MapMarker with the world pose of a marker object."""
    world = obj.matrix_world
    location = world.to_translation()
    rotation = world.to_euler("XYZ")
    scale = world.to_scale()
    size = float(obj[SIZE_PROP]) * (abs(scale.x) + abs(scale.y)) / 2
    return MapMarker(int(obj[ID_PROP]), size, location.x, location.y, location.z,
                     rotation.z, rotation.y, rotation.x)


def remove_markers(context):
    for obj in list(marker_objects(context)) + [bpy.data.objects.get(BACKPLATE_NAME)]:
        if obj is not None:
            bpy.data.objects.remove(obj, do_unlink=True)


def update_backplate(context, margin=0.1):
    """(Re)create the white sheet under all markers. Returns None when there are no markers."""
    markers = marker_objects(context)
    if not markers:
        return None
    corners = []
    for obj in markers:
        half = float(obj[SIZE_PROP]) / 2
        for cx, cy in ((-half, -half), (half, -half), (half, half), (-half, half)):
            corners.append(obj.matrix_world @ Vector((cx, cy, 0)))
    min_x, max_x = min(c.x for c in corners), max(c.x for c in corners)
    min_y, max_y = min(c.y for c in corners), max(c.y for c in corners)
    z = min(c.z for c in corners) - BACKPLATE_GAP

    old = bpy.data.objects.get(BACKPLATE_NAME)
    if old is not None:
        bpy.data.objects.remove(old, do_unlink=True)
    half_x, half_y = (max_x - min_x) / 2 + margin, (max_y - min_y) / 2 + margin
    obj = bpy.data.objects.new(BACKPLATE_NAME, _plane_mesh(BACKPLATE_NAME, half_x, half_y))
    obj.data.materials.append(_backplate_material())
    obj.location = ((min_x + max_x) / 2, (min_y + max_y) / 2, z)
    map_collection(context).objects.link(obj)
    return obj
