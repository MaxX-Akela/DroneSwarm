bl_info = {
    "name": "DroneSwarm",
    "author": "MaxX-Akela (animation exporter based on clever-show by Artem Vasiunik & Arthur Golubtsov)",
    "version": (0, 6, 0),
    "blender": (2, 80, 0),
    "location": "File > Import/Export, 3D View > Sidebar > DroneSwarm",
    "description": "Export swarm animation (.csv) and build/import/export Clover ArUco maps (.txt)",
    "category": "Import-Export",
}

from . import animation_export, aruco_ops

_modules = (animation_export, aruco_ops)


def register():
    for module in _modules:
        module.register()


def unregister():
    for module in reversed(_modules):
        module.unregister()
