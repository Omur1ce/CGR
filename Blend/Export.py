import json
import bpy
import mathutils
import os


def export_cameras(scene):
    cameras = []
    for obj in bpy.data.objects:
        if obj.type == 'CAMERA':
            # World-space TRS
            loc = obj.matrix_world.translation
            R3  = obj.matrix_world.to_3x3()

            # Blender camera local axes:
            # forward = -Z, up = +Y, right = +X
            gaze = (R3 @ mathutils.Vector((0, 0, -1))).normalized()
            up   = (R3 @ mathutils.Vector((0, 1,  0))).normalized()

            camera = {
                "name": obj.name,
                "location": [loc.x, loc.y, loc.z],
                "gaze_vector": [gaze.x, gaze.y, gaze.z],
                "up_vector":   [up.x,   up.y,   up.z],
                "focal_length": obj.data.lens,           # mm
                "sensor_width": obj.data.sensor_width,   # mm
                "sensor_height": obj.data.sensor_height, # mm
                "film_resolution": [
                    scene.render.resolution_x,
                    scene.render.resolution_y
                ],
                # optional but helpful for exact matching:
                "pixel_aspect": [
                    scene.render.pixel_aspect_x,
                    scene.render.pixel_aspect_y
                ],
                "resolution_percentage": scene.render.resolution_percentage
            }
            cameras.append(camera)
    return cameras


def export_point_lights():
    point_lights = []
    for obj in bpy.data.objects:
        if obj.type == 'LIGHT' and obj.data.type == 'POINT':
            point_light = {
                'name': obj.name,
                'location': list(obj.location),
                'radiant_intensity': obj.data.energy
            }
            point_lights.append(point_light)
    return point_lights


def export_spheres():
    spheres = []
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and 'Sphere' in obj.data.name:
            sphere = {
                'name': obj.name,
                'location': list(obj.location),
                # export full x, y, z scaling from Blender
                'scale': list(obj.scale)
            }
            spheres.append(sphere)
    return spheres


def export_cubes():
    cubes = []
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and 'Cube' in obj.data.name:
            cube = {
                'name': obj.name,
                'translation': list(obj.location),
                'rotation': list(obj.rotation_euler),  # radians
                # export full x, y, z scaling from Blender
                'scale': list(obj.scale)
            }
            cubes.append(cube)
    return cubes



def export_planes():
    planes = []
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and 'Plane' in obj.data.name:
            mesh = obj.data
            # Get world-space coordinates for all vertices
            world_vertices = [obj.matrix_world @ v.co for v in mesh.vertices]
            corners = []
            # Find first quad face and store its four corners
            for face in mesh.polygons:
                if len(face.vertices) == 4:
                    corners = [list(world_vertices[i]) for i in face.vertices]
                    break
            plane = {
                'name': obj.name,
                'corners': corners
            }
            planes.append(plane)
    return planes


def main():
    scene = bpy.context.scene
    data = {
        'cameras': export_cameras(scene),
        'point_lights': export_point_lights(),
        'spheres': export_spheres(),
        'cubes': export_cubes(),
        'planes': export_planes()
    }

    # Save to JSON file next to current .blend file
    filepath = bpy.path.abspath("//export.json")
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=4)

    print(f"Export complete: {filepath}")


if __name__ == "__main__":
    main()
