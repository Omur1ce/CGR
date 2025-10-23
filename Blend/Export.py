import json
import bpy
import mathutils
import os

def export_cameras(scene):
    cameras = []
    for obj in bpy.data.objects:
        if obj.type == 'CAMERA':
            # Compute gaze direction (camera looks along -Z in local space)
            gaze_vector = obj.matrix_world.to_3x3() @ mathutils.Vector((0, 0, -1))
            camera = {
                'name': obj.name,
                'location': list(obj.location),
                'gaze_vector': list(gaze_vector),
                'focal_length': obj.data.lens,
                'sensor_width': obj.data.sensor_width,
                'sensor_height': obj.data.sensor_height,
                'film_resolution': [
                    scene.render.resolution_x,
                    scene.render.resolution_y
                ]
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
                'radius': obj.scale[0]  # assumes uniform scaling
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
                'scale': obj.scale[0]  # assumes uniform scaling
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
