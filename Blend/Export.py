# Blend/Export.py
import json
import bpy
import math
import mathutils
import os

# ---------------- Material extraction ----------------
def extract_material(obj):
    """
    Return a dict with Blinn-Phong-friendly fields derived from Blender materials:
    kd (diffuse rgb), ks (specular rgb), shininess, reflectivity, transmissive, ior.
    If no material, returns sensible defaults.
    """
    # defaults
    mat_out = {
        "kd": [0.8, 0.8, 0.8],
        "ks": [0.2, 0.2, 0.2],
        "shininess": 32.0,
        "reflectivity": 0.0,
        "transmissive": 0.0,
        "ior": 1.5
    }

    mat = obj.active_material or (obj.material_slots[0].material if obj.material_slots else None)
    if mat is None:
        return mat_out

    # Helper: clamp
    def C(x, lo=0.0, hi=1.0): return max(lo, min(hi, float(x)))

    # Prefer Principled BSDF
    if getattr(mat, "use_nodes", False) and mat.node_tree:
        bsdf = None
        for n in mat.node_tree.nodes:
            if n.type == 'BSDF_PRINCIPLED':
                bsdf = n
                break
        if bsdf:
            base_col = list(bsdf.inputs["Base Color"].default_value)[:3]
            spec      = float(bsdf.inputs["Specular"].default_value)
            rough     = float(bsdf.inputs["Roughness"].default_value)
            metal     = float(bsdf.inputs["Metallic"].default_value)
            transmit  = float(bsdf.inputs.get("Transmission", bsdf.inputs.get("Transmission Weight", None)).default_value
                              if ("Transmission" in bsdf.inputs or "Transmission Weight" in bsdf.inputs) else 0.0)
            ior       = float(bsdf.inputs.get("IOR", None).default_value if "IOR" in bsdf.inputs else 1.45)

            # Map to Blinn-Phong-ish:
            kd = [C(c) for c in base_col]
            # specular colour: mix white with base for metallic looks
            ks_scalar = C(spec)
            ks_col = [(1.0 - metal) * ks_scalar + metal * kd[i] for i in range(3)]

            # Approximate Phong shininess from roughness (0..1). Common heuristic:
            # shininess ≈ 2 / (r^2) - 2, clamp to [2, 512]
            r = C(rough)
            shininess = 32.0
            if r > 1e-4:
                shininess = max(2.0, min(512.0, 2.0 / (r * r) - 2.0))

            # Reflectivity heuristic: combine metallic + specular
            reflectivity = C(0.5 * metal + 0.5 * ks_scalar)

            # Transmission and IOR
            transmissive = C(transmit)
            ior = max(1.0, min(3.0, ior))

            mat_out.update({
                "kd": kd,
                "ks": ks_col,
                "shininess": float(shininess),
                "reflectivity": float(reflectivity),
                "transmissive": float(transmissive),
                "ior": float(ior)
            })
            return mat_out

    # Fallback: non-node material props
    # Note: in newer Blender, diffuse/specular might be legacy; use best-guess mapping
    try:
        # diffuse_color is RGBA; use RGB
        kd = list(getattr(mat, "diffuse_color", (0.8, 0.8, 0.8, 1.0)))[:3]
        spec_int = float(getattr(mat, "specular_intensity", 0.2))
        rough = float(getattr(mat, "roughness", 0.5))
        # simple mappings
        ks = [spec_int]*3
        r = C(rough)
        shininess = max(2.0, min(512.0, 2.0 / (r * r + 1e-6) - 2.0))
        mat_out.update({
            "kd": [C(c) for c in kd],
            "ks": [C(x) for x in ks],
            "shininess": float(shininess),
            "reflectivity": float(0.25 * spec_int),
            "transmissive": 0.0,
            "ior": 1.5
        })
    except Exception:
        pass

    return mat_out

# ---------------- Cameras ----------------
def export_cameras(scene):
    cameras = []
    for obj in bpy.data.objects:
        if obj.type == 'CAMERA':
            # World-space TRS
            loc = obj.matrix_world.translation
            R3  = obj.matrix_world.to_3x3()

            # Blender camera local axes: forward = -Z, up = +Y
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
                "pixel_aspect": [
                    scene.render.pixel_aspect_x,
                    scene.render.pixel_aspect_y
                ],
                "resolution_percentage": scene.render.resolution_percentage
            }
            cameras.append(camera)
    return cameras

# ---------------- Lights ----------------
def export_point_lights():
    point_lights = []
    for obj in bpy.data.objects:
        if obj.type == 'LIGHT' and obj.data.type == 'POINT':
            point_light = {
                "name": obj.name,
                "location": list(obj.location),
                "radiant_intensity": float(obj.data.energy)
            }
            point_lights.append(point_light)
    return point_lights

# ---------------- Mesh categories ----------------
def export_spheres():
    spheres = []
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and 'Sphere' in obj.data.name:
            sphere = {
                "name": obj.name,
                "location": list(obj.location),
                "scale": list(obj.scale),
                "material": extract_material(obj)
            }
            spheres.append(sphere)
    return spheres

def export_cubes():
    cubes = []
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and 'Cube' in obj.data.name:
            cube = {
                "name": obj.name,
                "translation": list(obj.location),
                "rotation": list(obj.rotation_euler),  # radians
                "scale": list(obj.scale),
                "material": extract_material(obj)
            }
            cubes.append(cube)
    return cubes

def export_planes():
    planes = []
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and 'Plane' in obj.data.name:
            mesh = obj.data
            world_vertices = [obj.matrix_world @ v.co for v in mesh.vertices]
            corners = []
            for face in mesh.polygons:
                if len(face.vertices) == 4:
                    corners = [list(world_vertices[i]) for i in face.vertices]
                    break
            plane = {
                "name": obj.name,
                "corners": corners,
                "material": extract_material(obj)
            }
            planes.append(plane)
    return planes

# ---------------- Entry ----------------
def main():
    scene = bpy.context.scene
    data = {
        "cameras":      export_cameras(scene),
        "point_lights": export_point_lights(),
        "spheres":      export_spheres(),
        "cubes":        export_cubes(),
        "planes":       export_planes()
    }

    # Save next to the .blend
    filepath = bpy.path.abspath("//export.json")
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=4)

    print(f"Export complete: {filepath}")

if __name__ == "__main__":
    main()
