# Blend/Export.py
import json
import bpy
import math
import mathutils
import os

# ---------------- Material extraction ----------------
def extract_material(obj):
    """
    Returns a dict for Whitted shading:
      kd (diffuse), ks (specular), shininess, reflectivity, transmissive, ior,
      and optionally albedo_texture (image path).
    Supports Principled, Diffuse+Glossy(+Mix), Glass/Refraction.
    """
    import bpy, math

    def clamp01(x): return max(0.0, min(1.0, float(x)))
    def set_kd(rgb): return [clamp01(rgb[0]), clamp01(rgb[1]), clamp01(rgb[2])]

    out = {
        "kd": [0.8, 0.8, 0.8],
        "ks": [0.2, 0.2, 0.2],
        "shininess": 32.0,
        "reflectivity": 0.0,
        "transmissive": 0.0,
        "ior": 1.5
    }

    mat = obj.active_material or (obj.material_slots[0].material if obj.material_slots else None)
    if not mat:
        return out

    def average_image_rgb(img):
        try:
            px = img.pixels[:]  # RGBA floats
            stride = max(1, int((img.size[0] * img.size[1]) / 4096))
            r = g = b = n = 0
            for i in range(0, len(px), 4 * stride):
                r += px[i + 0]; g += px[i + 1]; b += px[i + 2]; n += 1
            if n > 0:
                return [r/n, g/n, b/n]
        except Exception:
            pass
        return None

    def read_color_socket(sock):
        # Returns (rgb, texture_path_or_None)
        if not sock:
            return None, None
        if sock.is_linked and sock.links:
            n = sock.links[0].from_node

            # --- NEW: handle MixRGB "Multiply" as tint (Color2) ---
            # e.g. Image Texture -> Color1, Constant Color -> Color2
            # We want Color2 (the tint) to become kd.
            if n.type == 'MIX_RGB' and getattr(n, "blend_type", "") == 'MULTIPLY':
                col2 = n.inputs.get("Color2")
                if col2:
                    # If Color2 is linked to an RGB node
                    if col2.is_linked and col2.links:
                        n2 = col2.links[0].from_node
                        if n2.type == 'RGB':
                            return list(n2.outputs[0].default_value[:3]), None
                    # Fallback: use Color2's own value
                    try:
                        return list(col2.default_value[:3]), None
                    except Exception:
                        pass
                # If we can't read Color2 cleanly, fall back to normal handling below

            if n.type == 'RGB':
                return list(n.outputs[0].default_value[:3]), None
            if n.type == 'TEX_IMAGE' and getattr(n, "image", None):
                img = n.image
                path = bpy.path.abspath(img.filepath)
                avg = average_image_rgb(img)
                return (avg if avg else list(n.outputs[0].default_value[:3])), path
        # unlinked default
        try:
            return list(sock.default_value[:3]), None
        except Exception:
            return None, None

    def roughness_to_shininess(r):
        r = clamp01(r)
        if r < 1e-4:
            return 512.0
        # very common mapping: Blinn exponent ~ 2/r^2 - 2 (clamped)
        return max(2.0, min(512.0, 2.0/(r*r) - 2.0))

    # Default fallbacks if no nodes:
    if not getattr(mat, "use_nodes", False) or not mat.node_tree:
        # Try legacy material props
        try:
            dc = getattr(mat, "diffuse_color", (0.8, 0.8, 0.8, 1.0))
            out["kd"] = set_kd(dc[:3])
        except Exception:
            pass
        spec_int = float(getattr(mat, "specular_intensity", 0.2))
        rough = float(getattr(mat, "roughness", 0.5))
        out["ks"] = [spec_int]*3
        out["shininess"] = roughness_to_shininess(rough)
        out["reflectivity"] = 0.25 * spec_int
        return out

    # Node-based parse
    nt = mat.node_tree
    principled = None
    diffuse = None
    glossy = None
    glass = None
    refr = None
    mix_nodes = []
    fresnel_node = None
    layer_weight = None

    for n in nt.nodes:
        t = n.type
        if t == 'BSDF_PRINCIPLED': principled = n
        elif t == 'BSDF_DIFFUSE': diffuse = n
        elif t == 'BSDF_GLOSSY': glossy = n
        elif t == 'BSDF_GLASS': glass = n
        elif t == 'BSDF_REFRACTION': refr = n
        elif t == 'MIX_SHADER': mix_nodes.append(n)
        elif t == 'FRESNEL': fresnel_node = n
        elif t == 'LAYER_WEIGHT': layer_weight = n

    # 1) PRINCIPLED path (colour + spec + rough + transmission + ior)
    if principled:
        base = principled.inputs.get("Base Color")
        rgb, tex = read_color_socket(base)
        if rgb: out["kd"] = set_kd(rgb)
        if tex: out["albedo_texture"] = tex

        spec = principled.inputs.get("Specular")
        rough = principled.inputs.get("Roughness")
        metal = principled.inputs.get("Metallic")
        trans = principled.inputs.get("Transmission")
        ior_s = principled.inputs.get("IOR")

        s = float(spec.default_value) if spec else 0.2
        r = float(rough.default_value) if rough else 0.5
        m = float(metal.default_value) if metal else 0.0
        tr= float(trans.default_value) if trans else 0.0
        ior= float(ior_s.default_value) if ior_s else 1.45

        # ks: mix specular with metallic tinting
        kd = out["kd"]
        ks_scalar = clamp01(s)
        ks_col = [(1.0 - m) * ks_scalar + m * kd[i] for i in range(3)]
        out["ks"] = [clamp01(x) for x in ks_col]
        out["shininess"] = roughness_to_shininess(r)
        out["reflectivity"] = clamp01(0.5*m + 0.5*ks_scalar)
        out["transmissive"] = clamp01(tr)
        out["ior"] = max(1.0, min(3.0, ior))
        return out

    # 2) DIFFUSE + GLOSSY (+MIX) pipeline
    # kd from Diffuse
    if diffuse:
        base = diffuse.inputs.get("Color")
        rgb, tex = read_color_socket(base)
        if rgb: out["kd"] = set_kd(rgb)
        if tex: out["albedo_texture"] = tex

    # ks/shininess from Glossy
    if glossy:
        gcol = glossy.inputs.get("Color")
        gro  = glossy.inputs.get("Roughness")
        rgb, _ = read_color_socket(gcol)
        if rgb:
            out["ks"] = set_kd(rgb)
        r = float(gro.default_value) if gro else 0.2
        out["shininess"] = roughness_to_shininess(r)

    # reflectivity from Mix Shader factor (if mixing Diffuse & Glossy)
    mix_reflect = None
    for mix in mix_nodes:
        fac = mix.inputs.get("Fac")
        if not fac: continue
        if fac.is_linked and fac.links:
            src = fac.links[0].from_node
            if src.type == 'FRESNEL':
                ior_val = float(getattr(src.inputs.get("IOR"), "default_value", 1.45))
                R0 = ((ior_val - 1.0)/(ior_val + 1.0))**2
                mix_reflect = clamp01(R0)
                out["ior"] = max(out["ior"], ior_val)
                break
            elif src.type == 'LAYER_WEIGHT':
                blend = float(getattr(src.inputs.get("Blend"), "default_value", 0.5))
                mix_reflect = clamp01(blend*0.5)
                break
        else:
            facv = float(getattr(fac, "default_value", 0.0))
            mix_reflect = clamp01(facv)
            break
    if mix_reflect is not None:
        out["reflectivity"] = mix_reflect
    else:
        if glossy:
            out["reflectivity"] = 0.25

    # 3) Glass / Refraction path (transmissive + ior)
    if glass or refr:
        node = glass if glass else refr
        ro = node.inputs.get("Roughness")
        io = node.inputs.get("IOR")
        if io: out["ior"] = float(io.default_value)
        if ro: out["shininess"] = max(out["shininess"], roughness_to_shininess(float(ro.default_value)))
        out["transmissive"] = 1.0  # treat as transparent in a Whitted sense

    return out



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
                "radiant_intensity": float(obj.data.energy/3)
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
