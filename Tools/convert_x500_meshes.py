#!/usr/bin/env python3
"""Bake the MRS Gazebo x500 meshes into OBJ models O3DE can import.

    python3 Tools/convert_x500_meshes.py [path to mrs_uav_gazebo_simulation]

O3DE imports neither .dae nor colored .stl, so the parts of models/.../sdf/drones/x500.sdf.jinja
are placed at their SDF poses and merged by color into Assets/Drone/x500/: one body model, and
one model per prop direction with its origin at the rotor hub. Only needs the Python stdlib.
"""

import math
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
OUTPUT = PROJECT_ROOT / "Assets" / "Drone" / "x500"
MESHES = Path("models/mrs_robots_description/meshes/x500v2")

QUARTER = math.pi / 2
HALF = math.pi

# Diffuse colors of the Gazebo materials; "map" materials use the texture from the DAE files.
MATERIALS = {
    "carbon": {"map": "carbon.jpg"},
    "carbon_light": {"map": "carbon_light.jpg"},
    "blue": {"color": (0.03, 0.74, 1.0)},
    "black": {"color": (0.02, 0.02, 0.02)},
    "red": {"color": (0.8, 0.02, 0.02)},
    "grey": {"color": (0.45, 0.45, 0.45)},
    "dark_grey": {"color": (0.15, 0.15, 0.15)},
    "green": {"color": (0.1, 0.45, 0.12)},
}

# (mesh, material, yaw [rad], z offset [m])
BODY_PARTS = [
    ("top_board_mesh_file.dae", "carbon", QUARTER, 0.0),
    ("bottom_board_mesh_file.dae", "carbon", QUARTER, 0.0),
    ("nuc_board_mesh_file.stl", "green", QUARTER, 0.0),
    ("nuc_cooler_mesh_file.stl", "grey", QUARTER, 0.0),
    ("nuc_hex_posts_mesh_file.stl", "grey", QUARTER, 0.0),
    ("pixhawk6c_mesh_file.stl", "dark_grey", QUARTER, 0.0),
    ("battery_mount_mesh_file.stl", "red", QUARTER, 0.001),
    ("battery_mesh_file.stl", "grey", QUARTER, 0.001),
    ("gps_mount_mesh_file.stl", "grey", QUARTER, 0.0),
    ("gps_module_mesh_file.stl", "black", QUARTER, 0.0),
]
for corner in range(4):
    yaw = corner * QUARTER
    print_material = "black" if corner < 2 else "red"  # Front and back differ, as in Gazebo.
    BODY_PARTS += [
        ("arm_carbon_mesh_file.dae", "carbon", yaw, 0.0),
        ("arm_plastic1_mesh_file.stl", "blue", yaw, 0.0),
        ("arm_plastic2_mesh_file.stl", "blue", yaw, 0.0),
        ("arm_3d_print_mesh_file.stl", print_material, yaw, 0.0),
        ("leg_3d_print_mesh_file.stl", print_material, yaw, 0.0),
        ("motor_bottom_mesh_file.stl", "grey", yaw, 0.0),
        ("motor_middle_mesh_file.stl", "black", yaw, 0.0),
    ]

PROP_PARTS = [
    ("prop_mesh_file.dae", "carbon_light", 0.0, 0.0),
    ("motor_top_mesh_file.stl", "grey", 0.0, -0.0086),
]


def load_stl(path):
    """Return triangles as lists of three (position, normal, uv)."""
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    triangles = []
    for i in range(count):
        values = struct.unpack_from("<12f", data, 84 + 50 * i)
        normal = values[:3]
        corners = [values[3:6], values[6:9], values[9:12]]
        triangles.append([(corner, normal, (0.0, 0.0)) for corner in corners])
    return triangles


def load_dae(path):
    """Return triangles of the visual scene node that instantiates the geometry."""
    root = ET.parse(path).getroot()
    ns = {"c": root.tag.split("}")[0].lstrip("{")}

    def floats(element):
        return [float(value) for value in element.text.split()]

    mesh = root.find(".//c:geometry/c:mesh", ns)
    sources = {}
    for source in mesh.findall("c:source", ns):
        array = floats(source.find("c:float_array", ns))
        stride = int(source.find(".//c:accessor", ns).get("stride"))
        sources[source.get("id")] = [array[i : i + stride] for i in range(0, len(array), stride)]
    vertices = mesh.find("c:vertices", ns)
    position_id = vertices.find("c:input[@semantic='POSITION']", ns).get("source").lstrip("#")

    node = next(n for n in root.iter(f"{{{ns['c']}}}node") if n.find("c:instance_geometry", ns) is not None)
    m = floats(node.find("c:matrix", ns))

    def to_scene(p, w):
        return tuple(m[r] * p[0] + m[r + 1] * p[1] + m[r + 2] * p[2] + m[r + 3] * w for r in (0, 4, 8))

    triangles = []
    for primitive in mesh.findall("c:triangles", ns):
        inputs = {i.get("semantic"): i for i in primitive.findall("c:input", ns)}
        stride = max(int(i.get("offset")) for i in inputs.values()) + 1
        indices = [int(value) for value in primitive.find("c:p", ns).text.split()]
        normals = sources[inputs["NORMAL"].get("source").lstrip("#")]
        uvs = sources[inputs["TEXCOORD"].get("source").lstrip("#")]
        corners = []
        for i in range(0, len(indices), stride):
            p = sources[position_id][indices[i + int(inputs["VERTEX"].get("offset"))]]
            n = normals[indices[i + int(inputs["NORMAL"].get("offset"))]]
            uv = uvs[indices[i + int(inputs["TEXCOORD"].get("offset"))]]
            corners.append((to_scene(p, 1.0), to_scene(n, 0.0), tuple(uv[:2])))
        triangles += [corners[i : i + 3] for i in range(0, len(corners), 3)]
    return triangles


def place(triangles, yaw, z_offset, mirror_x):
    """Apply the SDF pose: scale -1 1 1 first, then yaw and height."""
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    sign = -1.0 if mirror_x else 1.0

    def rotate(v, z=0.0):
        x = sign * v[0]
        return (cos_yaw * x - sin_yaw * v[1], sin_yaw * x + cos_yaw * v[1], v[2] + z)

    placed = []
    for triangle in triangles:
        corners = [(rotate(p, z_offset), rotate(n), uv) for p, n, uv in triangle]
        placed.append(corners[::-1] if mirror_x else corners)  # Mirroring flips the winding.
    return placed


def write_model(name, parts, source, mirror_x=False):
    groups = {}
    for mesh, material, yaw, z_offset in parts:
        path = source / mesh
        triangles = load_dae(path) if path.suffix == ".dae" else load_stl(path)
        groups.setdefault(material, []).extend(place(triangles, yaw, z_offset, mirror_x))

    lines = [f"mtllib {name}.mtl"]
    index = 1
    for material, triangles in groups.items():
        lines += [f"o {material}", f"usemtl {material}"]
        for triangle in triangles:
            for p, n, uv in triangle:
                lines.append("v {:.6f} {:.6f} {:.6f}".format(*p))
                lines.append("vn {:.5f} {:.5f} {:.5f}".format(*n))
                lines.append("vt {:.5f} {:.5f}".format(*uv))
            lines.append(f"f {index}/{index}/{index} {index + 1}/{index + 1}/{index + 1} {index + 2}/{index + 2}/{index + 2}")
            index += 3
    (OUTPUT / f"{name}.obj").write_text("\n".join(lines) + "\n")

    mtl = []
    for material in groups:
        properties = MATERIALS[material]
        mtl.append(f"newmtl {material}")
        mtl.append("Kd {:.3f} {:.3f} {:.3f}".format(*properties.get("color", (1.0, 1.0, 1.0))))
        if "map" in properties:
            mtl.append(f"map_Kd {properties['map']}")
    (OUTPUT / f"{name}.mtl").write_text("\n".join(mtl) + "\n")
    print(f"{name}: {(index - 1) // 3} triangles")


def main():
    repo = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / "git" / "mrs_uav_gazebo_simulation")
    source = repo / MESHES
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for texture in ("carbon.jpg", "carbon_light.jpg"):
        shutil.copy(source / texture, OUTPUT / texture)
    shutil.copy(repo / "LICENSE", OUTPUT / "LICENSE.mrs_uav_gazebo_simulation")
    write_model("x500_body", BODY_PARTS, source)
    write_model("x500_prop_ccw", PROP_PARTS, source)
    write_model("x500_prop_cw", PROP_PARTS, source, mirror_x=True)


if __name__ == "__main__":
    main()
