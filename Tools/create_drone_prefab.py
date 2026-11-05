#!/usr/bin/env python3
"""Generate the multirotor drone prefab and reference it from DemoLevel.

Standalone, no Editor needed, but the x500 models from Tools/convert_x500_meshes.py must have been
processed by the Asset Processor first, because the prefab references them by asset id:
    python3 Tools/create_drone_prefab.py

Prefabs are generated as JSON because the Editor's Python API cannot do it: entities it creates
belong to the level instance, CreatePrefab silently drops those, and an in-memory prefab cannot
be saved. Only non-default values are written; every other field takes its C++ default, which
includes the IMU (imu) and odometry (odom) publishers.

Ids derive from names, so reruns are stable.
"""

import hashlib
import json
import sqlite3
import uuid
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LEVEL_PREFAB = PROJECT_ROOT / "Levels" / "DemoLevel" / "DemoLevel.prefab"
LEVEL_CONTAINER = "Entity_[1146574390643]"
DRONE_SOURCE = "Assets/Drone/Multirotor.prefab"

DRONE_NAME = "uav1"  # Also the ROS 2 namespace: /uav1/cmd/attitude_rate, /uav1/imu, ...
SPAWN_POSITION = [1.5, 1.5, 0.155]  # [m], in the level; the legs reach 0.1525 m below the origin.
MASS = 2.0  # [kg]; with the default motor model this hovers at ~0.5 throttle.
SENSOR_RATE = 250.0  # [Hz]; the rate of the MRS HW API.

# Forward-facing RGB camera. ROS cameras look along the entity's Z with X right and Y down.
CAMERA_POSITION = [0.14, 0.0, -0.02]  # [m], in the body frame, in front of the frame and below the props.
CAMERA_ROTATION = [180.0, 90.0, 90.0]  # [deg]
CAMERA_CONFIG = {
    "VerticalFieldOfViewDeg": 60.0,
    "Width": 640,
    "Height": 480,
    "Color": True,
    "Depth": False,
    "ClipNear": 0.02,
    "ClipFar": 100.0,
}
CAMERA_RATE = 30.0  # [Hz]

# The game view follows the drone. Atom cameras look along the entity's Y.
FOLLOW_CAMERA_POSITION = [-1.2, 0.0, 0.55]  # [m], in the body frame, behind and above.
FOLLOW_CAMERA_ROTATION = [0.0, 25.0, -90.0]  # [deg]; faces forward, tilted down at the drone.
# The ROSbot prefab's own camera would otherwise compete for the game view.
ROSBOT_SOURCE = "RosbotXL/ROSBotXL_Slamtec.prefab"
ROSBOT_CAMERA = (
    "/Instances/Instance_[1711844155761]/Entities/Entity_[5417796519198]"
    "/Components/Component_[16056351394850261418]"
)

# Geometry of models/mrs_robots_description/sdf/drones/x500.sdf.jinja in mrs_uav_gazebo_simulation.
INERTIA_DIAGONAL = [0.03167, 0.03167, 0.0625]  # Solid cylinder, r = 0.25 m, h = 0.05 m.
ROTOR_XY = 0.1812  # [m]
ROTOR_Z = 0.057  # [m]
LEG_XY = 0.13  # [m]
# PX4 motor order, as in the SDF: (x sign, y sign, clockwise).
ROTORS = [(1, -1, False), (-1, 1, False), (1, 1, True), (-1, -1, True)]

CAMERA_TYPE = "{CA11DA46-29FF-4083-B5F6-E02C3A8C3A3D} EditorCameraComponent"
TRANSFORM_TYPE = "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent"
MULTIROTOR_TYPE = "{E4B7C2A9-5D16-4F38-9A0B-7C3E1D2F4A51} MultirotorComponent"
FREE_CAMERA_TYPE = "{2F6D8A14-93C7-4E1B-B5A2-6C0E7D1F3B92} FreeCameraComponent"
EDITOR_COMPONENTS = [
    "EditorDisabledCompositionComponent",
    "EditorEntityIconComponent",
    "EditorEntitySortComponent",
    "EditorInspectorComponent",
    "EditorLockComponent",
    "EditorOnlyEntityComponent",
    "EditorPendingCompositionComponent",
    "EditorVisibilityComponent",
]


def stable_id(*names):
    digest = hashlib.sha256("/".join(names).encode()).digest()
    return int.from_bytes(digest[:8], "little") >> 1


def entity_key(name):
    return f"Entity_[{stable_id('entity', name) % 10**12}]"


def wrapped(template_type, **fields):
    """Serialized form of a runtime component with no Editor counterpart."""
    return {"$type": "GenericComponentWrapper", "m_template": {"$type": template_type, **fields}}


def model_asset(name):
    """Asset reference of a processed model, from the Asset Processor database."""
    product = f"linux/assets/drone/x500/{name}.obj.azmodel"
    database = sqlite3.connect(f"file:{PROJECT_ROOT / 'Cache' / 'assetdb.sqlite'}?mode=ro", uri=True)
    row = database.execute(
        "SELECT s.SourceGuid, p.SubID FROM Products p JOIN Jobs j ON p.JobPK = j.JobID "
        "JOIN Sources s ON j.SourcePK = s.SourceID WHERE lower(p.ProductName) = ?",
        (product,),
    ).fetchone()
    if row is None:
        raise SystemExit(f"{product} not found; run the Asset Processor first")
    return {
        "assetId": {"guid": "{" + str(uuid.UUID(bytes=row[0])).upper() + "}", "subId": row[1] & 0xFFFFFFFF},
        "assetHint": product.removeprefix("linux/"),
    }


def mesh(name):
    return {
        "$type": "AZ::Render::EditorMeshComponent",
        "Controller": {"Configuration": {"ModelAsset": model_asset(name)}},
    }


def box_collider(size, position):
    return {
        "$type": "EditorColliderComponent",
        "ColliderConfiguration": {"Position": position},
        "ShapeConfiguration": {"ShapeType": 1, "Box": {"Configuration": size}},
    }


def drone_colliders():
    """Boxes approximating the SDF's collision cylinders: body, legs, and prop discs."""
    colliders = [box_collider([0.2, 0.2, 0.1], [0.0, 0.0, -0.05])]
    for x_sign, y_sign, _ in ROTORS:
        colliders.append(box_collider([0.03, 0.03, 0.165], [x_sign * LEG_XY, y_sign * LEG_XY, -0.07]))
        colliders.append(box_collider([0.35, 0.02, 0.02], [x_sign * ROTOR_XY, y_sign * ROTOR_XY, ROTOR_Z]))
    return colliders


def camera_sensor():
    return {
        "$type": "ROS2CameraSensorEditorComponent",
        "CameraSensorConfig": CAMERA_CONFIG,
        "SensorConfig": {
            "Frequency (HZ)": CAMERA_RATE,
            "Publishers": {
                "Color Camera Info": {"Type": "sensor_msgs::msg::CameraInfo", "Topic": "color_camera_info"},
                "Color Image": {"Type": "sensor_msgs::msg::Image", "Topic": "camera_image_color"},
            },
        },
    }


def make_entity(entity_id, name, parent, components, children=(), position=None, rotation=None):
    transform = {"$type": TRANSFORM_TYPE, "Parent Entity": parent}
    if position is not None:
        transform["Transform Data"] = {"Translate": position}
        if rotation is not None:
            transform["Transform Data"]["Rotate"] = rotation

    all_components = {"TransformComponent": transform}
    for type_name in EDITOR_COMPONENTS:
        all_components[type_name] = {"$type": type_name}
    if children:
        all_components["EditorEntitySortComponent"]["Child Entity Order"] = list(children)

    for component in components:
        template = component.get("m_template", {}).get("$type", "")
        key = (template or component["$type"]).split()[-1].replace("::", "_")
        unique_key, count = key, 1
        while unique_key in all_components:
            count += 1
            unique_key = f"{key}_{count}"
        all_components[unique_key] = component

    for key, component in all_components.items():
        component["Id"] = stable_id(name, key)
    return {"Id": entity_id, "Name": name, "Components": all_components}


def make_drone_prefab():
    drone = entity_key(DRONE_NAME)
    visual = entity_key("visual")
    camera = entity_key("camera")
    follow_camera = entity_key("follow_camera")
    free_camera = entity_key("free_camera")
    props = [entity_key(f"prop_{i}") for i in range(len(ROTORS))]
    rotors = [
        {"Position": [x_sign * ROTOR_XY, y_sign * ROTOR_XY, ROTOR_Z], "Clockwise": clockwise, "Visual": prop}
        for (x_sign, y_sign, clockwise), prop in zip(ROTORS, props)
    ]
    i_xx, i_yy, i_zz = INERTIA_DIAGONAL
    sensor_config = {"Frequency (HZ)": SENSOR_RATE}

    entities = {
        drone: make_entity(
            drone,
            DRONE_NAME,
            "ContainerEntity",
            [
                {
                    "$type": "EditorRigidBodyComponent",
                    "Configuration": {
                        "entityId": "",
                        "Compute Mass": False,
                        "Mass": MASS,
                        # The colliders are off-center, computing these would tilt the drone.
                        "Compute COM": False,
                        "Compute inertia": False,
                        "Inertia tensor": [i_xx, 0.0, 0.0, 0.0, i_yy, 0.0, 0.0, 0.0, i_zz],
                    },
                },
                *drone_colliders(),
                {"$type": "ROS2FrameEditorComponent"},
                wrapped(MULTIROTOR_TYPE, Config={"Rotors": rotors}),
                wrapped("ROS2ImuSensorComponent", SensorConfiguration=sensor_config),
                wrapped("ROS2OdometrySensorComponent", SensorConfiguration=sensor_config),
            ],
            children=[visual, camera, follow_camera, *props],
        ),
        visual: make_entity(visual, "visual", drone, [mesh("x500_body")]),
        # Not a child of the drone, so that it stays where it is left.
        free_camera: make_entity(
            free_camera,
            "free_camera",
            "ContainerEntity",
            [
                {
                    "$type": CAMERA_TYPE,
                    "Controller": {"Configuration": {"Field of View": 60.0, "MakeActiveViewOnActivation": False}},
                },
                wrapped(FREE_CAMERA_TYPE, FollowCamera=follow_camera),
            ],
        ),
        camera: make_entity(
            camera,
            "camera",
            drone,
            [{"$type": "ROS2FrameEditorComponent"}, camera_sensor()],
            position=CAMERA_POSITION,
            rotation=CAMERA_ROTATION,
        ),
        follow_camera: make_entity(
            follow_camera,
            "follow_camera",
            drone,
            [{"$type": CAMERA_TYPE, "Controller": {"Configuration": {"Field of View": 60.0}}}],
            position=FOLLOW_CAMERA_POSITION,
            rotation=FOLLOW_CAMERA_ROTATION,
        ),
    }
    for i, (prop, rotor) in enumerate(zip(props, rotors)):
        model = "x500_prop_cw" if rotor["Clockwise"] else "x500_prop_ccw"
        entities[prop] = make_entity(prop, f"prop_{i}", drone, [mesh(model)], position=rotor["Position"])

    return {
        "ContainerEntity": make_entity(
            "ContainerEntity",
            "Multirotor",
            "",
            [{"$type": "EditorPrefabComponent"}],
            children=[drone, free_camera],
            position=[0.0, 0.0, 0.0],  # Written out so the level can patch it.
        ),
        "Entities": entities,
    }


def add_instance_to_level():
    level = json.loads(LEVEL_PREFAB.read_text())
    instances = level["Instances"]

    # An existing instance keeps its place, which may have been moved in the Editor.
    if not any(value["Source"] == DRONE_SOURCE for value in instances.values()):
        transform = "/ContainerEntity/Components/TransformComponent"
        patches = [{"op": "replace", "path": f"{transform}/Parent Entity", "value": f"../{LEVEL_CONTAINER}"}]
        for axis, value in enumerate(SPAWN_POSITION):
            patches.append({"op": "replace", "path": f"{transform}/Transform Data/Translate/{axis}", "value": value})
        instances[f"Instance_[{stable_id('instance', DRONE_SOURCE) % 10**12}]"] = {
            "Source": DRONE_SOURCE,
            "Patches": patches,
        }

    for rosbot in (value for value in instances.values() if value["Source"] == ROSBOT_SOURCE):
        path = f"{ROSBOT_CAMERA}/Controller/Configuration/MakeActiveViewOnActivation"
        if not any(patch["path"] == path for patch in rosbot["Patches"]):
            rosbot["Patches"].append({"op": "add", "path": path, "value": False})

    LEVEL_PREFAB.write_text(json.dumps(level, indent=4) + "\n")


def main():
    path = PROJECT_ROOT / DRONE_SOURCE
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(make_drone_prefab(), indent=4) + "\n")
    add_instance_to_level()
    print(f"wrote {DRONE_SOURCE} and updated {LEVEL_PREFAB.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()
