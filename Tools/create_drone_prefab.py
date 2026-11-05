#!/usr/bin/env python3
"""Generate the multirotor drone prefab and reference it from DemoLevel.

Standalone, no Editor needed; the Asset Processor picks the files up on its own:
    python3 Tools/create_drone_prefab.py

Prefabs are generated as JSON because the Editor's Python API cannot do it: entities it creates
belong to the level instance, CreatePrefab silently drops those, and an in-memory prefab cannot
be saved. Only non-default values are written; every other field takes its C++ default, which
includes the IMU (imu, 50 Hz) and odometry (odom, 10 Hz) publishers.

Ids derive from names, so reruns are stable.
"""

import hashlib
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LEVEL_PREFAB = PROJECT_ROOT / "Levels" / "DemoLevel" / "DemoLevel.prefab"
LEVEL_CONTAINER = "Entity_[1146574390643]"
DRONE_SOURCE = "Assets/Drone/Multirotor.prefab"

DRONE_NAME = "uav1"  # Also the ROS 2 namespace: /uav1/hw_api/attitude_rate_cmd, /uav1/imu, ...
SPAWN_POSITION = [1.5, 1.5, 0.3]  # [m], in the level.
MASS = 2.0  # [kg]; with the default motor model this hovers at ~0.5 throttle.
BODY_SIZE = [0.4, 0.4, 0.1]  # [m]

TRANSFORM_TYPE = "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent"
MULTIROTOR_TYPE = "{E4B7C2A9-5D16-4F38-9A0B-7C3E1D2F4A51} MultirotorComponent"
BOX_MODEL = {
    "assetId": {"guid": "{C15CD465-9589-56ED-945F-8416FA4798A3}", "subId": 284301017},
    "assetHint": "objects/_primitives/_box_1x1.fbx.azmodel",
}
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


def wrapped(template_type):
    """Serialized form of a runtime component with no Editor counterpart."""
    return {"$type": "GenericComponentWrapper", "m_template": {"$type": template_type}}


def make_entity(entity_id, name, parent, components, children=(), position=None):
    transform = {"$type": TRANSFORM_TYPE, "Parent Entity": parent}
    if position is not None:
        transform["Transform Data"] = {"Translate": position}

    all_components = {"TransformComponent": transform}
    for type_name in EDITOR_COMPONENTS:
        all_components[type_name] = {"$type": type_name}
    if children:
        all_components["EditorEntitySortComponent"]["Child Entity Order"] = list(children)

    for component in components:
        template = component.get("m_template", {}).get("$type", "")
        key = template.split()[-1] if template else component["$type"].split()[-1]
        all_components[key.replace("::", "_")] = component

    for key, component in all_components.items():
        component["Id"] = stable_id(name, key)
    return {"Id": entity_id, "Name": name, "Components": all_components}


def make_drone_prefab():
    drone = entity_key(DRONE_NAME)
    visual = entity_key("visual")
    return {
        "ContainerEntity": make_entity(
            "ContainerEntity",
            "Multirotor",
            "",
            [{"$type": "EditorPrefabComponent"}],
            children=[drone],
            position=[0.0, 0.0, 0.0],  # Written out so the level can patch it.
        ),
        "Entities": {
            drone: make_entity(
                drone,
                DRONE_NAME,
                "ContainerEntity",
                [
                    {
                        "$type": "EditorRigidBodyComponent",
                        "Configuration": {"entityId": "", "Compute Mass": False, "Mass": MASS},
                    },
                    {
                        "$type": "EditorColliderComponent",
                        "ShapeConfiguration": {"ShapeType": 1, "Box": {"Configuration": BODY_SIZE}},
                    },
                    {"$type": "ROS2FrameEditorComponent"},
                    wrapped(MULTIROTOR_TYPE),
                    wrapped("ROS2ImuSensorComponent"),
                    wrapped("ROS2OdometrySensorComponent"),
                ],
                children=[visual],
            ),
            visual: make_entity(
                visual,
                "visual",
                drone,
                [
                    {
                        "$type": "AZ::Render::EditorMeshComponent",
                        "Controller": {"Configuration": {"ModelAsset": BOX_MODEL}},
                    },
                    {"$type": "EditorNonUniformScaleComponent", "NonUniformScale": BODY_SIZE},
                ],
            ),
        },
    }


def add_instance_to_level():
    level = json.loads(LEVEL_PREFAB.read_text())
    instances = level["Instances"]
    for alias in [key for key, value in instances.items() if value["Source"] == DRONE_SOURCE]:
        del instances[alias]

    transform = "/ContainerEntity/Components/TransformComponent"
    patches = [{"op": "replace", "path": f"{transform}/Parent Entity", "value": f"../{LEVEL_CONTAINER}"}]
    for axis, value in enumerate(SPAWN_POSITION):
        patches.append({"op": "replace", "path": f"{transform}/Transform Data/Translate/{axis}", "value": value})

    instances[f"Instance_[{stable_id('instance', DRONE_SOURCE) % 10**12}]"] = {
        "Source": DRONE_SOURCE,
        "Patches": patches,
    }
    LEVEL_PREFAB.write_text(json.dumps(level, indent=4) + "\n")


def main():
    path = PROJECT_ROOT / DRONE_SOURCE
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(make_drone_prefab(), indent=4) + "\n")
    add_instance_to_level()
    print(f"wrote {DRONE_SOURCE} and updated {LEVEL_PREFAB.relative_to(PROJECT_ROOT)}")


if __name__ == "__main__":
    main()
