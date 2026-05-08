import math
from typing import Sequence

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import vamp

ARM_JOINT_NAMES = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
]


def load_request(request_yaml_path):
    """Return (start_q, goal_q) arm configs from a request YAML."""
    with open(request_yaml_path) as f:
        req = yaml.safe_load(f)

    start_map = dict(
        zip(
            req["start_state"]["joint_state"]["name"],
            req["start_state"]["joint_state"]["position"],
        )
    )
    goal_map = {
        jc["joint_name"]: jc["position"]
        for jc in req["goal_constraints"][0]["joint_constraints"]
    }

    start = [start_map[n] for n in ARM_JOINT_NAMES]
    goal = [goal_map[n] for n in ARM_JOINT_NAMES]
    return start, goal


def load_scene(scene_yaml_path):
    """Parse a scene YAML and build coal collision geometries.

    Returns a `vamp.Environment` with objects inserted.
    """
    with open(scene_yaml_path) as f:
        scene = yaml.safe_load(f)

    env = vamp.Environment()
    for obj in scene.get("world", {}).get("collision_objects", []):
        for prim, pose in zip(obj["primitives"], obj["primitive_poses"]):
            t = pose["position"]
            quat = pose["orientation"]
            print(quat)
            r = Rotation.from_quat(quat)
            # convert to rotation matri

            ptype = prim["type"]
            dims = prim["dimensions"]
            if ptype == "box":
                env.add_cuboid(vamp.Cuboid(t, r.as_euler("xyz"), [d / 2 for d in dims]))
            elif ptype == "cylinder":
                half_length = dims[0]
                radius = dims[1]
                p = list(np.array(t) + r.apply([[0, 0, -half_length]])[0])
                env.add_capsule(
                    vamp.Cylinder(
                        p,
                        r.as_euler("xyz"),
                        radius,
                        half_length * 2,
                    )
                )
            elif ptype == "sphere":
                env.add_sphere(vamp.Sphere(t, dims[0]))
            else:
                continue

    return env
