import math
import random
from pathlib import Path
from typing import Sequence, List
import json
import types

try:
    import rerun as rr
except ImportError:
    rr = None
import scipy
from ompl import geometric as og

import vamp


def log_environment(rec: "rr.RecordingStream", path: str, env: vamp.Environment):
    """
    Log the collision-checking environment from this demo.
    """
    rec.log(
        f"{path}/spheres",
        rr.Ellipsoids3D(
            centers=[s.position for s in env.spheres],
            half_sizes=[[s.r] * 3 for s in env.spheres],
            colors=[[253, 253, 150]] * len(env.spheres),
            fill_mode=rr.components.FillMode.Solid,
        ),
        static=True,
    )
    rec.log(
        f"{path}/cuboids",
        rr.Boxes3D(
            centers=[[c.x, c.y, c.z] for c in env.cuboids],
            half_sizes=[[c.axis_1_r, c.axis_2_r, c.axis_3_r] for c in env.cuboids],
            quaternions=[
                quat_from_r(
                    [
                        [c.axis_1_x, c.axis_2_x, c.axis_3_x],
                        [c.axis_2_y, c.axis_2_y, c.axis_3_y],
                        [c.axis_3_z, c.axis_2_z, c.axis_3_z],
                    ]
                )
                for c in env.cuboids
            ],
            colors=[[239, 167, 207]] * len(env.cuboids),
            fill_mode=rr.components.FillMode.Solid,
        ),
        static=True,
    )
    rec.log(
        f"{path}/z_cuboids",
        rr.Boxes3D(
            centers=[[c.x, c.y, c.z] for c in env.z_aligned_cuboids],
            half_sizes=[
                [c.axis_1_r, c.axis_2_r, c.axis_3_r] for c in env.z_aligned_cuboids
            ],
            quaternions=[
                quat_from_r(
                    [
                        [c.axis_1_x, c.axis_2_x, c.axis_3_x],
                        [c.axis_2_y, c.axis_2_y, c.axis_3_y],
                        [c.axis_3_z, c.axis_2_z, c.axis_3_z],
                    ]
                )
                for c in env.z_aligned_cuboids
            ],
            colors=[[246, 180, 201]] * len(env.z_aligned_cuboids),
            fill_mode=rr.components.FillMode.Solid,
        ),
        static=True,
    )
    rec.log(
        f"{path}/capsules",
        rr.Capsules3D(
            translations=[[c.x1, c.y1, c.z1] for c in env.capsules],
            lengths=[1 / c.rdv for c in env.capsules],
            quaternions=[capsule_quat([c.xv, c.yv, c.zv]) for c in env.capsules],
            colors=[[255, 178, 111]] * len(env.capsules),
            radii=[c.r for c in env.capsules],
            fill_mode=rr.components.FillMode.Solid,
        ),
        static=True,
    )
    rec.log(
        f"{path}/z_capsules",
        rr.Capsules3D(
            translations=[[c.x1, c.y1, c.z1] for c in env.z_aligned_capsules],
            lengths=[1 / c.rdv for c in env.z_aligned_capsules],
            quaternions=[
                capsule_quat([c.xv, c.yv, c.zv]) for c in env.z_aligned_capsules
            ],
            colors=[[180, 210, 130]] * len(env.z_aligned_capsules),
            radii=[c.r for c in env.z_aligned_capsules],
            fill_mode=rr.components.FillMode.Solid,
        ),
        static=True,
    )

    # # test code for checking correctness of vis
    # colliding = []
    # for _ in range(10000):
    #     x = random.uniform(-2, 2)
    #     y = random.uniform(-2, 2)
    #     z = random.uniform(-2, 2)

    #     if not vamp.sphere.validate([x, y, z], env):
    #         colliding.append([x, y, z])

    # rec.log(
    #     f"{path}/colliding",
    #     rr.Ellipsoids3D(
    #         centers=[s for s in colliding],
    #         half_sizes=[[0.2] * 3] * len(colliding),
    #         colors=[[253, 253, 150]] * len(colliding),
    #         fill_mode=rr.components.FillMode.Solid,
    #     ),
    #     static=True,
    # )

    # hack to make coordinate frames make sense
    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame=f"tf#/{path}"),
        static=True,
    )

    rec.flush()


def log_traj(rec: "rr.RecordingStream", traj: og.PathGeometric):

    # `log_file_from_path` automatically uses the built-in URDF data-loader.
    urdf_path = Path(__file__).parent.parent / "panda/panda.urdf"
    rec.log_file_from_path(urdf_path, static=True)
    # Load the URDF tree structure into memory
    urdf_tree = rr.urdf.UrdfTree.from_file_path(urdf_path)

    # The `flush` call is optional, but it helps with logging consistency,
    # because it ensures that the URDF finishes loading before continuing.
    rec.flush()

    traj.interpolate(traj.getStateCount() * 15)
    i = 0

    for q in traj.getStates():
        for theta, joint_name in zip(q, vamp.panda.joint_names()):
            # Animate joints by logging transforms
            for joint in urdf_tree.joints():
                if joint.name == joint_name:
                    # compute_transform gives you a complete transform that is ready to log,
                    # calculated from joint origin and the current angle and with the frame names set.
                    transform = joint.compute_transform(theta)
                    rec.set_time("frame_idx", sequence=i)
                    rec.log("transforms", transform)

        spheres = vamp.panda.fk(q[:7])
        rec.log(
            "robot_spheres",
            rr.Ellipsoids3D(
                centers=[s.position for s in spheres],
                half_sizes=[[s.r] * 3 for s in spheres],
                colors=[[253, 253, 200]] * len(spheres),
                fill_mode=rr.components.FillMode.DenseWireframe,
            ),
        )
        rec.log(
            "transforms",
            rr.Transform3D(parent_frame="world", child_frame="tf#/robot_spheres"),
            static=True,
        )
        i += 1

    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame="panda_link0"),
        static=True,
    )
    rec.flush()


def quat_from_r(R) -> "rr.Quaternion":
    """
    Convert a rotation matrix to a Rerun quaternion.
    """
    [r11, r12, r13] = R[0]
    [r21, r22, r23] = R[1]
    [r31, r32, r33] = R[2]

    # Calculate quaternion components
    q0 = 0.5 * math.sqrt(1 + r11 + r22 + r33)
    q1 = 0.5 * math.copysign(math.sqrt(1 + r11 - r22 - r33), r32 - r23)
    q2 = 0.5 * math.copysign(math.sqrt(1 - r11 + r22 - r33), r13 - r31)
    q3 = 0.5 * math.copysign(math.sqrt(1 - r11 - r22 + r33), r21 - r12)

    return rr.Quaternion(xyzw=[q1, q2, q3, q0])


def capsule_quat(v: Sequence[float]) -> "rr.Quaternion":
    norm = math.sqrt(sum(vi * vi for vi in v))
    v = [vi / norm for vi in v]

    theta = -math.acos(v[2])
    rotvec = [-v[1] * theta, v[0] * theta, 0]
    rotation = scipy.spatial.transform.Rotation.from_rotvec(rotvec)
    if v[2] == -1:
        # special case for 180 degree rotations
        rotation = scipy.spatial.transform.Rotation.from_euler(
            "xyz", [180, 0, 0], degrees=True
        )

    # TODO fix this to make capsules correct

    return rr.Quaternion(xyzw=rotation.as_quat())


def export_trajectory(
    json_path: str,
    robot: types.ModuleType,
    path: Sequence,
    scene_path: str,
    cylinder_pos: List[float] = [0.4, 0.6, 0.0],
    dimension: int = 7
):
    """
    Export OMPL path states and environment obstacles to a JSON file for visualizer.html.
    """
    from typing import List
    import types
    import json
    import yaml
    from scipy.spatial.transform import Rotation
    
    with open(scene_path) as f:
        scene_data = yaml.safe_load(f)
        
    obstacles_data = []
    for obj in scene_data.get("world", {}).get("collision_objects", []):
        obj_id = obj.get("id", "")
        is_mounting_base = "mounting_base" in obj_id
        
        for prim, pose in zip(obj["primitives"], obj["primitive_poses"]):
            t = pose["position"]
            quat = pose["orientation"]
            r = Rotation.from_quat(quat)
            euler = list(r.as_euler("xyz"))
            ptype = prim["type"]
            dims = prim["dimensions"]
            
            if ptype == "box":
                obstacles_data.append({
                    "type": "cuboid",
                    "center": [float(val) for val in t],
                    "rotation": [float(val) for val in euler],
                    "half_extents": [float(d / 2.0) for d in dims],
                    "is_mounting_base": is_mounting_base
                })
            elif ptype == "cylinder":
                length = dims[0]
                radius = dims[1]
                start_pos = [float(t[0]), float(t[1]), float(t[2] - length / 2.0)]
                obstacles_data.append({
                    "type": "cylinder",
                    "start": start_pos,
                    "rotation": [float(val) for val in euler],
                    "radius": float(radius),
                    "length": float(length)
                })
                
    scene_name = "cage" if "cage" in scene_path.lower() else "standard"
    
    frames_data = []
    for q in path.getStates():
        spheres = robot.fk(q[:7])
        spheres_list = [[float(s.position[0]), float(s.position[1]), float(s.position[2]), float(s.r)] for s in spheres]
        frames_data.append({
            "stage": 1,
            "q": [float(q[i]) for i in range(dimension)],
            "robot_spheres": spheres_list,
            "object_pos": cylinder_pos
        })
        
    export_data = {
        "obstacles": obstacles_data,
        "frames": frames_data,
        "scene_name": scene_name
    }
    
    with open(json_path, "w") as f:
        json.dump(export_data, f, indent=2)
    print(f"Exported trajectory data to {json_path}")
