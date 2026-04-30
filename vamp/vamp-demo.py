import time
from pathlib import Path
from typing import Any, Sequence

import rerun as rr
from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot

import vamp

SPHERE_CENTERS = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
]
SPHERE_RADII = [0.2] * len(SPHERE_CENTERS)
SPHERES = [vamp.Sphere(p, r) for (p, r) in zip(SPHERE_CENTERS, SPHERE_RADII)]

DIMENSION = 7
Q_START = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
Q_END = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]

vamp.panda.joint_names


def main():

    env = make_environment(SPHERES)

    # Use VAMP's robot module to initialize state space and validations
    robot = vamp.panda
    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)

    motion_validator = VampMotionValidator(si=si, env=env, robot=robot)
    state_validity_checker = VampStateValidityChecker(si=si, env=env, robot=robot)

    # Select a planner
    planner = og.RRTConnect(si)

    # Set validators
    si.setMotionValidator(motion_validator)
    si.setStateValidityChecker(state_validity_checker)

    # Build SimpleSetup object
    ss = og.SimpleSetup(si)
    ss.setPlanner(planner)

    # Create start and goal states
    start = si.allocState()
    start[:DIMENSION] = Q_START
    goal = si.allocState()
    goal[:DIMENSION] = Q_END
    ss.setStartAndGoalStates(start, goal)

    # Solve
    planning_start = time.time()
    result = ss.solve(5.0)
    planning_time = time.time() - planning_start

    if not result:
        raise TimeoutError(f"No solution found in {planning_time}s")

    print(f"Found trajectory in {planning_time}s")
    path = ss.getSolutionPath()
    print(f"Generated path with {path.getStateCount()} states")

    rec = rr.RecordingStream("ompl-vamp-demo")
    rec.set_time("frame_idx", sequence=0)
    rec.spawn()
    log_environment(rec, "spheres", SPHERES)
    log_traj(rec, path)


class VampMotionValidator(ob.MotionValidator):
    """Motion validator using VAMP collision checking"""

    robot: Any
    env: vamp.Environment

    def __init__(self, si: ob.SpaceInformation, env: vamp.Environment, robot: Any):
        super().__init__(si)
        self.env = env
        self.robot = robot

    def checkMotion(
        self, s1: ob.RealVectorStateType, s2: ob.RealVectorStateType
    ) -> bool:
        config1 = s1[: self.robot.dimension()]
        config2 = s2[: self.robot.dimension()]
        return self.robot.validate_motion(config1, config2, self.env)


class VampStateValidityChecker(ob.StateValidityChecker):
    robot: Any
    env: vamp.Environment

    def __init__(self, si: ob.SpaceInformation, env: vamp.Environment, robot: Any):
        super().__init__(si)
        self.env = env
        self.robot = robot

    def isValid(self, s: ob.RealVectorStateType) -> bool:
        return self.robot.validate(s[0 : self.robot.dimension()], self.env)


class VampStateSpace(ob.RealVectorStateSpace):
    """State Space class using VAMP's robot methods"""

    def __init__(self, robot: Any):
        super().__init__(robot.dimension())
        self.robot = robot
        self.dimension = robot.dimension()
        bounds = ob.RealVectorBounds(self.dimension)
        upper_bounds = robot.upper_bounds()
        lower_bounds = robot.lower_bounds()

        for i in range(self.dimension):
            bounds.setLow(i, lower_bounds[i])
            bounds.setHigh(i, upper_bounds[i])
        self.setBounds(bounds)


def validate_panda_state(s: ob.RealVectorStateType, env: vamp.Environment) -> bool:
    """
    Determine whether a Panda configuration in environment `env` is valid.
    """
    return vamp.panda.validate(s[0:DIMENSION], env)


def make_environment(spheres: Sequence[vamp.Sphere]) -> vamp.Environment:
    """
    Construct the collision-checking environment for this problem.
    """
    env = vamp.Environment()
    for sphere in spheres:
        env.add_sphere(sphere)
    return env


def log_environment(rec: rr.RecordingStream, path: str, spheres: Sequence[vamp.Sphere]):
    """
    Log the collision-checking environment from this demo.
    """
    rec.log(
        path,
        rr.Ellipsoids3D(
            centers=[s.position for s in spheres],
            half_sizes=[[s.r] * 3 for s in spheres],
            colors=[[255, 255, 0]] * len(spheres),
            fill_mode=rr.components.FillMode.Solid,
        ),
    )
    # hack to make coordinate frames make sense
    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame=f"tf#/{path}"),
        static=True,
    )


def log_traj(rec: rr.RecordingStream, traj: og.PathGeometric):

    # `log_file_from_path` automatically uses the built-in URDF data-loader.
    urdf_path = Path(__file__).parent / "panda/panda.urdf"
    rec.log_file_from_path(urdf_path, static=True)
    # Load the URDF tree structure into memory
    urdf_tree = rr.urdf.UrdfTree.from_file_path(urdf_path)

    # The `flush` call is optional, but it helps with logging consistency,
    # because it ensures that the URDF finishes loading before continuing.
    rec.flush()

    traj.interpolate(traj.getStateCount() * 15)
    i = 0
    print(vamp.panda.joint_names())
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
        i += 1

    rec.log(
        "transforms",
        rr.Transform3D(parent_frame="world", child_frame="panda_link0"),
        static=True,
    )


if __name__ == "__main__":
    main()
