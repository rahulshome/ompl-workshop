import os
from pathlib import Path
import time
import types
import numpy as np
from ompl import base as ob
from ompl import geometric as og

import vamp
import mbm
from vis import export_trajectory

DIMENSION = 7

# Top down grasp
# Q_GRASP = [1.1336, 0.9205, -0.191, -1.4283, 0.2096, 2.3289, 1.6573]

# Custom grasp 
Q_GRASP = [0.9500, 0.8650, -0.0300, -1.9560, 2.9000, 1.9210, 0.8350]


class VampMotionValidator(ob.MotionValidator):
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(
        self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType
    ):
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
    """A state validity checker for a VAMP robot's configuration."""

    robot: types.ModuleType
    env: vamp.Environment

    def __init__(
        self, si: ob.SpaceInformation, env: vamp.Environment, robot: types.ModuleType
    ):
        super().__init__(si)
        self.env = env
        self.robot = robot

    def isValid(self, s: ob.RealVectorStateType) -> bool:
        return self.robot.validate(s[: self.robot.dimension()], self.env)


class VampStateSpace(ob.RealVectorStateSpace):
    """A real-valued state space for a given VAMP robot."""

    robot: types.ModuleType
    dimension: int

    def __init__(self, robot: types.ModuleType):
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

# Environment is loaded from problems/pick_demo_panda/scene.yaml


def state_from_q(si: ob.SpaceInformation, q: list) -> ob.State:
    state = si.allocState()
    state[:DIMENSION] = q
    return state


def get_gripper_center(spheres) -> list:
    # Sphere 57 is on the left finger tip, 55 is on the right finger tip.
    # We average both finger tips to find the grasp center.
    left_finger = spheres[57].position
    right_finger = spheres[55].position
    mid = [
        (left_finger[0] + right_finger[0]) / 2.0,
        (left_finger[1] + right_finger[1]) / 2.0,
        (left_finger[2] + right_finger[2]) / 2.0,
    ]
    
    # Vector from palm center (sphere 30) to midpoint to align with gripper axis
    palm = spheres[30].position
    v = [mid[0] - palm[0], mid[1] - palm[1], mid[2] - palm[2]]
    
    # Push the ball outward (away from palm) by 3.5 cm (so it sits between fingers)
    norm = np.linalg.norm(v)
    if norm > 1e-5:
        v_unit = [vi / norm for vi in v]
        mid = [mid[i] + v_unit[i] * 0.035 for i in range(3)]
        
    return mid


def main():
    import sys
    problem_dir = "pick_demo_panda"
    if len(sys.argv) > 1 and "cage" in sys.argv[1].lower():
        problem_dir = "pick_cage_panda"
        print(f"Loading cage environment: {problem_dir}")
    else:
        print(f"Loading standard environment: {problem_dir}")

    scene_path = os.path.join(os.path.dirname(__file__), f"../problems/{problem_dir}/scene.yaml")
    request_path = os.path.join(os.path.dirname(__file__), f"../problems/{problem_dir}/request.yaml")
    
    env = mbm.load_scene(scene_path)
    robot = vamp.panda
    
    q_start, _ = mbm.load_request(request_path)

    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)
    si.setMotionValidator(VampMotionValidator(si=si, env=env, robot=robot))
    si.setStateValidityChecker(VampStateValidityChecker(si=si, env=env, robot=robot))

    ss1 = og.SimpleSetup(si)
    ss1.setPlanner(og.RRTConnect(si))
    # ==========================================
    # STAGE 1: Home -> Pick (Approach)
    # ==========================================
    print("\n--- Planning: Home -> Top-Down Grasp ---")
    start_state = state_from_q(si, q_start)
    pick_state = state_from_q(si, Q_GRASP)
    ss1.setStartAndGoalStates(start_state, pick_state)

    start_time = time.time()
    solved = ss1.solve(5.0)
    print(f"Stage 1 solved in {time.time() - start_time:.4f}s (Success: {bool(solved)})")

    if not solved:
        print("Failed to solve Stage 1.")
        return

    path1 = ss1.getSolutionPath()

    # Interpolate paths for smooth animation
    path1.interpolate(path1.getStateCount() * 15)

    # ==========================================
    # Export trajectory data to JSON for browser
    # ==========================================
    json_path = os.path.join(os.path.dirname(__file__), "trajectory_data.json")
    export_trajectory(json_path, robot, path1, scene_path)
    
    print("Demo complete!")


if __name__ == "__main__":
    main()
