import time
from typing import Any, Dict, Optional

import numpy as np

from gello.cameras.camera import CameraDriver
from gello.robots.robot import Robot



class RobotEnv:
    def __init__(
        self,
        robot: Robot,
        control_rate_hz: float = 100.0,
        camera_dict: Optional[Dict[str, CameraDriver]] = None,
    ) -> None:
        self._robot = robot
        self._camera_dict = {} if camera_dict is None else camera_dict

    def robot(self) -> Robot:
        """Get the robot object.

        Returns:
            robot: the robot object.
        """
        return self._robot

    def __len__(self):
        return 0

    def step(self, joints: np.ndarray) -> Dict[str, Any]:
        """Step the environment forward.

        Args:
            joints: joint angles command to step the environment with.

        Returns:
            obs: observation from the environment.
        """
        self.act(joints)
        return self.get_obs()

    def act(self, action: np.ndarray):
        assert len(action) == (
            self._robot.num_dofs()
        ), f"input:{len(action)}, robot:{self._robot.num_dofs()}"
        self._robot.command_joint_state(action)


    def get_obs(self) -> Dict[str, Any]:
        """Get observation from the environment.

        Returns:
            obs: observation from the environment.
        """
        observations = {}
        import time 
        print("Getting camera observations")
        before_img_time = time.time()
        for name, camera in self._camera_dict.items():
            img, depth = camera.read((256, 128)) # lower res -> less (de)serialization overhead..
            observations[f"{name}_rgb"] = img
            observations[f"{name}_depth"] = depth
        after_img_time = time.time()
        print("Getting robot observations")
        robot_obs = self._robot.get_observations()
        after_robot_time = time.time()
        observations.update(robot_obs)


        print(f"observation collection time: {int((after_robot_time - before_img_time) * 1000)} ms, images: {int((after_img_time - before_img_time) * 1000)} ms, robot: {int((after_robot_time - after_img_time) * 1000)} ms")
        return observations

def main() -> None:
    pass


if __name__ == "__main__":
    main()
