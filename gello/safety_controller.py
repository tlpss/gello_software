from typing import Any
import numpy as np 


class SafetyController:
    """
    Joint-space safety controller. can modify the target joint angles sent by an agent before sending them to the robot.
    """

    def __call__(self, target_joint_angles:np.ndarray, current_joint_angles: np.ndarray, **kwargs) -> np.ndarray:
        raise NotImplementedError
    
class URTableSafetyController:
    # safety controller to keep z coord above 1.0 cm

    def __init__(self, z_min=0.01, tcp_z_offset=0.176):
        self.z_min = z_min
        self.tcp_offset = np.array([0,0,tcp_z_offset])
        self.tcp_in_eef = np.eye(4)
        self.tcp_in_eef[:3,3] = self.tcp_offset

    def __call__(self, target_joint_angles:np.ndarray, current_joint_angles: np.ndarray, **kwargs) -> np.ndarray:
        from ur_analytic_ik import ur5e

        target_eef_pose = ur5e.forward_kinematics(*target_joint_angles)

        target_tcp_pose = target_eef_pose @ self.tcp_in_eef

        target_tcp_pose[2][3] = np.clip(target_tcp_pose[2][3], self.z_min, np.inf)

        target_eef_pose = target_tcp_pose @ np.linalg.inv(self.tcp_in_eef)
        target_joint_angles = ur5e.inverse_kinematics_closest(target_eef_pose, *current_joint_angles)

        return target_joint_angles[0][0]



class URPlanarSafetyController:
    def __init__(self,x_min, x_max, y_min, y_max, z, tcp_z_offset=0.176):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z = z
        self.tcp_offset = np.array([0,0,tcp_z_offset])
        self.tcp_in_eef = np.eye(4)
        self.tcp_in_eef[:3,3] = self.tcp_offset




    def __call__(self, target_joint_angles:np.ndarray, current_joint_angles: np.ndarray, **kwargs) -> np.ndarray:
        from ur_analytic_ik import ur5e

        target_eef_pose = ur5e.forward_kinematics(*target_joint_angles)

        target_tcp_pose = target_eef_pose @ self.tcp_in_eef


        target_tcp_pose[0][3] = np.clip(target_tcp_pose[0][3], self.x_min, self.x_max)
        target_tcp_pose[1][3] = np.clip(target_tcp_pose[1][3], self.y_min, self.y_max)
        target_tcp_pose[2][3] = self.z

        # top down orientation 
        target_tcp_pose[:3,:3] = np.array([[0,1,0],[1,0,0],[0,0,-1]])
        target_eef_pose = target_tcp_pose @ np.linalg.inv(self.tcp_in_eef)
        target_joint_angles = ur5e.inverse_kinematics_closest(target_eef_pose, *current_joint_angles)


        # clip target joints for max velocity.
        max_delta = 0.2 # 2 rad/s
        delta = target_joint_angles - current_joint_angles
        delta = np.clip(delta, -max_delta, max_delta)
        target_joint_angles = current_joint_angles + delta


        return target_joint_angles
    


if __name__ == "__main__":
    pass
    safety_controller = URTableSafetyController()
    target_joint_angles = np.array([0,0,0,0,0,0])
    current_joint_angles = np.array([0,0,0,0,0,0])
    print(safety_controller(target_joint_angles, current_joint_angles))
