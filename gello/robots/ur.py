from typing import Dict

import numpy as np

from gello.robots.robot import Robot

import cyclonedds as dds
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.util import duration
from sensor_comm_dds.utils.liveliness_listener import LivelinessListener
from sensor_comm_dds.communication.data_classes.irtouch32 import IRTouch32
from sensor_comm_dds.communication.data_classes.sequence import Sequence


class URRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.168.1.10", no_gripper: bool = False):
        import rtde_control
        import rtde_receive

        [print("in ur robot") for _ in range(4)]
        try:
            self.robot = rtde_control.RTDEControlInterface(robot_ip)
        except Exception as e:
            print(e)
            print(robot_ip)

        self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not no_gripper:
            from gello.robots.robotiq_gripper import RobotiqGripper

            self.gripper = RobotiqGripper()
            self.gripper.connect(hostname=robot_ip, port=63352)
            print("gripper connected")
            # gripper.activate()

        [print("connect") for _ in range(4)]

        self._free_drive = False
        self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

        # Sensor readers
        # qos = dds.qos.Qos()
        # # qos.history.kind = dds.HistoryKind.KEEP_LAST
        # # qos.history.depth = 1
        # irtouch32_listener = LivelinessListener(topic_name="IRTouch32")
        # irtouch32_domain_participant = DomainParticipant()
        # irtouch32_topic = Topic(irtouch32_domain_participant, "IRTouch32", IRTouch32)
        # self.irtouch32_reader = DataReader(irtouch32_domain_participant, irtouch32_topic, listener=irtouch32_listener, qos=qos)

        # accelnet_listener = LivelinessListener(topic_name="AccelNet")
        # accelnet_domain_participant = DomainParticipant()
        # accelnet_topic = Topic(accelnet_domain_participant, "AccelNet", Sequence)
        # self.accelnet_reader = DataReader(accelnet_domain_participant, accelnet_topic, listener=accelnet_listener, qos=qos)

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        import time

        time.sleep(0.01)
        gripper_pos = self.gripper.get_current_position()
        assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        return gripper_pos / 255

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        robot_joints = np.array(self.r_inter.getActualQ())
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

    
    def get_tcp_pose(self) -> np.ndarray:
        tcp_pose_rotvec = self.r_inter.getActualTCPPose()
        return tcp_pose_rotvec

    
    def get_FT_readings(self) -> np.ndarray:
        wrench = self.r_inter.getActualTCPForce()
        return wrench


    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0 / 500  # 2ms
        lookahead_time = 0.2
        gain = 100

        robot_joints = joint_state[:6]
        t_start = self.robot.initPeriod()
        self.robot.servoJ(
            robot_joints, velocity, acceleration, dt, lookahead_time, gain
        )
        if self._use_gripper:
            gripper_pos = joint_state[-1] * 255
            self.gripper.move(gripper_pos, 255, 10)
        self.robot.waitPeriod(t_start)

    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.freedriveMode()
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.endFreedriveMode()

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = self.get_tcp_pose()
        wrench = self.get_FT_readings()
        
        # fingertips = self.irtouch32_reader.read_one(timeout=duration(seconds=5)).taxel_values
        # accelerometer = self.accelnet_reader.read_one(timeout=duration(seconds=5)).values[2]  # Only taking the z-axis

        obs_dict = {
            "joint_positions": joints,
            "tcp_pose_rotvec": pos_quat,
            "wrench": wrench,
            # "fingertips": fingertips,
            # "accelerometer": accelerometer,
           # "microphone": spectogram
        }

        if self._use_gripper:
            gripper_pos = np.array([self.get_joint_state()[-1]])
        else:
            gripper_pos = np.array([self.get_joint_state()[-1]])
        obs_dict["gripper_position"] = gripper_pos

        return obs_dict
def main():

    robot_ip = "10.42.0.162"
    ur = URRobot(robot_ip, no_gripper=True)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())


if __name__ == "__main__":
    main()
