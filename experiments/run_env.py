import datetime
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from sensor_comm_dds.communication.readers.data_publisher import DataPublisher
from sensor_comm_dds.communication.data_classes.publishable_integer import PublishableInteger

import numpy as np
import tyro

from gello.agents.agent import BimanualAgent, DummyAgent
from gello.agents.gello_agent import GelloAgent
from gello.data_utils.format_obs import save_frame
from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.zmq_core.robot_node import ZMQClientRobot
from gello.zmq_core.camera_node import ZMQClientCamera


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "gello"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5002
    hostname: str = "127.0.0.1"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 10
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = True
    data_dir: str = "~/bc_data/mic-switches"
    bimanual: bool = False
    verbose: bool = False
    no_gripper: bool = False


def main(args):
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
           "left-wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
           # "right-wrist": ZMQClientCamera(port=5001, host=args.hostname),
           "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        #camera_clients  ={}
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)
    toggle_LED_cmd_publisher = DataPublisher(topic_name="ToggleLEDsCmd", topic_data_type=PublishableInteger)

    print("Robot initialized, env created")
    if args.bimanual:
        if args.agent == "gello":
            # dynamixel control box port map (to distinguish left and right gello)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792AL6-if00-port0"
            left_agent = GelloAgent(port=left)
            right_agent = GelloAgent(port=right)
            agent = BimanualAgent(left_agent, right_agent)
        elif args.agent == "quest":
            from gello.agents.quest_agent import SingleArmQuestAgent

            left_agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
            right_agent = SingleArmQuestAgent(
                robot_type=args.robot_type, which_hand="r"
            )
            agent = BimanualAgent(left_agent, right_agent)
            # raise NotImplementedError
        elif args.agent == "spacemouse":
            from gello.agents.spacemouse_agent import SpacemouseAgent

            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            left_agent = SpacemouseAgent(
                robot_type=args.robot_type, device_path=left_path, verbose=args.verbose
            )
            right_agent = SpacemouseAgent(
                robot_type=args.robot_type,
                device_path=right_path,
                verbose=args.verbose,
                invert_button=True,
            )
            agent = BimanualAgent(left_agent, right_agent)

        elif args.agent == "policy":
            from gello.agents.lerobot_agent import LeRobotAgent,LeRobotTactileAgent, load_act_policy

            checkpoint_path = "/home/tlips/Code/lerobot/outputs/train/2024-12-04/16-17-27_ur5e_act_ur5e-act-micro-all/checkpoints/040000/pretrained_model"
            policy = load_act_policy(checkpoint_path)
            agent = LeRobotTactileAgent(policy)
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([90, -90, 90, -90, -90, -90, 0])
        reset_joints_right = np.deg2rad([-90, -90, -90, -90, 90, 90, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])

        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
    else:
        if args.agent == "gello":
            gello_port = args.gello_port
            if gello_port is None:
                # hardcode right gello arm 
                gello_port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0"
                # usb_ports = glob.glob("/dev/serial/by-id/*")
                # print(f"Found {len(usb_ports)} ports")
                # if len(usb_ports) > 0:
                #     gello_port = usb_ports[0]
                #     print(f"using port {gello_port}")
                # else:
                #     raise ValueError(
                #         "No gello port found, please specify one or plug in gello"
                #     )
            if args.start_joints is None:
                reset_joints = np.deg2rad(
                    [0, -90, 90, -90, -90, 0, 0]
                )  # Change this to your own reset joints
                reset_joints = np.deg2rad([-90, -90, -90, -90, 90, 0, 0]) # left robot as single arm atm

                # reset_joints = np.deg2rad([-90, -90, -90, -90, 90, 90, 0]) # right ar, reset joints


                if args.no_gripper:
                    reset_joints = reset_joints[:-1]
            else:
                reset_joints = args.start_joints
            agent = GelloAgent(port=gello_port, start_joints=args.start_joints)

            print("-Getting observation for current joints")
            curr_joints = env.get_obs()["joint_positions"]
            if reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(int(max_delta / 0.001), 100)

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
        elif args.agent == "quest":
            from gello.agents.quest_agent import SingleArmQuestAgent

            agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
        elif args.agent == "spacemouse":
            from gello.agents.spacemouse_agent import SpacemouseAgent

            agent = SpacemouseAgent(robot_type=args.robot_type, verbose=args.verbose)
        elif args.agent == "dummy" or args.agent == "none":
            agent = DummyAgent(num_dofs=robot_client.num_dofs())
        elif args.agent == "policy":
            from gello.agents.lerobot_agent import LeRobotAgent, load_act_policy, LeRobotTactileAgent

            #
            # checkpoint_path = "/home/tlips/Code/gello_software/lerobot-output/checkpoints/coffee-handle-no-tactile/checkpoints/030000/pretrained_model/"
            # policy = load_act_policy(checkpoint_path)
            # agent = LeRobotAgent(policy)

            #checkpoint_path = "/home/tlips/Code/gello_software/lerobot-outputs/checkpoints/coffee-handle-tactile-chunk60/checkpoints/080000/pretrained_model/"
            #checkpoint_path = "/home/tlips/Code/lerobot/outputs/train/2024-08-14/17-04-36_ur5e_act_bimanual-ur5e-highest-point-towel/checkpoints/080000/pretrained_model"
            
            # button v1
            #checkpoint_path = "/home/tlips/Code/lerobot/outputs/train/2024-12-04/16-17-27_ur5e_act_ur5e-act-micro-all/checkpoints/040000/pretrained_model"
            
            # button v2
            checkpoint_path = "/home/tlips/Code/lerobot/outputs/train/2024-12-11/22-06-58_ur5e_act_ur5e-act-micro-all/checkpoints/020000/pretrained_model"
            policy = load_act_policy(checkpoint_path)
            agent = LeRobotTactileAgent(policy)
        else:
            raise ValueError("Invalid agent name")

    if args.use_save_interface:
        from gello.data_utils.keyboard_interface import KBReset

        kb_interface = KBReset()


    # going to start position
    print("Going to start position")
    start_pos = agent.act(env.get_obs())
    if args.no_gripper:
        start_pos = start_pos[:-1]

    obs = env.get_obs()
    joints = obs["joint_positions"]
    abs_deltas = np.abs(start_pos - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    max_joint_delta = 0.8
    if (abs_deltas[id_max_joint_delta] > max_joint_delta) and args.agent == "gello":
        print("current joints are too far from the start position!")
        id_mask = abs_deltas > max_joint_delta
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            start_pos[id_mask],
            joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    max_delta = 1/args.hz
    for _ in range(25):
        obs = env.get_obs()
        command_joints = agent.act(obs)
        if args.no_gripper:
            command_joints = command_joints[:-1]
        current_joints = obs["joint_positions"]
        delta = command_joints - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
        env.step(current_joints + delta)
        
    

    obs = env.get_obs()
    joints = obs["joint_positions"]
    action = agent.act(obs)
    if args.no_gripper:
        action = action[:-1]
    if (action - joints > 0.5).any() and args.agent == "gello":
        print("Action is too big")

        # print which joints are too big
        joint_index = np.where(action - joints > 0.8)
        for j in joint_index:
            print(
                f"Joint [{j}], leader: {action[j]}, follower: {joints[j]}, diff: {action[j] - joints[j]}"
            )
        exit()


    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))

    save_path = None
    start_time = time.time()


    # safety controller 
    from gello.safety_controller import URTableSimpleSafetyController

    #safety_controller = URPlanarSafetyController(-0.63,-0.25,-0.53,-0.09,0.01,0.26)
    safety_controller = URTableSimpleSafetyController(z_min=0.0, tcp_z_offset=0.172)

    while True:
        num = time.time() - start_time
        message = f"\rTime passed: {round(num, 2)}          "
        print_color(
            message,
            color="white",
            attrs=("bold",),
            end="",
            flush=True,
        )
        action = agent.act(obs)
        if args.no_gripper:
            action = action[:-1]

        # safety controller
        old_action = action.copy()
        action[:6] = safety_controller(action[:6], obs["joint_positions"][:6])
        if args.bimanual:
            action[7:13] = safety_controller(action[7:13],obs["joint_positions"][7:13])
        # safety controller 

        dt = datetime.datetime.now()
        if args.use_save_interface:
            state = kb_interface.update()
            if state == "start":
                dt_time = datetime.datetime.now()
                save_path = (
                    Path(args.data_dir).expanduser()
                    / args.agent
                    / dt_time.strftime("%m%d_%H%M%S")
                )
                save_path.mkdir(parents=True, exist_ok=True)
                print(f"Saving to {save_path}")
            elif state == "save":
                assert save_path is not None, "something went wrong"
                # TODO: this is a wonky place to add an observation, inconsistent with the project architecture, to be improved
                cropped_base_img = obs['base_rgb'].copy()
                cropped_base_img[:, 600:] = [0, 0, 0]
                cropped_base_img[:, :400] = [0, 0, 0]
                cropped_base_img[:400, :] = [0, 0, 0]
                obs['base_rgb_cropped'] = cropped_base_img
                save_frame(save_path, dt, obs, action)
            elif state == "normal":
                save_path = None
            elif state == "pause":
                continue  # skip executing the action
            elif state == "action1":
                toggle_LED_cmd_publisher.publish_sensor_data(PublishableInteger(1))
            else:
                raise ValueError(f"Invalid state {state}")
            
        before_obs = time.time()
        obs = env.step(action)  # execute action
        after_obs = time.time()
        difference = after_obs - before_obs
        #print(f"env step took {difference} seconds.")


if __name__ == "__main__":
    main(tyro.cli(Args))
