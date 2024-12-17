
import numpy as np
import torch
from gello.agents.agent import Agent

from lerobot.common.policies.diffusion.modeling_diffusion import DiffusionPolicy
from lerobot.common.policies.act.modeling_act import ACTPolicy
def load_act_policy(checkpoint_path: str) -> ACTPolicy:
    # Load the checkpoint
    policy = ACTPolicy.from_pretrained(checkpoint_path)
    policy.eval()
    return policy


def load_diffusion_policy(checkpoint_path: str) -> DiffusionPolicy:
    # Load the checkpoint
    policy = DiffusionPolicy.from_pretrained(checkpoint_path)
    policy.eval()
    return policy


class LeRobotAgent(Agent):
    def __init__(self,policy, controls_gripper: bool = True) -> None:
        self.policy = policy
        self.controls_gripper = controls_gripper

    def act(self, obs: dict) -> np.ndarray:
        """
        takes in a gello observation, turns it into a Lerobot observation and returns the action in gello format
        """

        base_image = obs["base_rgb"]
        #wrist_image = obs["wrist_rgb"]
        joint_positions = obs["joint_positions"]

        # already contains the gripper position
        state = torch.tensor(joint_positions).unsqueeze(0).float()

        # format to torch tensors
        base_image = torch.tensor(base_image).permute(2, 0, 1).unsqueeze(0).float() / 255
        #wrist_image = torch.tensor(wrist_image).permute(2, 0, 1).unsqueeze(0).float() / 255


        formatted_obs = {
            "observation.images.base.rgb": base_image,
            #"observation.images.wrist.rgb": wrist_image,
            "observation.state": state
        }
        action = self.policy.select_action(formatted_obs)

        action = action.squeeze().detach().cpu().numpy()

        # append gripper position dummy if the gripper was not controlled by the policy (output dim = 6)
        if len(action) == 6:
            action = np.append(action, 0.0)
        
        return action
    


class LeRobotTactileAgent(Agent):
    def __init__(self, policy) -> None:
        self.policy = policy

        # policy to cuda 
        self.policy = self.policy.to("cuda")

    def act(self, obs: dict) -> np.ndarray:
        """
        takes in a gello observation, turns it into a Lerobot observation and returns the action in gello format
        """

        base_image = obs["base_rgb"]
        wrist_image = obs["left-wrist_rgb"]
        mic_spectrogram = obs["mic_spectrogram"]
        joint_positions = obs["joint_positions"]
        switches = obs["switches"]

        # Hardcode observations here for ablation
        # switches = [0, 0]
        # wrist_image = np.zeros((480, 640, 3))


        # already contains the gripper position
        state = np.concatenate([joint_positions], axis=0)
        state = torch.tensor(state).unsqueeze(0).float()

        # format to torch tensors
        base_image = torch.tensor(base_image).permute(2, 0, 1).unsqueeze(0).float() / 255
        wrist_image = torch.tensor(wrist_image).permute(2, 0, 1).unsqueeze(0).float() / 255
        spectrogram = torch.tensor(mic_spectrogram).permute(2,0,1).unsqueeze(0).float() / 255


        # resize
        from torchvision.transforms import Resize, InterpolationMode
        resizer = Resize((256,256),InterpolationMode.BICUBIC)

        base_image = resizer(base_image)
        wrist_image = resizer(wrist_image)
        spectrogram = resizer(spectrogram)

        formatted_obs = {
            "observation.images.base.rgb": base_image,
            "observation.images.left-wrist.rgb": wrist_image,
            "observation.images.spectogram.rgb": spectrogram,
            "observation.state": state
        }

        # all observations to policy device
        for key in formatted_obs:
            formatted_obs[key] = formatted_obs[key].to("cuda")

        action = self.policy.select_action(formatted_obs)

        action = action.squeeze().detach().cpu().numpy()

        # append gripper position dummy if the gripper was not controlled by the policy (output dim = 6)
        if len(action) == 6:
            action = np.append(action, 0.0)
        
        return action

    
    
   
    
if __name__ == "__main__":
    checkpoint_path = "/home/tlips/Code/lerobot/outputs/train/2024-12-04/16-17-27_ur5e_act_ur5e-act-micro-all/checkpoints/040000/pretrained_model"
    policy = load_act_policy(checkpoint_path).to("cuda")

    # checkpoint_path = "/home/tlips/Code/lerobot/lerobot/outputs/train/2024-12-13/16-10-23_ur5e_diffusion_ur5e-dp-micro-wrist-base-spectrogram/checkpoints/080000/pretrained_model"
    # policy = load_diffusion_policy(checkpoint_path)
    observation = {
        "observation.images.base.rgb" : torch.rand(1,3, 256, 256),
        "observation.images.left-wrist.rgb" : torch.rand(1,3, 256, 256),
        "observation.images.spectogram.rgb" : torch.rand(1,3, 256, 256),
        "observation.state" : torch.rand(1, 7),
    }

    # all to policy device
    for key in observation:
        observation[key] = observation[key].to("cuda")

    action = policy.select_action(observation)
    print(action)