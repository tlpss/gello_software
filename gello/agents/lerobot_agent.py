
import numpy as np
import torch
from gello.agents.agent import Agent
from lerobot.common.policies.act.modeling_act import ACTPolicy

def load_act_policy(checkpoint_path: str) -> ACTPolicy:
    # Load the checkpoint
    policy = ACTPolicy.from_pretrained(checkpoint_path)
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

        action = action.squeeze().detach().numpy()

        # append gripper position dummy if the gripper was not controlled by the policy (output dim = 6)
        if len(action) == 6:
            action = np.append(action, 0.0)
        
        return action
    


class LeRobotTactileAgent(Agent):
    def __init__(self, policy) -> None:
        self.policy = policy

    def act(self, obs: dict) -> np.ndarray:
        """
        takes in a gello observation, turns it into a Lerobot observation and returns the action in gello format
        """

        base_image = obs["base_rgb"]
        wrist_image = obs["wrist_rgb"]
        #wrist_image = np.zeros((480, 640, 3))# obs["wrist_rgb"]
        joint_positions = obs["joint_positions"]
        fingertips = obs["fingertips"]
        accelerometer = obs["accelerometer"]

        # fingertips = np.random.randint(25,35,(32,))
        # fingertips = [16, 35, 40, 34, 31, 15, 39, 38, 44, 8, 22, 35, 49, 38, 11, 26, 33, 28, 6, 24, 27, 28, 25, 30, 32, 26, 26, 27, 31, 28, 18, 24]
        # accelerometer = 0


        # already contains the gripper position
        state = np.concatenate([joint_positions, fingertips, [accelerometer]], axis=0)
        state = torch.tensor(state).unsqueeze(0).float()

        # format to torch tensors
        base_image = torch.tensor(base_image).permute(2, 0, 1).unsqueeze(0).float() / 255
        wrist_image = torch.tensor(wrist_image).permute(2, 0, 1).unsqueeze(0).float() / 255

        formatted_obs = {
            "observation.images.base.rgb": base_image,
            "observation.images.wrist.rgb": wrist_image,
            "observation.state": state
        }
        action = self.policy.select_action(formatted_obs)

        action = action.squeeze().detach().numpy()

        # append gripper position dummy if the gripper was not controlled by the policy (output dim = 6)
        if len(action) == 6:
            action = np.append(action, 0.0)
        
        return action

    
    
   
    
if __name__ == "__main__":
    checkpoint_path = "/home/tlips/Code/gello_software/lerobot-output/checkpoints/gello-planar-push-last"
    policy = load_act_policy(checkpoint_path)

    observation = {
        "observation.images.base.rgb" : torch.rand(1,3, 480, 640),
        "observation.images.wrist.rgb" : torch.rand(1,3, 480, 640),
        "observation.state" : torch.rand(1, 7),
    }

    action = policy.select_action(observation)
    print(action)