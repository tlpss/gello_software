- start up the robot, initialize and start (brakes should make noise)
- start the gripper
- turn on the power supply (5v) 
- activate the gripper


open 2 terminals, activate conda `gello` env in both.

# first command
python experiments/launch_nodes.py --robot=ur



# second command
first hold the gello arm in the 'home' position, then start the position mimicking using the following command:

python experiments/run_env.py --agent=gello


