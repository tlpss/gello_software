

# installation

- create conda env 
- follow instructions in readme
- add 5V power suplly to U2D2 board and turn on switch. 
- install dynamixel wizard
- add user to dialout group (don't forget to log out)
- try to scan and check for connection with all motors.



# Teleop

## Configuration

- make sure the robot ip is correct (and the robot is in remote control)

### cameras
To change the used camera's:
1.  Edit `camera_configs` in `launch_camera_nodes.py`
2.  Edit the ports in `launch_camera_clients.py`
3.  Edit `camera_clients` in `run_env.py` 


### starting pose

to configure the starting poses of the robots, you have to set them in 


- calibrate the motor offsets for the dynamixels using 


right gello arm
```
python scripts/gello_get_offset.py --start-joints 0 -1.57 1.57 -1.57 -1.57 0 --joint-signs 1 1 -1 1 1 1 --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0

```

left arm 
```
python scripts/gello_get_offset.py --start-joints -1.57 -1.57 -1.57 -1.57 1.57 0 --joint-signs 1 1 -1 1 1 1 --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792AL6-if00-port0
```

these values should then be configured in the agents/gello_agent script.
- 
## runnning teleop
- start up the robot, initialize and start (brakes should make noise)
- start the gripper
- turn on the power supply (5v) 
- activate the gripper


### single robot arm
open 3 terminals and activate `gello-pt` conda env 

from dir `~/Code/gello_software`:

1. start up cameras: `python experiments/launch_camera_nodes.py`
2. start robot: `python experiments/launch_nodes.py`
3. start 'agent': `python experiments/run_env.py  --agent=policy` or `python experiments/run_env.py  --agent=gello`


### dual UR robot arm
1. start up cameras: ` python experiments/launch_nodes.py --robot bimanual_ur`
2. start robot: `python experiments/launch_nodes.py`
3. start 'agent': `python experiments/run_env.py  --agent=policy` or `python experiments/run_env.py  --agent=gello`


to capture demonstrations:

- use_save_interface & set data dir  in `experiments/run_env.py`
- pygame window will pop up, press S tot start, Q to finish
- to visualize saved pickles: `data_utils/explore_pickle.ipynb` 


to add sensor:
- in robots/ur/get_observation: add entry to observation dictionary (np.array)

pygame state machine:  data_utils/keyboard_interface
experiments/run_env.py