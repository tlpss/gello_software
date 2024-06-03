

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

- we assume the starting pose is the 90 90 90 90 config from gello
- calibrate the motor offsets for the dynamixels using 

```
python scripts/gello_get_offset.py --start-joints 0 -1.57 1.57 -1.57 -1.57 0 --joint-signs 1 1 -1 1 1 1 --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT792DZ5-if00-port0
```
- 
## runnning teleop
- start up the robot, initialize and start (brakes should make noise)
- start the gripper
- turn on the power supply (5v) 
- activate the gripper


open 2 terminals, activate conda `gello` env in both.

# first command
python experiments/launch_nodes.py --robot=ur (sim_ur)



# second command
first hold the gello arm in the 'home' position, then start the position mimicking using the following command:

python experiments/run_env.py --agent=gello


