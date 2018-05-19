# Robot Package

This package includes some auv-specific tools, hardware-interfaces, models,
and messages.

Not necessarily an auxiliary package... But pretty damn close to it right now...

## Dry Test:

Part of this package includes scripts to automate drytests, allowing one to
easily check the functionality of hardware.

**To check sensors**:
1. _Make sure ros is running (`roscore`)
2. _Initialize all available sensors manually or through tmuxinator
3. `rosrun robot sensors.py`

**To check thrusters**:
1. _Make sure ros is running (`roscore`)
3. `rosrun robot thrusters.py`

**To run a full dry test**:
1. _Make sure ros is running (`roscore`)
2. _Initialize all available sensors manually or through tmuxinator
3. `roslaunch <path_to_package>/launch/drytest.launch`

_(Note that this also automatically runs image_view on the cameras to check
them as well)
