# McGill Robotics Planner

## To run

```bash
roslaunch planner planner.launch
```

## Mission switch

The main planner state machine responsible for executing tasks will begin once
the mission switch is activated, or, equivalently, run:

```bash
rostopic pub -1 /mission std_msgs/Bool "data: true"
```

To abort the planner state machine, kill the robot or, equivalently, run:

```bash
rostopic pub -1 /mission std_msgs/Bool "data: false"
```

## Run dependencies

The servers listed in the given YAML must be active. For AUV, run Taskr.
