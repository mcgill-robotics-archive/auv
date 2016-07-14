# Taskr

Taskr is an action server that listens to goals from planner. The goals come in
the form of a simple string containing the name of the task. Each task known to
planner and taskr has its own action server.

## To Run

Run the command:
```bash
roslaunch taskr taskr.launch
```

## Task Objects

Within the taskr node, each known task is a class which inherits from the
`Task` object. The class knows the name of of the associated YAML file, and
inherited functions iterate through actions in the file and handle the
execution of each.

The known tasks are:

* Initialize
* Bins
* Buoys
* Gate
* Maneuver
* Octagon
* Torpedo
* Square (misc test task)

## Actions

Each task is made up of a combination of five known actions:

* Initialize: Initialize horizon
* Move: Move to a given point
* Shoot: Shoot a torpedo
* Visual servo: Visual servo to a target
* Acoustic servo: Use hydrophones to servo to the pinger

Each action has its own python file containing an object which handles the
action execution. The constructor must accept the arguments associated with the
action from the YAML file. The object must own a function called `start()`
which accepts an action server instance, for dealing with feedback and preempt
requests, and a feedback message to populate.

### Move

Move dead reckons to a given point. It has a field called `feedback` which, if
`True`, will cause the action to listen for goals from sonar and vision and use
them to correct its yaw.

## YAML Config

Each task has its own YAML file. It contains a dictionary with a single key,
`actions`, which has as a value a list of action dictionaries, as such:

```yaml
actions:
  - move:
    position:
      x: [X]
      y: [Y]
      z: [Z]
    feedback: [BOOL]
  - visual_servo: "[LANDMARK]"
  - acoustic_servo: "hydrophones"
  - shoot: "[TORPEDO]"
```

## Testing

There is a fake planner client and fake controls server in the
[`test` directory](test) which are only to be used for debugging taskr without
the other nodes present.
To run, you will need three terminals:

1. `rosrun taskr fake_controls_server.py`
2. `roslaunch taskr taskr.launch`
3. `rosrun taskr fake_planner_client.py [TASK]`

You can tell the fake client to connect to any server you wish to test. The
TASK can be any of the following values.

- `square` [default]
- `initialize`
- `bin`
- `buoy`
- `gate`
- `maneuver`
- `octagon`
- `torpedo`


## Things That Might Not Work

* Messages temporarily live in old planner and need to be updated.
* YAMLs contain fake data.
* In `move.py`, the `VELOCITY` variable is apparently unrelated to m/s units.
  As such, the `get_time()` function will not produce expected values. Since
  position is a more human-understandable metric, velocity should be tuned here
  to have it correspond more closely to reality.
* In `move.py`, there is a confusing `range` parameter. This might not be doing
  the right thing.
