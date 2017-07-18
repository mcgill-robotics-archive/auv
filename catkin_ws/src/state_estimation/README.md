# State Estimation Package

## Tmux test setup
To make launching tests with bags easier, you can use the `lane_test.yml` file
in the `tmux/` directory of this package. Simply link it into your
`.tmuxinator` directory like this:

```bash
ln -s ${ROBOTIC_PATH}/auv/catkin_ws/src/state_estimation/tmux/lane_test.yml \
~/.tmuxinator/
```

now relaunch your terminal and you can start tests with

```bash
tmuxinator start lane_test
```
