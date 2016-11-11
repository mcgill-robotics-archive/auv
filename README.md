# McGill Robotics Autonomous Underwater Vehicle

This is the repository for the McGill Robotics Autonomous Underwater Vehicle
project. The AUV competes in the AUVSI's RoboSub competition, hosted annually
at the TRANSDEC facility in San Diego.

## Prerequisites

Before working on this repository, please clone the [CompSys](https://github.com/mcgill-robotics/compsys)
repository and follow the instructions in the README to set up your environment
properly. If you have already installed and run `compsys` and you wish to manually update dependencies, do:

```bash
./setup.sh
```


## Cloning & Updating the Repository

This repository contains [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules).
To update the submodules, do:

```bash
git pull && git submodule update --init --recursive
rosdep update
```

## How to Run the AUV

1. Connect to the AUV's network, either by connecting the ethernet cable to our
router and connecting to McGill Robotics WiFi, or by connecting the ethernet
cable directly to your computer.
2. In a terminal, type:

   ```bash
   robotx
   ```

   This creates an SSH session to the AUV.
3. In the session, type:

   ```bash
   mux start auv
   ```

   This will open a [tmuxinator](https://github.com/tmuxinator/tmuxinator)
   session with all nodes running. You can then view all the processes in
   separate windows.

## Build Status

[master]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=auv_master
[master url]: http://dev.mcgillrobotics.com:8080/job/auv_master

[dev]: http://dev.mcgillrobotics.com:8080/buildStatus/icon?job=auv_dev
[dev url]: http://dev.mcgillrobotics.com:8080/job/auv_dev

| Branch   | Status                  |
|:--------:|:-----------------------:|
| `master` | [![master]][master url] |
| `dev`    | [![dev]][dev url]       |
