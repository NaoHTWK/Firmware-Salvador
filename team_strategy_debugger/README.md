# Team Strategy Debugger

This directory contains a 2D visual debugger to test and visualize team strategies without a real robot.

## Overview

The debugger consists of two main components:
1.  `debugger`: The main application that renders the simulation. It displays a top-down view of the soccer field, the robots, and the ball. It uses SFML for graphics.
2.  `robot_process`: A separate process for each simulated robot. Each process runs an instance of a `TeamStrategy`.

Communication between the `debugger` and the `robot_process`es happens via shared memory, using the Boost.Interprocess library.

## Dependencies

You will need to have the following libraries installed on your system:
-   **SFML >= 2.5**: For graphics and windowing.
-   **Boost >= 1.71**: For inter-process communication and process management.

You can typically install them on a Debian-based system with:
```bash
sudo apt-get update
sudo apt-get install libsfml-dev libboost-all-dev
```

## How to Build

The debugger is integrated into the main CMake build system. 
1. Make sure you have installed the dependencies.
2. From the `build` directory of the project, run CMake and Make:
   ```bash
   cd /path/to/project/build
   cmake ..
   make
   ```
3. This will build the `debugger` and `robot_process` executables and place them in `build/team_strategy/debugger/`.

## How to Run

1.  Navigate to the directory where the executables were built.
2.  Run the main `debugger` application:
    ```bash
    ./debugger
    ```
This will open the simulation window and automatically spawn the `robot_process` instances. You can move the ball by clicking with the left mouse button in the window. The robots will currently just move in a circle around their starting point.

## Next Steps

This is a foundational framework. Future work includes:
-   Integrating the actual `TeamStrategy` logic into the `robot_process`.
-   Faking the pub/sub mechanism to feed data (ball position, etc.) to the strategies.
-   Implementing simplified "fake agents" that can execute the `Orders` produced by the strategies and update the robot's state in the simulation. 