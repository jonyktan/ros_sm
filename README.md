# ros_sm
 Toy state machine in ROS, using [YASMIN](https://github.com/uleroboticsgroup/yasmin) package.


## Docker

1. Build image

    ```
    docker build --file docker/Dockerfile --tag ros_sm:yasmin .
    ```

1. Run container (in WSL). Default container name is `ros_sm`

    ```
    ./docker/docker.sh
    ```

    - Make sure to check that `docker/docker.sh` points to correct docker image. 

1. Remember to build the ROS 2 packages before use. In `ros2_ws/`,

    ```
    colcon build
    ```

    - Add `--symlink-install` to avoid having to rebuild each time source files are edited. 
    - Add `--packages-select <package name>` to build specific packages only. 

1. Remember to source the ROS 2 workspace before use. In `ros2_ws/`, 
    
    ```
    . install/local_setup.bash
    ```


## System Overview

### ROS 2 Nodes

1. Main state machine node (`'ros_sm_node'`). 

1. Command line input to trigger state machine transitions (`'cmdline_publisher'`). 

    - Accepts inputs in 2 parts: `'command'` (required) and `'goal value'` (optional).

    - List of accepted `'command'` (depends on current state):

        1. `'do_action'`. Requires a positive integer for `'goal value'`.

        1. `'call_service'`. Requires a list of positive integers for `'goal value'` (e.g. '1,2,3').

        1. `'re_init'`

        1. `'failed'`. Transits back to current state. 

        1. `'end'`

1. Action server (`'simple_action_node'`). Required for `'do_action'`.

1. Service (`'simple_service_node'`). Required for `'call_service'`. 

1. [OPTIONAL] State machine viewer (`'yasmin_viewer_node'`). Displays state machine in browser. 


### State Machine

| State | Transition | Outcome |
| :---: | :---: | :---: |
| `INIT` | `'do_action'` | `DOING_ACTION` |
| `INIT` | `'call_service'` | `CALLING_SERVICE` |
| `DOING_ACTION` | Action complete | `STANDBY` |
| `DOING_ACTION` | Action aborted | `STANDBY` |
| `CALLING_SERVICE` | Service complete | `STANDBY` |
| `CALLING_SERVICE` | Service aborted | `STANDBY` |
| `STANDBY` | `'do_action'` | `DOING_ACTION` |
| `STANDBY` | `'call_service'` | `CALLING_SERVICE` |
| `STANDBY` | `'re_init'` | `INIT` |
| `STANDBY` | `'end'` | `END` |


## Start Up

In any order, launch the ROS 2 nodes by running the corresponding ROS 2 executables. 

-   ```
    ros2 run ros_sm ros_sm
    ```

-   ```
    ros2 run cmdline_publisher cmdline_publisher
    ```

-   ```
    ros2 run servers simple_action_server
    ```

-   ```
    ros2 run servers simple_service_server
    ```

-   ```
    ros2 run yasmin_viewer yasmin_viewer_node
    ```


## Test Action Server

1. With the action server running in one terminal, use another terminal to send request to action server.

    ```
    ros2 action send_goal simple_action_name custom_interfaces/action/Simpleactiontype "{simple_request: 3}"
    ```

    - Add `--feedback` to display feedback messages.


## TODO

- [ ] Allow transition back to last state. E.g. if action/service aborted/failed. 