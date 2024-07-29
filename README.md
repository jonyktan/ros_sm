# ros_sm
 Toy state machine in ROS

## Docker

1. Build image

    ```
    docker build --file docker/Dockerfile --tag ros_sm:jazzy .
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

## Simple Action

1. In one terminal, run the server.

    ```
    ros2 run action_servers simple_action_server
    ```

1. In another terminal, send request to action server.

    ```
    ros2 action send_goal simple_action custom_action_interfaces/action/Simpleaction "{simple_request: 3}"
    ```

    - Add `--feedback` to display feedback messages.

## Command Line Publisher

To send instructions to the ROS state machine via CLI. 

1. In one terminal, run the state machine.

    ```
    ros2 run ros_sm ros_sm
    ```

1. In another terminal, run the command line publisher.

    ```
    ros2 run cmdline_publisher cmdline_publisher
    ```
