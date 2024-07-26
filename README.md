# ros_sm
 Toy state machine in ROS

## Docker

1. Build image

    ```
    docker build --file docker/Dockerfile --tag ros_sm .
    ```

1. Run container (in WSL). Default container name is `ros_sm`

    ```
    ./docker/docker.sh
    ```

    - Make sure to check that `docker/docker.sh` points to correct docker image. 
