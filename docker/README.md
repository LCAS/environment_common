
Launch the ci to autofill files
```sh
function environment_common_ci () {
    cd environment_common/docker
    export ENVIRONMENT_TEMPLATE=$HOME/ros2_ws/src/environment_template
    docker compose up
}
```

