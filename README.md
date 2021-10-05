# SENSOR_HUB

This project is named `sensor_hub` which could be used for sensor app layer.

## BUILD

``` shell
# build 
./build.sh

# build help menu
./build.sh help

Usage:
    ./build.sh [OPTION]

Options:
    all: run the code style check
    build: run the code build
    clean: clean the code build
    cov: run the code test coverage

```

## EXECUTE

``` shell

export CRDC_WS=

execute_camera_drivers.sh
execute_lidar_drivers.sh
execute_radar_drivers.sh
execute_imu_drivers.sh

```

## DEPENDENCIES
1. [cyber]()
2. [opencv]() for camera