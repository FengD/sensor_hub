# SENSOR_HUB

## Description

* This project is named `sensor_hub` which could be used for sensor app layer.

``` mindmap

# SensorHub
## CameraDriver
### Input
#### Gstreamer
#### Mxc
#### ROS2Topic
#### Openvx/tiovx 
### Decoder
### Encoder
#### opencv
#### h264
#### turbo_jpeg
## LidarDriver
### Input
#### Cyber
#### PCAP
#### ROS2
#### Socket
### Parser
#### Innovusion-Falcon
#### Innoviz-pro/one
#### Hesai-pandar40,XT32
#### Robosense-Ruby
#### Velodyne-16
#### Arbe(Imaging radar cloud)
## RadarDriver
## ChassisDriver
## InsDriver

```

## 1. Build

* Set env for different platform
`export PLATFORM=X86`  `export TAG=1804`  `export TAG=AFRED` for X86
`export PLATFORM=TDA4` `export TAG=0703_HI`  `export TAG=AFRED` for TDA4

`./build.sh` build the project.
`./build.sh clean` clean the build.
run `./build.sh help` to see the details.

### 1.1. help menu
./build.sh help

```
Usage:
    ./build.sh [OPTION]

Options:
    all: run the code style check
    build: run the code build
    clean: clean the code build
    cov: run the code test coverage
    check_code [param]: check code qulity param 'help' for check_code param list
```


### 1.2. CodeCheck
* use `./build.sh check_code run` first to verify if all check passed!

### 1.3. Coverage check
``` shell
function gen_coverage() {
  export LD_LIBRARY_PATH=${WS}/build/modules/crdc_airi_common/lib/
  cd ${WS}
  rm -rf cov
  mkdir -p cov
  lcov -d ./build -z
  cd build
  ctest
  cd ${WS}
  lcov -d ./build -b modules --no-external --rc lcov_branch_coverage=1  -c -o cov/raw_coverage.info
  lcov -r cov/raw_coverage.info "*test*" -o cov/drivers.info --rc lcov_branch_coverage=1 'build/*'
}
```

## Release
* If the build successed, all the release are in the `build_dist` folder.

## EXECUTE

``` shell
export CRDC_WS=
execute_camera_drivers.sh
execute_lidar_drivers.sh
execute_radar_drivers.sh
execute_imu_drivers.sh

```

