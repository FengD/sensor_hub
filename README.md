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
``` shell
function check_code() {
    wget http://airi-server-4010.hirain.local/release/code_check_tools/code_check_tools.tar.gz
    tar -zxvf code_check_tools.tar.gz
    chmod -R +x code_check_tools
    export WORKSPACE=${WS}
    export CODE_CHECK_EXCLUDE_LIST="3rdparty,tools,gstcamera,test"
    echo -e "${RED}exclude list: $CODE_CHECK_EXCLUDE_LIST ${NO_COLOR}"
    ./code_check_tools/code_check.sh $param
    if [ $? -eq 0 ]; then
        rm -rf code_check_tools.tar.gz code_check_tools
        exit 0
    else
        rm -rf code_check_tools.tar.gz code_check_tools
        exit 1
    fi
}
```

### 1.3. GetDependencies
``` shell
function get_dependencies() {
    echo "get-dependencies"
    mkdir -p build/modules
    wget http://airi-server-4010.hirain.local/share/pkgs/get-dependencies.py
    python get-dependencies.py
    rm -rf get-dependencies.py
}
```

All the dependencies will extract into `build/modules/`. So it is better to put all the source in `modules` folder. And then,
* `set(DEPENDENCIES_PATH ${CMAKE_CURRENT_BINARY_DIR}/modules)` in the top CMakeLists.txt
* Put use for `link_directories` and `include_directories` in the CMakeLists.txt inside the `modules`

```
include_directories(
    ${DEPENDENCIES_PATH}/apollo/cyber/include
    ${DEPENDENCIES_PATH}/crdc_airi_common/include
    ${DEPENDENCIES_PATH}/tiovx_camera/include
)

link_directories(
    ${DEPENDENCIES_PATH}/apollo/cyber/lib
    ${DEPENDENCIES_PATH}/crdc_airi_common/lib
    ${DEPENDENCIES_PATH}/tiovx_camera/lib
)
```

### 1.4. Coverage check
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

