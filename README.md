# SENSOR_HUB

This project is named `sensor_hub` which could be used for sensor app layer.

## 1. BUILD

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
    check_code: check code qulity
    help: help menu
```

### 1.1. Multi-platform Support
`export PLATFORM=TDA4` for TDA4 camera input 
`export PLATFORM=A6` for A6 camera input 
`export PLATFORM=X86` for IPC camera input
# set env (Eg. TDA4 platform, A6, X86)
export PLATFORM=TDA4
# get dependencies(Eg, 0703,0702 for tda4, 1804,1604 for x86, "" for a6)
export TAG=0703

### 1.2. CodeCheck
``` shell
function check_code() {
    wget http://10.10.173.10/release/code_check_tools/code_check_tools.tar.gz
    tar -zxvf code_check_tools.tar.gz
    chmod -R +x code_check_tools
    export WORKSPACE=${WS}
    export CODE_CHECK_EXCLUDE_LIST="3rdparty"
    ./code_check_tools/code_check.sh run
    rm -rf code_check_tools.tar.gz code_check_tools
}
```

### 1.3. GetDependencies
``` shell
function get_dependencies() {
    echo "get-dependencies"
    mkdir -p build/modules
    wget http://10.10.173.10/share/pkgs/get-dependencies.py
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

## EXECUTE

``` shell

export CRDC_WS=
execute_camera_drivers.sh
execute_lidar_drivers.sh
execute_radar_drivers.sh
execute_imu_drivers.sh

```

## DEPENDENCIES
1. [crdc_airi_apollo_cyber](http://10.10.173.10/release/crdc_airi_apollo_cyber/)
2. [crdc_airi_common_lib](http://10.10.173.10/release/crdc_airi_common_lib/)
3. [tiovx_camera](http://10.10.173.10/release/tiovx_camera/)
* Details in [dependencies.yaml](dependencies.yaml)

```
packages:
  - name: crdc_airi_apollo_cyber
    url: http://10.10.173.10/release/crdc_airi_apollo_cyber/
    version: 69773acd93c88b33eab1ba2aff1af5bc629d87e3
  - name: crdc_airi_common_lib
    url: http://10.10.173.10/release/crdc_airi_common_lib/
    version: 0e86b49a547db1a7c00babc4ef486442fa24b120
  - name: tiovx_camera
    url: http://10.10.173.10/release/tiovx_camera/
    version: tiovx_camera
```
