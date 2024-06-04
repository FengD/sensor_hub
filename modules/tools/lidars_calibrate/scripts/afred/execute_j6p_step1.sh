# !/bin/bash
# params folder path
export CRDC_WS=./
source /install/setup.bash
source ../crdc_airi_common/share/sensor_msg/local_setup.bash
export LD_LIBRARY_PATH=../crdc_airi_common/lib/:$LD_LIBRARY_PATH
export VIN=j6p/810
export vehicle_type=j6p
export CALI_MODEL=manual
ros2 run beatles af_launcher . &
rviz2 -d rviz/init_j6p_cloud.rviz