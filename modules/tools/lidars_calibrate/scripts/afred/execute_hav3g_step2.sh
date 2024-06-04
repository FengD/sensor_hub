# !/bin/bash
# params folder path
export CRDC_WS=./
source /install/setup.bash
source ../crdc_airi_common/share/sensor_msg/local_setup.bash
export LD_LIBRARY_PATH=../crdc_airi_common/lib/:$LD_LIBRARY_PATH
export VIN=hav/600
export vehicle_type=hav3g
export CALI_MODEL=auto
ros2 run beatles af_launcher . &
rviz2 -d rviz/tf_hav3g_cloud.rviz
