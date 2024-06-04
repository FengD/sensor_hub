# !/bin/bash
# params folder path
export CYBER_PATH=../../crdc_airi_common/apollo/cyber
export LD_LIBRARY_PATH=../../crdc_airi_common/lib
export CRDC_WS=../
export VIN=j6p/810
export CALI_MODEL=manual
export vehicle_type=j6p
./lidar_calibrate --config_file params/j6p/lidar_config.prototxt
