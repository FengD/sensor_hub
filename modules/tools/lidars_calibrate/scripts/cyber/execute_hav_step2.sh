# !/bin/bash
# params folder path
export CYBER_PATH=../../crdc_airi_common/apollo/cyber
export LD_LIBRARY_PATH=../../crdc_airi_common/lib
export CRDC_WS=../
export VIN=hav/600
export CALI_MODEL=manual
export vehicle_type=hav
./lidar_calibrate --config_file params/hav/lidar_config.prototxt


