# !/bin/bash
# params folder path
export CRDC_WS=./
source ../../share/sensor_hub/local_setup.bash
source /install/setup.bash

#Is packet or not
#The point cloud is transformed by calibration parameters or not
#Optional: yes or no
export ispacket=yes
export TF=yes

#set point_raw_record convert to point_cloud_record.  Optional: default or record
export RECORD=record

#set the car number corresponding to the calibration parameter file
if [ -z ${VIN} ];then
    export VIN=v2x/JN/A0102I
    echo "Please set the car VIN corresponding to the calibration parameter file, otherwise the default will be used: ${VIN}"
else
    echo "The VIN is: ${VIN}"
fi

# output_data  save path. default folder: .
if [ -z ${SAVE_PATH} ];then
    export SAVE_PATH=.
    echo "Please set the output_file SAVE_PATH, otherwise the default folder will be used: ${SAVE_PATH}"
else
    echo "The output_file SAVE_PATH is: ${SAVE_PATH}"
fi

# $1 is input_data folder path. default folder: data_packet/
if [ -z $1 ];then
    echo "No input_file given. Use the default: data_packet/"
    ./databag_message_type_convertor --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true \
     --lidar_config_path params/drivers/lidar/v2x/x86/lidar_config.prototxt
else
    echo "The input folder is: $1"
    ./databag_message_type_convertor --input_file $1 --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true \
     --lidar_config_path params/drivers/lidar/v2x/x86/lidar_config.prototxt
fi
