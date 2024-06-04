# !/bin/bash
# params folder path
export CRDC_WS=./
source ../../share/sensor_hub/local_setup.bash
source /install/setup.bash
 
#set camera_h264_record convert to camera_raw_record.  Optional: default or record
export RECORD=default

#set camera_h264_record extract to single frame nv12 image
export TYPE=yuv

#set the car number corresponding to the calibration parameter file
export VIN=default_vin
echo "The VIN is: ${VIN}"

# output_data  save path. default folder: .
if [ -z ${SAVE_PATH} ];then
    export SAVE_PATH=.
    echo "Please set the output_file SAVE_PATH, otherwise the default folder will be used: ${SAVE_PATH}"
else
    echo "The output_file SAVE_PATH is: ${SAVE_PATH}"
fi

# $1 is input_data folder path. default folder: data_packet/
if [ -z $1 ];then
    echo "No input folder given. Use the default: data_packet/"
    ./databag_message_type_convertor --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true \
    --camera_config_path params/drivers/camera/HH02-8/camera_config.prototxt
else
    echo "The input folder is: $1"
    ./databag_message_type_convertor --input_file $1 --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true \
    --camera_config_path params/drivers/camera/HH02-8/camera_config.prototxt
fi
