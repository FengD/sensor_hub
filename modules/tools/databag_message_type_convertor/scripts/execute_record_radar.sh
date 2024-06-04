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

#set radar packet format. Optional: db3 or pcap
if [ -z ${PACKET_FORMAT} ];then
    export PACKET_FORMAT=pcap
    echo "Please set the  PACKET_FORMAT corresponding to the input file, otherwise the default will be used: ${PACKET_FORMAT}"
else
    echo "The PACKET_FORMAT is: ${PACKET_FORMAT}"
fi

#set the car number corresponding to the calibration parameter file
if [ -z ${VIN} ];then
    export VIN=default_vin
    echo "Please set the car VIN corresponding to the calibration parameter file, otherwise the default will be used: ${VIN}"
else
    echo "The VIN is: ${VIN}"
fi

#set convert source ip and udp
if [ -z ${SOURCE_IP} ];then
    export SOURCE_IP=10.20.30.40
    echo "Please set the SOURCE_IP, otherwise the default will be used: ${SOURCE_IP}"
else
    echo "The SOURCE_IP is: ${SOURCE_IP}"
fi

if [ -z ${UDP_PORT} ];then
    export UDP_PORT=6003
    echo "Please set the UDP_PORT, otherwise the default will be used: ${UDP_PORT}"
else
    echo "The UDP_PORT is: ${UDP_PORT}"
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
     --lidar_config_path params/drivers/radar/test/radar_config.prototxt
else
    echo "The input folder is: $1"
    ./databag_message_type_convertor --input_file $1 --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true \
     --lidar_config_path params/drivers/radar/test/radar_config.prototxt
fi