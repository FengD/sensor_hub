# !/bin/bash
# params folder path
export CRDC_WS=../

#The lidar pcd data type, FORMAT Optional: ASCII or Binary
export FORMAT=ASCII

#The camera image type, TYPE Optional: png or jpg
export TYPE=jpg

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
    ./databag_message_data_extractor --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true
else
    echo "The input folder is: $1"
    ./databag_message_data_extractor --input_file $1 --alsologtostderr true --stderrthreshold 3 --v 0 --minloglevel 3 --colorlogtostderr true
fi
