# !/bin/bash
source /install/setup.bash
source ../../share/sensor_hub/local_setup.bash
source ../../../build/modules/crdc_airi_common/share/sensor_msg/local_setup.bash

input_arg1=$1 # path of raw bag folder

# revise raw bag name
python3 bag_to_image_convert.py $1raw/ 'replace_dash'

# generate metadata.yaml file
ros2 bag reindex $1raw/ sqlite3

# revise metadata.yaml
merged_bag_name=$(python3 bag_to_image_convert.py $1raw/metadata.yaml 'revise_yaml')

# create out.yaml file
python3 bag_to_image_convert.py $1 'create_out_yaml' $merged_bag_name

# merge bags
ros2 bag convert -i $1raw/ -o $1out.yaml

# delete metadata.yaml file
python3 bag_to_image_convert.py $1 'delete_yaml' $merged_bag_name

# convert to images
directory="../../databag_message_type_convertor/bin/"
cd "$directory"
export SAVE_PATH=$1img
./execute_jpeg_to_png_with_mask.sh $1$merged_bag_name

# align numbers of mask and image
cd "../../bag_to_image_to_video/bin/"
python3 bag_to_image_convert.py $1 'select_same' $merged_bag_name
















