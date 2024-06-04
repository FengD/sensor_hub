#!/usr/bin/env bash
set -e
declare -A HeaderMap=(["HE00-1"]="V200_J6P10" ["HE00-2"]="V200_J6P10" ["HE00-3"]="V200_J6P10"
                      ["HH02-1"]="V201_HAV02" ["HH02-2"]="V201_HAV02" ["HH02-5"]="V201_HAV02"
                      ["HH02-6"]="V201_HAV02" ["HH02-7"]="V201_HAV02" ["HH02-8"]="V201_HAV02"
                      ["HH03-3"]="V210_HAV30" ["HH03-4"]="V210_HAV30" ["HH03-5"]="V210_HAV30" ["HH03-6"]="V210_HAV30"
		              ["HR00-1"]="V2X" ["HR00-2"]="V2X")
declare -A PathMap=(["HE00-1"]="Camera5" ["HE00-2"]="LiDar2" ["HE00-3"]="LiDar3"
                    ["HH02-1"]="LR_FR" ["HH02-2"]="LR_FL" ["HH02-5"]="LR_RL"
                    ["HH02-6"]="LR_RR" ["HH02-7"]="CameraR" ["HH02-8"]="CameraF"
                    ["HH03-3"]="LR_FL" ["HH03-4"]="CameraF" ["HH03-5"]="LR_RR" ["HH03-6"]="CameraR"
		            ["HR00-1"]="Camera" ["HR00-2"]="Camera")
declare -A IDMap=(["HE00-1"]="001" ["HE00-2"]="002" ["HE00-3"]="003"
                  ["HH02-1"]="001" ["HH02-2"]="002" ["HH02-5"]="005"
                  ["HH02-6"]="006" ["HH02-7"]="007" ["HH02-8"]="008"
                  ["HH03-3"]="003" ["HH03-4"]="004" ["HH03-5"]="005" ["HH03-6"]="006"
		          ["HR00-1"]="001" ["HR00-2"]="002")
declare -A SizeMap=(["HE00-1"]=209715200 ["HE00-2"]=283115520 ["HE00-3"]=283115520
                    ["HH02-1"]=335544320 ["HH02-2"]=335544320 ["HH02-5"]=335544320
                    ["HH02-6"]=335544320 ["HH02-7"]=335544320 ["HH02-8"]=335544320
                    ["HH03-3"]=335544320 ["HH03-4"]=335544320 ["HH03-5"]=335544320 ["HH03-6"]=335544320
		            ["HR00-1"]=335544320 ["HR00-2"]=335544320)
product_num=`getprop ro.boot.product`
vehicle_num=`getprop persist.product.vehiclenum`
savetime=20
sleeptime=0.1

if [ "HE00-1" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V200_J6P10_Camera5_CV_ENCODER_0 /hav_fs_visu_perception /hav_ld_visu_perception"
elif [ "HE00-2"  ==  ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_RIGHT_RAW /lidar/LIDAR_UPPER_RAW /detect_visu
        /track_visu /freespace_visu /boundary_visu /signboard_visu /bridge_visu  /LIDAR_DIAGNOSE"
elif [ "HE00-3" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_LEFT_RAW /lidar/LIDAR_REAR_RAW /detect_visu
        /track_visu /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE"
elif [ "HH02-1" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_FR_RAW /detect_visu /track_visu /detect_confidence
        /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE"
elif [ "HH02-2"  ==  ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_FL_RAW /detect_visu /track_visu /detect_confidence
        /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE"
elif [ "HH02-5" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_RL_RAW /detect_visu /track_visu /detect_confidence
        /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE"
elif [ "HH02-6" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_RR_RAW /detect_visu /track_visu /detect_confidence
        /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE"
elif [ "HH02-7" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V201_HAV02_CameraR_CV_ENCODER_0
        /hav_fs_visu_perception /hav_ld_visu_perception /hav_ld_mask /hav_fs_mask /CAMERA_DIAGNOSE"
elif [ "HH02-8" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V201_HAV02_CameraF_CV_ENCODER_0
        /hav_fs_visu_perception /hav_ld_visu_perception /hav_ld_mask /hav_fs_mask /CAMERA_DIAGNOSE"
elif [ "HH03-3" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_FL_RAW /detect_visu /track_visu /detect_confidence
        /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE /TboxInfo /ADUInfo"
elif [ "HH03-4" == ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V210_HAV30_CameraF_CV_ENCODER_0
        /hav_fs_visu_perception /hav_ld_visu_perception /hav_ld_mask /hav_fs_mask /CAMERA_DIAGNOSE"
elif [ "HH03-5"  ==  ${product_num} ]; then
    RECORD_OPTIONS="${RECORD_OPTIONS} /lidar/LIDAR_RR_RAW /detect_visu /track_visu /detect_confidence
        /freespace_visu /boundary_visu /signboard_visu /LIDAR_DIAGNOSE /TboxInfo /ADUInfo"
elif [ "HR00-1" == ${product_num} ]; then
    vehicle_num=${product_num}
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V2X_JN_FPU01_Camera_CV_ENCODER_0"
elif [ "HR00-2" == ${product_num} ]; then
    vehicle_num=${product_num}
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V2X_JN_FPU02_Camera_CV_ENCODER_0"
else
    RECORD_OPTIONS="${RECORD_OPTIONS} /camera/V210_HAV30_CameraR_CV_ENCODER_0
        /hav_fs_visu_perception /hav_ld_visu_perception /hav_ld_mask /hav_fs_mask /CAMERA_DIAGNOSE"
fi

mkdir -p /extdata/${PathMap[${product_num}]}/
data_dir="/extdata/${PathMap[${product_num}]}/"
nameheader="${HeaderMap[${product_num}]}"

temp_dir1="/extdata/af_tempdata_1/"
temp_dir2="/extdata/af_tempdata_2/"
if [ -d $temp_dir1 ]; then
    mv ${temp_dir1}/*.db3 ${data_dir}/
    rm -rf ${temp_dir1}
    temp_dir=${temp_dir2}
elif [ -d $temp_dir2 ]; then
    mv ${temp_dir2}/*.db3 ${data_dir}/
    rm -rf ${temp_dir2}
    temp_dir=${temp_dir1}
else
    temp_dir=${temp_dir1}
fi

bag_dir="${nameheader}_${vehicle_num}-${IDMap[${product_num}]}-${PathMap[${product_num}]}"
ros2 bag record ${RECORD_OPTIONS} -t -d ${savetime} -n ${bag_dir} -o ${temp_dir} &
pid=$!

while true
do
    find ${data_dir} -name "${nameheader}*" -type d | xargs rm -rf
    find ${data_dir} -name "metadata*" -type f | xargs rm -rf
    total_size=`du --max-depth=0 $data_dir | awk '{print $1}'`
    while [ $total_size -ge ${SizeMap[${product_num}]} ]
    do
        to_rm_file=`ls ${data_dir} -tr | head -1`
        rm -r ${data_dir}/${to_rm_file}
        total_size=`du --max-depth=0 $data_dir | awk '{print $1}'`
    done
    sleep ${savetime}
    total_num=`ls ${temp_dir} -l | grep "^-" | wc -l`
    while [ ${total_num} -ge 2 ]
    do
        to_mv_file=`ls ${temp_dir} -tr | head -1`
        mv ${temp_dir}/${to_mv_file} ${data_dir}/
        total_num=`ls ${temp_dir} -l | grep "^-" | wc -l`
    done
    if ! kill -0 $pid 2>/dev/null;then
        exit 1
    fi
done
