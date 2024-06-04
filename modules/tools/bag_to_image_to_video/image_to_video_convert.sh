# !/bin/bash
source /install/setup.bash

input_arg1=$1 # path of raw bag folder

if [ $# -eq 1 ]; then
    # 输入第二个参数，第二个参数为fs_res所在目录
    merged_bag_name=$(python3 bag_to_image_convert.py $1out.yaml 'read_yaml')

    # generate video
    python3 bag_to_image_convert.py $1 'generate_video' $merged_bag_name

    # convert format
    ffmpeg -i $1img/"$merged_bag_name"_0/res.mp4 -pix_fmt yuv420p -c:a copy -movflags +faststart $1img/"$merged_bag_name"_0/res_format_converted.mp4

    video_file=$1img/"$merged_bag_name"_0/res_format_converted.mp4
    output_file=$1img/"$merged_bag_name"_0/res_format_converted_smaller_size.mp4
elif [ $# -eq 2 ]; then
    # $2: path of fs_res folder
    # generate video
    python3 bag_to_image_convert.py $2 'generate_video_dir'

    # convert format
    ffmpeg -i $2res.mp4 -pix_fmt yuv420p -c:a copy -movflags +faststart $2res_format_converted.mp4

    video_file=$2res_format_converted.mp4
    output_file=$2res_format_converted_smaller_size.mp4
else
    echo "Wrong input arguments number!"
fi
    
# change size
max_size=100000000
file_info=$(ffprobe -v error -select_streams v:0 -show_entries format=size -of default=noprint_wrappers=1:nokey=1 "$video_file")
file_size=$((file_info))
if [[ $file_size -le $max_size ]]; then
    echo "The file size is already within the limit. No compression needed."
    exit 0
fi

width=$(ffprobe -v error -select_streams v:0 -show_entries stream=width -of csv=p=0 "$video_file")
height=$(ffprobe -v error -select_streams v:0 -show_entries stream=height -of csv=p=0 "$video_file")
new_width=$width
new_height=$height
while [[ $file_size -gt $max_size ]]; do
    if [ -f "$output_file" ]; then
        rm "$output_file"
    fi
    new_width=$((new_width - width * 1 / 8))
    new_height=$((new_height - height * 1 / 8))
    ffmpeg -i "$video_file" -vf "scale=$new_width:$new_height" "$output_file"
    file_size=$(stat -c %s "$output_file")
    new_width=$(ffprobe -v error -select_streams v:0 -show_entries stream=width -of csv=p=0 "$output_file")
    new_height=$(ffprobe -v error -select_streams v:0 -show_entries stream=height -of csv=p=0 "$output_file")
done

echo "Compression complete. Output file size is now within the limit."




















