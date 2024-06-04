# bag_to_image_to_video
Scripts in this folder are used to convert raw db3 bags to images, and then convert images to video.

# Running steps
## Step 1: convert bags to images
(1) Create a folder named "raw" in the path where the data is located, store raw db3 bags in the "raw" floder.
(2) Execute the following command in the terminal. In the command, data_folder_path is the path where data is located, e.g., /data_nas/data/jining/v2.3.0/20231206_1441/.
```
./bag_to_image_convert.sh data_folder_path
```

## Step 2: execute the post-processing program
(1) revise path info in fs_post_proc.prototxt.
(2) execute (compile first if not complied)

## Step 3: convert images (after post-processing) to video
Note: data_folder_path2 is the path of fs_res folder, which is added for only converting images to vedio.
```
./image_to_video_convert.sh data_folder_path data_folder_path2(optional)
```

