# v3.2.0 (20240229) feng.ding
1. (camera_calib): increase the number of calibration points to 8
2. (camera_calib): update circle center extraction algorithm
3. (lidar calibration): fix issue that lidar_drivers cannot run
4. (camera_calib): input extrinsic params and regenerate luts
5. (camera_drivers): Fix the timestamps of ros2 input

# v3.1.0 (20231214) feng.ding
1. (radar):adaptive radar point cloud transformation function
2. (tools): median filtering removes noise in the ld mask
3. (tools): align the timestamps of mask and image
4. (tools):add scripts for converting bags to images and videos
5. (lidar diagnose): add pandar_xt32 lidar packet loss diagnosis
6. (camera input): add db3 input format for camera
7. (data_extractor): add aduinfo module
8. (calibration): remove the logic of copying calibration parameter files

# v2.15.0 (20230918) feng.ding
1. (calibrate): add lidar dynamic calibrate module
2. (camera_drivers): switch input init from main thread to Camera thread avoid ANR
3. (tiovx_camera): add signal processing to deinit when crashed
4. (tools): add more color display to the mask of Jingtang Port Lane line
5. (radar): add radar continuously control and and timing 

# v2.14.0 (20230619) feng.ding
1. (chassis): add chassis drivers
2. (calibration service): add calibration server and test client
3. (someip_v210_hav30): add someip_v210_hav30 and chassis of hav3g
4. (type_convertor):modify raw images encoding type
5. (gstcamera): fixed gst cannot run under afred of 20.04 ubuntu
6. (camera_drivers): adapted to new tiovx_camera interfaces for self-developed camera
7. (gst-camera): make gst-launch params configurable
8. (radar): fix bug about determining frame end
9. (lidar calibrate): add the calibrate module for x86_AFRED
10. (glog): save lidar_drivers and camera_drivers logs of ERROR and above for 15 days
11. (data_collect): modify HAV02&HAV3G storage capacity to 320G

# v2.13.0 (20230421) feng.ding
1. (data_convertor): add function of save fs&ld visu_perception to new ros2bag
2. (radar): Modified the order of RadarPoint structure fields to adapt to traditional algorithms
3. (data_play_scripts): add lidar data_play_scripts in the tool module
4. (type_convertor): add function about decoding jpeg image to raw image
5. (camera_driver): send encoded image by 'sensor_msgs::CompressedImage'
6. (data_convertor):integrate h264_to_yuv and jpeg_to_yuv into type_convertor
7. (tools):data_extractor supports both custom and ROS2 msg
8. (type_convertor):modify ld msg type to marker
9. (data_collect):unified save time to 20s
10. (lidar calibrate):accurate calibrate x and y based on previous calibration

# v2.12.0 (20230320) feng.ding
1. (tools): optimize tools with ros messages
2. (tools): integrate data collection in one script
3. (config): correct FL and RR lidar destination port
4. (config): read angle filter params from calibration files
5. (tools): optimize extract_frame and h264_to_yuv tools
6. (radar): add snr message
7. (lidar_driver/camera_driver): use ros msgs instead of user-defined msg
8. (tools): reorganize camera data collecting tools
9. (radar):add frame end judgment basis

# v2.11.0 (20230203) feng.ding
1. (ins): parse INS data from pcap file
2. (tools): Improved camera data collect scripts
3. (socket): add multicast_ip method of socketopt
4. (lidar_driver): make lidar frequency changable
5. (params): add pcap data transform to record params
6. (radar_driver):add radar_driver module
7. (robosense):modify coordonate system
8. (lidar): correct pcap reading cannot finish

# v2.10.0 (20221214) feng.ding
1. (lidar&camera): add afred startup config
2. (tools): use openmp for extract frames
3. (lidar&camera): use product name as the input config, and rename the config file as HH02-XX, HE00-XX
4. (camera): add data collect tools for hla
5. (all): update the directory structure

# v2.9.0 (20221202) feng.ding
1. (tools): transform 2D and 3D ground truth from apollo to sensor
2. (pcapinput): pcap file offline replay
3. (tool): transform apollo message to sensor message
4. (robosense):add parser for RS-Ruby-80V
5. (camera): modify topic name
6. (parser): modify PointField arrange of intensity
7. (tools): optimizing extract frame function
8. (innovizone): optimize parse module
9. (tools): add h264 data convert to png
10. (ci): integrate images of both 0703_HI and 0703_HLA

# v2.8.0 (20221027) feng.ding
1. (diag): add diag of empty cloud as DeviceStatus::DEVICE_ERROR
2. (tools): add fs and bbox channel data in write packet function
3. (tools): add h264 record data translate to yuv data
4. (build): adapt for afred, add TAG AFRED
5. (ci): fix unit test statistic errors. exclude files in common lib
6. (ci): add cov html report in artifacts one day valid
7. (gtest): add unit test to lidar_drivers and camera_drivers
8. (params): modify hav and E2HPC lidar_drivers params to fit MANIFEST

# v2.7.0 (20220804) feng.ding
1. (v2x): add v2x related config files
2. (tiovx_camera): change the dependency of tiovx camera input
3. (tool): optimize write packet function and add cloud transform function
4. (unit_test): add unit test in all drivers
5. (lidar_drivers): modify mktime to conversion_time
6. (MANIFEST): add level to limit output log size

# v2.6.0 (20220714) feng.ding
1. (dependencies): update version
2. (ci): modify image path
1. (parser): use system time if sensor time is invalid
2. (lidar_drivers): modify socket poll_timeout time period to 300ms

# v2.5.0 (20220623) feng.ding
1. (lidar): modify lidars prototxt of hpc
2. (ci): delete build of a6
3. (lidar): add velodyne VLP16 and lanhai_lds50cs driver
4. (camera): add compression type H264
5. add lidar and camera config params for HAV

# v2.4.0 (20220523) feng.ding
1. (lidar): add lidar parser of innoviz_one, pandar_p40 and robosense_rubylite
2. (lidar): add communcation diagnosis function
3. (camera): add h264 encoder

# v2.3.1 (20220324) feng.ding
1. (lidar) add tai time offset to pandarXT;
2. (lidar) remove merge cloud action if lidar merge could channel not give
3. (ci): add deploy for each commit

# v2.3.0 (20220323) feng.ding
1. add extract tools

# v1.7.9 (20220210) feng.ding
1. correct some spell errors.

# v1.7.8 (20220209) feng.ding
1. Add parameters for tda4 hpc2&hpc3.

# v1.7.0 (20220107) feng.ding
1. Add CyberInput for lidar_driver, the raw_data fps should be 100.

# v1.6.0 (20211214) feng.ding
1. Change to dependencies build style.

# v1.5.0 (20211208) feng.ding
1. Correct gstreamer bug error.
2. Add build for tda4platform.
3. Rename openvx camera.

# v1.4.0 (20211202) feng.ding
1. Add A6 camera drivers
2. Add encoder for A6 and tda4 cameras

# v1.3.2 (20211114) feng.ding
1. Add tiovx input for camera on tda4 platform

# v1.3.1 (20211112) feng.ding
1. Add gstreamer input for camera

# v1.3.0 (20211106) feng.ding
1. Create Ins drivers
2. Build on three platform
