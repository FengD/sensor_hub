import os 
import sys
import yaml
import shutil
import cv2

def replace_dash(folder_path):
    for filename in os.listdir(folder_path):
        if os.path.isfile(os.path.join(folder_path, filename)):
            # only process files, not including folders
            new_filename = filename.replace('-', '_')
            
            # revise filename
            try:
                os.rename(os.path.join(folder_path, filename), os.path.join(folder_path, new_filename))
                print(f"Filename has been revised to {new_filename}")
            except FileNotFoundError:
                print(f"Cannot find file {filename}")
            except FileExistError:
                print(f"File {new_filename} already exists!")
            except Exception as e:
                print(f"Error: {e}")

def revise_yaml(yaml_name):
    with open(yaml_name, 'r') as file:
        data = yaml.safe_load(file)
    data_files = data['rosbag2_bagfile_information']['relative_file_paths']
    if 'CameraF_' in data_files[0]:
        sorted_files = sorted(data_files, key=lambda x: x.split('CameraF_')[1])
    else:
        sorted_files = sorted(data_files, key=lambda x: x.split('CameraR_')[1])
    sorted_files.insert(0, sorted_files[0])
    data['rosbag2_bagfile_information']['relative_file_paths'] = sorted_files
    with open(yaml_name, 'w') as file:
        yaml.dump(data, file)
    return sorted_files[0][:-11]

def create_out_yaml(yaml_name, file_name):
    
    data = {
        "output_bags": [
            {
                "uri": file_name, 
                "all": True, 
                "storage_id":"sqlite3"
            }
        ]
    }
    with open(yaml_name, 'w') as file:
        yaml.dump(data, file)

def select_same(filepath1, filepath2, output_path1, output_path2):
    files_folder1 = os.listdir(filepath1)
    files_folder2 = os.listdir(filepath2)

    duplicate_files = set(files_folder1).intersection(files_folder2)

    for file_name in duplicate_files:
        file_path_folder1 = os.path.join(filepath1, file_name)
        file_path_folder2 = os.path.join(filepath2, file_name)
        output_file_path1 = os.path.join(output_path1, file_name)
        output_file_path2 = os.path.join(output_path2, file_name)

        shutil.copy(file_path_folder1, output_file_path1)
        shutil.copy(file_path_folder2, output_file_path2)

def images_to_video(image_folder, video_name):
    images = sorted([img for img in os.listdir(image_folder) if img.endswith(".jpg") or img.endswith(".png")], key=lambda x: int(x.split('.')[0]))
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()
    

if __name__ == "__main__":
    if sys.argv[2] == 'replace_dash':
        rawBagFile_folder_path = sys.argv[1]
        replace_dash(rawBagFile_folder_path)
    if sys.argv[2] == 'revise_yaml':
        yaml_name = sys.argv[1]
        merged_bag_name = revise_yaml(yaml_name)
        print(merged_bag_name)
    if sys.argv[2] == 'create_out_yaml':
        yaml_name = sys.argv[1] + 'out.yaml'
        file_name = sys.argv[1] + sys.argv[3]
        create_out_yaml(yaml_name, file_name)
        # print(os.listdir(sys.argv[1] + 'raw/')[0][:-11])
    if sys.argv[2] == 'delete_yaml':
        file_name = sys.argv[1] + sys.argv[3]
        print(file_name)
        os.remove(file_name + '/' + 'metadata.yaml')
    if sys.argv[2] == 'select_same':
        folder_path = sys.argv[1] + 'img/' + sys.argv[3] + '_0'
        filepath2 = folder_path + '/hav_fs_mask' #folder path for masks
        parent_directory = folder_path + '/camera'
        items = os.listdir(parent_directory)
        subfolder_path = None
        for item in items:
            filepath1 = os.path.join(parent_directory, item)#folder path for images
            if os.path.isdir(filepath1):
                if subfolder_path is not None:
                    print('Not only include one folder')
                    break
                subfolder_path = filepath1
        output_path1 = folder_path + '/image' # new folder path for images
        output_path2 = folder_path + '/fs_mask' # new folder path for masks
        if not os.path.exists(output_path1):
            os.makedirs(output_path1)
        if not os.path.exists(output_path2):
            os.makedirs(output_path2)
        select_same(filepath1, filepath2, output_path1, output_path2)
    if sys.argv[2] == 'generate_video':
        image_folder = sys.argv[1] + 'img/' + sys.argv[3] + '_0/' + 'fs_res'
        video_name = sys.argv[1] + 'img/' + sys.argv[3] + '_0/' + 'res.mp4'
        images_to_video(image_folder, video_name)
    if sys.argv[2] == 'generate_video_dir':
        image_folder = sys.argv[1] + 'fs_res'
        video_name = sys.argv[1] + 'res.mp4'
        images_to_video(image_folder, video_name)
    if sys.argv[2] == 'read_yaml':
        filename = sys.argv[1]
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        name = data['output_bags'][0]['uri']
        name_last = name.rsplit('/', 1)[-1]
        print(name_last)

    



