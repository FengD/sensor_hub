import os
import io
import json
import sys
WEATHER_PATH = "./"

def generate_weather(packet):
    weather = "Sunny"
    port = packet.split("_")[1]
    weather_list_path = os.path.join(WEATHER_PATH, "weather_of_" + port + ".json")
    with io.open(weather_list_path, "r", encoding="utf-8") as w:
        weather_list = json.load(w)
        for date in weather_list.keys():
            if date in packet.replace("_", "-"):
                weather = weather_list[date]
    return weather

def generate_raw_data_type(packet_path):
    raw_data_type = " "
    if os.path.exists(packet_path):
        raw_list = os.listdir(packet_path)
        type = []
        for raw in raw_list:
            if raw == "pcap" and len(os.listdir(os.path.join(packet_path, raw))) > 0:
                type.append(raw)   
        if len(type) == 1 and os.path.exists(os.path.join(packet_path, "pcap")):
            raw_data_type = "Pcap"
    return raw_data_type

def generate_time_and_time_interval(packet):
    year = packet.split("_")[2]
    month = packet.split("_")[3]
    day = packet.split("_")[4]
    hour = packet.split("_")[5]
    minute = packet.split("_")[6]
    sec = packet.split("_")[7]
    time = year + "-" + month + "-" + day + " " + hour + ":" + minute + ":" + sec
    time_interval = ""
    hour = int(hour)
    if hour >= 7 and hour < 11:
        time_interval = "Morning"
    if hour >= 11 and hour < 13:
        time_interval = "Noon"
    if hour >= 13 and hour < 18:
        time_interval = "Afternoon"
    if hour >= 18 and hour < 21:
        time_interval = "Evening"
    if hour >= 21 and hour < 24:
        time_interval = "Night"
    if hour < 4:
        time_interval = "Eve"
    if hour >= 4 and hour < 7:
        time_interval = "Daybreak"
    return time, time_interval

def generate_scene_picture_link(path):
    for root, dirs, files in os.walk(path):
        if root != path:
            continue
        for dir in dirs:
            packet = os.path.basename(dir)
            if 'port' not in dir:
                continue
            time, time_interval = generate_time_and_time_interval(packet)
            weather=generate_weather(packet)
            packet_path=os.path.join(path,dir, "raw_data")
            raw_data_type=generate_raw_data_type(packet_path)
            if raw_data_type.isspace():
                continue
            print time_interval,"                   ",weather,"                   ",dir
        sys.exit()    
    return "None"
generate_scene_picture_link(r"/data/ai_data/collect_data/dataset")