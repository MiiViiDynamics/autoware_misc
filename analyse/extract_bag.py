#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.time import Time
from rclpy.serialization import deserialize_message
import importlib.util
import configparser
import math
import pandas as pd
import os
from datetime import datetime

def get_message_class(message_type):
    """Dynamically loads the message class from its type string."""
    package, module, msgname = message_type.split('/')
    mod = importlib.import_module(f'{package}.{module}')
    return getattr(mod, msgname)

def create_config(topic_types, start_time, end_time, config_file_path):
    """Creates and saves a configuration file with topics and time range."""
    config = configparser.ConfigParser()
    for i, topic in enumerate(topic_types):
        section_name = f"topic{i}"
        config[section_name] = {
            'topic_name': topic.name,
            'type': topic.type,
            'freq': '',
            'first_offset': -0.1,
        }
    
    config['Time'] = {
        'Start': str(math.floor(start_time) + 1),
        'End': str(math.floor(end_time) - 1)
    }

    with open(config_file_path, 'w') as configfile:
        config.write(configfile)
    return config

def extract_timestamps(reader, config, output_dir):
    """Extracts timestamps from the bag and writes them to CSV files."""
    timestamps = {topic['topic_name']: [] for _, topic in config.items() if topic.get('topic_name')}
    
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg_type = next((t.type for t in reader.get_all_topics_and_types() if t.name == topic), None)
        
        if msg_type:
            msg_class = get_message_class(msg_type)
            msg = deserialize_message(data, msg_class)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # if int(config['Time']['Start']) <= timestamp < int(config['Time']['End']):
            timestamps[topic].append(timestamp)
    
    for topic, ts in timestamps.items():
        df = pd.DataFrame({'Timestamp': ts})
        filename = os.path.join(output_dir, f"{topic.replace('/', '_')}_timestamps.csv")
        df.to_csv(filename, index=False)

def main(bag_path, config_file_path):
    """Main function to process the ROS2 bag and generate CSV files with timestamps."""
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    metadata = reader.get_metadata()
    start_time = metadata.starting_time.to_msg().sec + metadata.starting_time.to_msg().nanosec * 1e-9
    end_time = start_time + (metadata.duration.to_msg().sec + metadata.duration.to_msg().nanosec * 1e-9)

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"csv_{current_time}"
    os.makedirs(output_dir, exist_ok=True)

    config = create_config(reader.get_all_topics_and_types(), start_time, end_time, config_file_path)
    extract_timestamps(reader, config, output_dir)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 extract_bag.py <path_to_bag_file> <path_to_config_file>")
        sys.exit(1)

    bag_path = sys.argv[1]
    config_file_path = sys.argv[2]
    main(bag_path, config_file_path)