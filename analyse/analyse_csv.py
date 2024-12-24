#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys
import pandas as pd
import os
import configparser
import numpy as np

def print_colored(text, color_code):
    print(f"{color_code}{text}\033[0m")
    

def read_timestamps_from_csv(config, csv_dir):
    """Reads timestamp CSV files based on the topic names in the configuration and stores them in a dictionary."""
    timestamps_dicts = {}

    # 遍历配置文件中的所有部分
    for section in config.sections():
        if section.startswith('topic'):
            try:
                topic_name = config.get(section, 'topic_name')
                freq_str = config.get(section, 'freq', fallback=None)
                first_offset = config.getfloat(section, 'first_offset', fallback=0.0)

                if freq_str is None or not freq_str.strip().isdigit():
                    raise ValueError(f"Missing or invalid 'freq' value in section '{section}'")

                freq = int(freq_str)

                # 构建CSV文件名
                csv_filename = f"{topic_name.replace('/', '_')}_timestamps.csv"
                file_path = os.path.join(csv_dir, csv_filename)

                # 检查文件是否存在
                if not os.path.exists(file_path):
                    print_colored(f"Warning: {file_path} does not exist. Skipping this topic.", "\033[91m") 
                    continue

                # 读取CSV文件到DataFrame
                df = pd.read_csv(file_path)

                # 将时间戳列转换为列表并存储到字典中
                timestamps_dicts[topic_name] = {
                    'timestamps': df['Timestamp'].tolist(),
                    'freq': freq,
                    'first_offset': first_offset
                }
            except (configparser.NoOptionError, ValueError) as e:
                print_colored(f"Error in section '{section}': {e}", "\033[91m") 
                continue

    return timestamps_dicts

def analyse(timestamps_dicts, start_time, end_time):
    all_timestamps_dfs = []

    for topic_name, timestamps_dict in timestamps_dicts.items():
        timestamps = timestamps_dict['timestamps']
        expected_hz = timestamps_dict['freq']
        first_offset = timestamps_dict['first_offset']

        if len(timestamps) > 0:
            print_colored("=============================================================================================", "\033[0m") 
            
            print_colored(f"Topic: {topic_name}", "\033[92m") 

            # 根据第一个时间戳和最后一个之间戳还有start_time、end_time计算出是否有整体偏移？
            first_timestamp = int(timestamps[0])
            last_timestamp = int(timestamps[-1])
            
            real_start_time = int(start_time) - 1
            real_end_time = int(end_time) + 1 

            offset_start = first_timestamp - real_start_time
            offset_end = last_timestamp - real_end_time

            overall_offset = (offset_start + offset_end) / 2


            # 计算在start_time与end_time之间的实际频率与丢失率
            timestamps = [ts for ts in timestamps if start_time <= ts < end_time]
            message_count = len(timestamps)
            if message_count == 0:
                print_colored("", "\033[0m")
                print_colored("##################################  Summary  ##################################", "\033[93m")
                print_colored(f"Topic:", "\033[93m") 
                print_colored(f"{topic_name}", "\033[94m")
                print()
                print_colored("Overall Offset:", "\033[93m")
                if overall_offset < 0 :
                    # 如果整体偏移小于0，说明时间戳整体提前了
                    print_colored(f"Time offset: {overall_offset} seconds (timestamps are ahead)", "\033[91m")
                elif overall_offset > 0:
                    # 如果整体偏移大于0，说明时间戳整体延迟了
                    print_colored(f"Time offset: {overall_offset} seconds (timestamps are delayed)", "\033[91m")
                else:
                    #时间戳没有整体偏移
                    print_colored("Time offset: No offset detected", "\033[94m")
                
                print_colored("################################################################################", "\033[93m")
            else:
                time_diffs = np.diff(timestamps)
                actual_hz = 1.0 / np.mean(time_diffs)
                total_time = end_time - start_time
                expected_messages = total_time * expected_hz
                lost_rate = 1 - (message_count / expected_messages)

                # 如果丢失率大于0，找出异常时间段
                if lost_rate > 0:
                    # 将时间戳转换为秒级索引
                    seconds_indices = np.floor(timestamps).astype(int)
                    unique_seconds, counts_per_second = np.unique(seconds_indices, return_counts=True)

                    # 确定每个秒内的预期数据个数
                    expected_per_second = expected_hz

                    # 识别异常时间段
                    abnormal_seconds = []
                    for second, count in zip(unique_seconds, counts_per_second):
                        if count < expected_per_second:
                            # 如果这一秒的实际数据个数小于预期，则记录这个时间段
                            abnormal_seconds.append((second, count))
                        elif count > expected_per_second:
                            abnormal_seconds.append((second, count))

                    # 处理缺失所有数据的秒
                    all_seconds = set(range(start_time, end_time))
                    missing_seconds = all_seconds - set(unique_seconds)
                    for second in missing_seconds:
                        abnormal_seconds.append((second, 0))

                    # 打印异常时间段的信息
                    if abnormal_seconds:
                        # 不正常的秒，数据可能丢失或者增加
                        print_colored("Abnormal time intervals with data loss or increase:", "\033[91m")
                        for second, count in abnormal_seconds:
                            print_colored(f"Second: {second}s, Expected: {expected_per_second}, Actual: {count}", "\033[0m")
                    else:
                        print_colored("No abnormal time intervals found.", "\033[94m") 

                # 获取每秒的第一个时间戳
                first_timestamps_per_second = []
                current_second = None
                for ts in timestamps:
                    ts_second = int(ts)
                    if ts_second != current_second:
                        first_timestamps_per_second.append(ts)
                        current_second = ts_second

                df = pd.DataFrame({topic_name: first_timestamps_per_second})
                all_timestamps_dfs.append(df)

                if first_offset < 0.0:
                    # 设置第一个时间戳可能的偏移量
                    nanoseconds = [(ts - int(ts)) * 1e9 for ts in first_timestamps_per_second]
                    mean_nanoseconds = np.mean(nanoseconds)
                    first_offset = mean_nanoseconds / 1e9

                # 将每个时间戳转换为其所在秒内的偏移量
                offsets = [ts % 1 for ts in timestamps]
                # 生成理想偏移量
                ideal_offsets_per_second = np.linspace(0, 1, expected_hz, endpoint=False)
                # 平移理想偏移量
                ideal_offsets_shifted = (ideal_offsets_per_second + first_offset) % 1

                # 根据实际的时间戳生成理想偏移量
                ideal_offsets = []
                deviations = []
                for ts in timestamps:
                    sec = int(ts)
                    offset_in_sec = (ts - sec) % 1
                    closest_ideal_offset_index = np.argmin(np.abs(ideal_offsets_shifted - offset_in_sec))
                    closest_ideal_offset = ideal_offsets_shifted[closest_ideal_offset_index]
                    ideal_offsets.append(closest_ideal_offset)
                    # 计算实际偏移量与最接近的理想偏移量之间的偏差
                    deviation = abs(offset_in_sec - closest_ideal_offset)
                    deviations.append(deviation)

                # 计算标准差
                std_deviation = np.std(deviations)
                # 计算平均绝对偏差
                mad = np.mean(deviations)

                # 理想间隔是 1/expected_hz 秒
                ideal_interval = 1.0 / expected_hz
                # 计算偏差百分比
                deviation_percentages = (np.array(deviations) / ideal_interval)
                # 平均偏差百分比
                mean_deviation_percentage = np.mean(deviation_percentages)

                print_colored("", "\033[0m")
                print_colored("##################################  Summary  ##################################", "\033[93m")
                print_colored(f"Topic:", "\033[93m") 
                print_colored(f"{topic_name}", "\033[94m")
                print()
                print_colored("Overall Offset:", "\033[93m")
                if overall_offset < 0 :
                    print_colored(f"Time offset: {overall_offset} seconds (timestamps are ahead)", "\033[91m")
                elif overall_offset > 0:
                    print_colored(f"Time offset: {overall_offset} seconds (timestamps are delayed)", "\033[91m")
                else:
                    print_colored("Time offset: No offset detected", "\033[94m")
                print()
                print_colored("Message Statistics:", "\033[93m")
                print_colored(f"Expected messages: {expected_messages}, Actual messages: {message_count}, Lost messages: {expected_messages - message_count}", "\033[94m") 
                print_colored(f"Expected frequency: {expected_hz} Hz, Actual frequency: {actual_hz:.2f} Hz, Lost rate: {lost_rate:.2%}", "\033[94m") 
                print()
                print_colored("Timestamp Alignment:", "\033[93m")
                print_colored(f"Ideal interval: \t\t{ideal_interval:.3f} s", "\033[94m") 
                print_colored(f"Standard deviation: \t\t{std_deviation:.2f}", "\033[94m")
                print_colored(f"Mean Deviation: \t\t{mad:.2f} ns", "\033[94m")
                print_colored(f"Mean Deviation Percentage: \t{mean_deviation_percentage:.2%}", "\033[94m") 
                print_colored("################################################################################", "\033[93m")
                print()
                print_colored("=============================================================================================", "\033[0m") 

def main(config_file_path, csv_dir):
    """Main function to process the configuration and CSV files."""
    # 读取配置文件
    config = configparser.ConfigParser()
    config.read(config_file_path)

    start_time = int(config['Time']['start'])
    end_time = int(config['Time']['end'])

    # 读取所有时间戳
    timestamps_dicts = read_timestamps_from_csv(config, csv_dir)
    analyse(timestamps_dicts, start_time, end_time)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 analyse_csv.py <path_to_config_file> <csv_dir>")
        sys.exit(1)

    config_file_path = sys.argv[1]
    csv_dir = sys.argv[2]
    main(config_file_path, csv_dir)