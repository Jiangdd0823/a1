import re, os
import math
from datetime import datetime

m_pi = 3.1415926535  # 定义π常量

def hex_to_signed(hex_str):
    if not hex_str or len(hex_str) != 4:
        return 0.0
    value = int(hex_str, 16)
    if value >= 0x8000:  # 检查符号位
        value -= 0x10000
    return value

def parse_IMU(line):
    line = re.sub(r'[,]', ',', line.strip())  # 统一分隔符[4,7](@ref)
    parts = re.split(r'[,]', line)
    
    timestamp = parts[1]
    
    hex_data = []
    for part in parts[2:-1]:
        hex_seqs = re.findall(r'[0-9A-Fa-f]{4}', part, re.IGNORECASE)
        hex_data.extend(hex_seqs)
        # hex_seqs = int(part[1:], 16)
        # hex_data.append(hex_seqs)
        
    
    # if len(hex_data) < 6:
    #     return None
    
    # gyro = [h/131.2 for h in hex_data[:3]]  # 陀螺仪单位转换[^用户代码]
    # accel = [h/1024 for h in hex_data[3:6]]  # 加速度计单位转换[^用户代码]
    gyro = [hex_to_signed(h)/131.2 for h in hex_data[:3]]  # 陀螺仪单位转换[^用户代码]
    accel = [hex_to_signed(h)/1024.0 for h in hex_data[3:6]] 
    return (timestamp, gyro, accel)

def parse_nmea(line):
    if line.startswith('$GNGGA'):
        data = line.split(',')
        # if len(data) < 13:
        #     return

        # 时间解析
        time_str = data[1]
        # utc_time = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:6]}"
        result = []
        # result.append(time_str)
        # 纬度转换
        lat_n = float(data[2][:2]) + float(data[2][2:])/60 if data[2] else 0.0
        lon_e = float(data[4][:3]) + float(data[4][3:])/60 if data[4] else 0.0
        lat_n_t = lat_n/180 * m_pi
        lon_e_t = lon_e/180 * m_pi
        result += [f"{lat_n_t:.10f}", f"{lon_e_t:.10f}"]
        # result.append(lat_n_t, lon_e_t)
        # result.append(data[6:9])
        # 经度转换
        # lon = float(data[9])
        # lon_hemi = data[5]
        
        # 其他参数
        quality = data[6] if data[6] else 0.0
        # num_sats = data[7]
        hdop = data[8] if data[8] else 0.0
        altitude = float(data[9]) if data[9] else 0.0
        # geoid_separation = data[11]
        result += [altitude, quality, hdop]
        # result.append(quality, num_sats, hdop, altitude, geoid_separation)
        
        return time_str, result

    elif line.startswith('$GNRMC'):
        data = line.split(',')
        # 时间日期解析
        time_str = data[1]
        result = []
        # 纬度转换
        # lat_n = float(data[3][:2]) + float(data[3][2:])/60 if data[3] else 0.0
        # lon_e = float(data[5][:3]) + float(data[5][3:])/60 if data[5] else 0.0
        # lat_n_t = lat_n/180 * m_pi
        # lon_e_t = lon_e/180 * m_pi
        
        # 运动参数
        speed_knots = float(data[7]) *1.852/3.6  if data[7] else 0.0
        course = float(data[8])/180*m_pi if data[8] else 0.0
        result += [1, f"{speed_knots:.6f}", f"{course:.6f}"]
        # result.append(lat_n_t, lon_e_t, speed_knots, course)
        return time_str, result

    elif line.startswith('$GNGST'):
        data = line.split(',')
        # if len(data) < 8:
        #     return
        time_str = data[1]
        # 误差统计
        # data[5] = float(data[5]) /180*m_pi
        return time_str, data[-3:-1] + [data[-1].split('*')[0]]+[0,0,0]
       
def convert_file(input_path, output_path):
    with open(input_path, 'r', encoding='utf-8') as f_in:  # 指定编码避免乱码[4,7](@ref)
        with open(output_path, 'w', encoding='utf-8') as f_out:
            result_list = []
            for line in f_in:
                
                if line.startswith(('$GPIMU')):                
                    result = parse_IMU(line)
                    if not result:
                        continue
                    
                    timestamp, gyro, accel = result
                    f_out.write(f"{timestamp}")
                    f_out.write(",1,")
                    f_out.write(",".join(f"{v:.6f}" for v in accel))
                    f_out.write(",")
                    f_out.write(",".join(f"{v:.6f}" for v in gyro))
                    f_out.write(",")
                    f_out.write(",".join(f"{v}" for v in [0,0]))
                    f_out.write("\n")
                    
                elif line.startswith(('$GNGGA', '$GNRMC', '$GNGST')):
                    timestamp, result = parse_nmea(line)
                    result_list += result
                    if line.startswith(('$GNGST')):
                        f_out.write(f"{timestamp}")
                        f_out.write(",0,")
                        f_out.write(",".join(f"{r}" for r in result_list))
                        f_out.write("\n")
                        result_list = []

def fixtxt(input_file, output_file):
    with open(input_file, 'r') as f_in:
        with open(output_file, 'w') as f_out:
            
            result_list = []
            for line in f_in:
                data = line.split(',')
                cls = data[1]                    
                if int(cls) == 0 and float(data[0]) == int(float(data[0])):
                    result_list.append(line)
                else:
                    if result_list:
                        f_out.write(line)
                        # f_out.write('\n')
                        f_out.write(result_list[0])
                        # f_out.write('\n')
                        result_list = []
                    else:
                        f_out.write(line)
                        # f_out.write('\n')
                        
def trans_seconds(input_file, output_file):
    with open(input_file, 'r') as f_in:
        with open(output_file, 'w') as f_out:
            
            for line in f_in:
                data = line.split(',')
                data1 = data[1:]
                time = data[0].split('.')
                m_s = time[1]
                date_t = time[0]
                h = date_t[:2]
                m = date_t[2:4]
                s = date_t[4:6]
                total_s = float(h) * 3600 + float(m) * 60 + float(s)
                time_str = str(int(total_s)) + '.' + m_s
                f_out.write(time_str)
                f_out.write(',')
                f_out.write(",".join(f"{r}" for r in data1))
                # f_out.write("\n")
                        
if __name__ == "__main__":
    # input_file = "2025-04-18_03-18-18.txt"  # 替换为实际输入文件路径
    # output_file = "output-04-18_03-18-18_.txt"  # 替换为输出文件路径
    # fix_file = 'output-04-18_03-18-18.txt'
    
    name=input('请输入要转换txt文件名（把txt文件放在与.exe同一路径下）：')
    name = str(name)
    input_file = name + '.txt'
    output_file = 'output-' + name + '_.txt'
    fix_file = 'output-' + name + '.txt'
    convert_file(input_file, output_file)
    fixtxt(output_file, fix_file)
    print('转换为：', fix_file)
    if os.path.exists(output_file):
        os.remove(output_file)
