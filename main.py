import socket
import json
import threading
import time
from rrt_circle import RRT
import matplotlib.pyplot as plt
import math

Host = '127.0.0.1'  #IP地址

Port_GPS = 60100  #GPS端口
Port_IMU = 60101  #IMU端口
Port_Buoy = 60103  #浮标端口

Port_Control = 60199  #控制端口
Port_Path = 60198  #路径输出端口

socket_GPS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建UDP套接字
socket_IMU = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_Buoy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket_Control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
socket_Path = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

socket_GPS.bind((Host, Port_GPS))  #端口绑定
socket_IMU.bind((Host, Port_IMU))
socket_Buoy.bind((Host, Port_Buoy))

longitude = None  #经度
latitude = None  #纬度

pitch = None  #俯仰角
roll = None  #翻滚角
yaw = None  #偏航角
velocity = None  #速度

buoy_list = None  #浮标位置
path = None  #路径
dt = 0.1
dl = 2


# GPS信息接收函数
def GPS_receive():
    global longitude, latitude
    while True:
        data, _ = socket_GPS.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        longitude = data_json["longitude"]
        latitude = data_json["latitude"]


# IMU信息接收函数
def IMU_receive():
    global pitch, roll, yaw, velocity
    while True:
        data, _ = socket_IMU.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        pitch = data_json["pitch"]
        roll = data_json["roll"]
        yaw = data_json["yaw"]
        velocity = data_json["velocity"]


# 浮标位置接收函数
def Buoy_receive():
    global buoy_list
    while True:
        data, _ = socket_Buoy.recvfrom(1024)
        data_json = json.loads(data.decode('utf-8'))
        buoy_list = data_json['buoys']


def PID_controller(current_yaw, target_yaw):
    global pre_error, sum_error

    Kp = 0.02
    Ki = 0
    Kd = 0.002

    max = 1
    min = -1

    error = target_yaw - current_yaw

    # 将error的值限制在(-180,180]
    if error > 180:
        error = error - 360
    if error <= -180:
        error = error + 360

    steer = Kp * error + Ki * sum_error * dt + Kd * (error - pre_error) / dt

    if steer > max:
        steer = max
    if steer < min:
        steer = min

    sum_error += error
    pre_error = error

    return steer


# 发送路径
def Path_send(path_points):
    path = {}
    path["points"] = path_points
    path = json.dumps(path)
    path = path.encode('utf-8')
    socket_Path.sendto(path, (Host, Port_Path))


# 发送控制信号，throttle为油门，steer为方向，范围均为 -1 ~ 1
def Control_send(throttle, steer):
    control_cmd = {}
    control_cmd["throttle"] = throttle
    control_cmd["steer"] = steer
    control_cmd = json.dumps(control_cmd)
    control_cmd = control_cmd.encode('utf-8')
    socket_Control.sendto(control_cmd, (Host, Port_Control))


#以起点为原点，计算目标点的相对坐标,单位为米
def get_relative_position(longitude, latitude, target_longitude,
                          target_latitude):
    #x为东西方向，y为南北方向，单位为米
    x = math.cos(
        latitude * math.pi / 180) * (target_longitude - longitude) * 111000
    y = (target_latitude - latitude) * 111000
    return [x, y]


#已知目标点的相对坐标，计算目标点的绝对坐标
def get_absolute_position(longitude, latitude, x, y):
    #x为东西方向，y为南北方向，单位为米d
    target_longitude = longitude + x / 111000 / math.cos(
        latitude * math.pi / 180)
    target_latitude = latitude + y / 111000
    return [target_longitude, target_latitude]


def get_path():
    if longitude is None or latitude is None or buoy_list is None:
        return
    else:
        return path()


def path():
    global path
    global buoy_list
    wps84_obstacles = buoy_list
    wps84_begin = [121.438074312978, 31.0286316336127]
    wps84_end = [121.438954507803, 31.0280814275016]
    begin = get_relative_position(wps84_begin[0], wps84_begin[1],
                                  wps84_begin[0], wps84_begin[1])
    end = get_relative_position(wps84_begin[0], wps84_begin[1], wps84_end[0],
                                wps84_end[1])
    # with open('obstacles.json', 'r') as f:
    #     wps84_obstacles = json.load(f)
    obstacles = []
    for obstacle in wps84_obstacles:
        [x, y] = get_relative_position(wps84_begin[0], wps84_begin[1],
                                       obstacle['longitude'],
                                       obstacle['latitude'])
        obstacles.append([x, y, 5])
    #坐标转换
    #print(obstacles)
    # plt.figure()
    # for obstacle in obstacles:
    #     circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2])
    #     plt.gca().add_patch(circle)
    # plt.scatter(begin[0], begin[1], c='r')
    # plt.scatter(end[0], end[1], c='g')

    boundary = [-35, 58, -35, 10]
    #draw boundary
    # plt.plot([boundary[0], boundary[1]], [boundary[2], boundary[2]], c='k')
    # plt.plot([boundary[0], boundary[1]], [boundary[3], boundary[3]], c='k')
    # plt.plot([boundary[0], boundary[0]], [boundary[2], boundary[3]], c='k')
    # plt.plot([boundary[1], boundary[1]], [boundary[2], boundary[3]], c='k')
    step_size = 1
    max_iter = 1000
    rrt = RRT(begin, end, obstacles, step_size, max_iter, boundary)
    rrt.build_rrt()
    rrt.prune(rrt.goal)
    rrt.get_path()
    rrt.get_control_points(mode='default')
    path_points = []
    t = 0
    for line in rrt.path:
        if t == 0:
            path_points.append([line[0].x, line[0].y])
            path_points.append([line[1].x, line[1].y])
            t = 1
            continue
        path_points.append([line[1].x, line[1].y])
    wps84_path = []  #json格式
    for point in path_points:
        [longitude,
         latitude] = get_absolute_position(wps84_begin[0], wps84_begin[1],
                                           point[0], point[1])
        wps84_path.append({'longitude': longitude, 'latitude': latitude})
    with open('path.json', 'w') as f:
        json.dump(wps84_path, f)

    total_curve = rrt.bezier()
    wps84_curve = []  #json格式
    for curve in total_curve:
        x = [point.x for point in curve]
        y = [point.y for point in curve]
        plt.plot(x, y, 'r-')
    for curve in total_curve:
        for point in curve:
            [longitude,
             latitude] = get_absolute_position(wps84_begin[0], wps84_begin[1],
                                               point.x, point.y)
            wps84_curve.append({'longitude': longitude, 'latitude': latitude})
    with open('curve.json', 'w') as f:
        json.dump(wps84_curve, f)
    #rrt.plot()
    return wps84_path


# 控制算法
def Model():

    while True:

        if longitude == None or latitude == None or pitch == None or roll == None or yaw == None:

            # 如果没有收到信息，就跳出这个循环
            time.sleep(0.1)
            continue

        print("longitude:", longitude)
        print("latitude:", latitude)

        print("pitch:", pitch)
        print("roll:", roll)
        print("yaw:", yaw)

        print("buoy:", buoy_list)

        path = [{
            'longitude': 121.438074312978,
            'latitude': 31.0286316336127
        }, {
            'longitude': 121.438954507803,
            'latitude': 31.0280814275016
        }]
        #path = get_path()
        try:
            Path_send(path)
        except:
            pass
        end_point = get_relative_position(path[0]['longitude'],
                                          path[0]['latitude'],
                                          path[1]['longitude'],
                                          path[1]['latitude'])
        ship_point = get_relative_position(path[0]['longitude'],
                                           path[0]['latitude'], longitude,
                                           latitude)
        startpoint = [0, 0]
        #垂线方程

        target_yaw = 0  #待计算
        steer = PID_controller(yaw, target_yaw)  #控制

        Control_send(0.2, steer)  #发送控制量

        time.sleep(dt)  #控制量更新频率


if __name__ == "__main__":

    #多线程接收
    thread_GPS = threading.Thread(target=GPS_receive, daemon=True)
    thread_IMU = threading.Thread(target=IMU_receive, daemon=True)
    thread_Buoy = threading.Thread(target=Buoy_receive, daemon=True)
    thread_Path = threading.Thread(target=get_path, daemon=True)
    thread_GPS.start()
    thread_IMU.start()
    thread_Buoy.start()
    thread_Path.start()

    #控制程序
    Model()
