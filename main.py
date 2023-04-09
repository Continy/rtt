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
pre_error = 0
sum_error = 0
distance = 0
is_close_to_start = False


#求距离
def get_distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


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
    Ki = 0.0
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


def get_path(wps84_begin, wps84_end):
    if longitude is None or latitude is None or buoy_list is None:
        return
    else:
        return path(wps84_begin, wps84_end)


#计算向量与x轴的夹角，范围为0~360
def get_angle(vector):
    x = vector[0]
    y = vector[1]
    angle = math.atan2(x, y) * 180 / math.pi
    if angle < 0:
        angle = 360 + angle
    return angle


#根据角度计算向量
def get_vector(angle):
    angle = angle * math.pi / 180
    vector = [4 * math.sin(angle), 4 * math.cos(angle)]
    return vector


def path(wps84_begin, wps84_end):
    global path
    global buoy_list
    wps84_obstacles = buoy_list
    # wps84_begin = [121.438074312978, 31.0286316336127]
    # wps84_end = [121.438954507803, 31.0280814275016]
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
        obstacles.append([x, y, 0])
    #坐标转换

    boundary = [-35, 58, -35, 10]

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

    # total_curve = rrt.bezier()
    wps84_curve = []  #json格式
    # for curve in total_curve:
    #     x = [point.x for point in curve]
    #     y = [point.y for point in curve]
    #     plt.plot(x, y, 'r-')
    # for curve in total_curve:
    #     for point in curve:
    #         [longitude,
    #          latitude] = get_absolute_position(wps84_begin[0], wps84_begin[1],
    #                                            point.x, point.y)
    #         wps84_curve.append({'longitude': longitude, 'latitude': latitude})
    with open('curve.json', 'w') as f:
        json.dump(wps84_curve, f)
    rrt.plot()
    wps84_path.pop(0)
    wps84_path.reverse()
    with open('path.json', 'w') as f:
        json.dump(wps84_path, f)
    return wps84_path


#阶段速度控制
def speed_control(distance):
    if distance > 20:
        return 0.4
    elif distance > 10:
        return 0.4
    else:
        return 0.05 * distance


#求两向量夹角
def get_angle_between(vector1, vector2):
    angle = math.acos((vector1[0] * vector2[0] + vector1[1] * vector2[1]) /
                      (math.sqrt(vector1[0]**2 + vector1[1]**2) *
                       math.sqrt(vector2[0]**2 + vector2[1]**2)))
    angle = angle * 180 / math.pi
    return angle


#回到起点


#循迹算法 ship与point为相对坐标
def follow_point(ship, point):

    ship_vector = get_vector(yaw)
    #船首与目标点向量

    distance = get_distance(ship, point)
    target_yaw = get_angle([point[0] - ship[0], point[1] - ship[1]])

    #print(target_yaw)
    steer = PID_controller(yaw, target_yaw)
    if distance < 3:
        return True
    #Control_send(0.2, steer)
    if yaw - target_yaw < 90:
        Control_send(speed_control(distance), steer)
    else:
        Control_send(-speed_control(distance), steer)
    return False


def point_to_line_distance(point, p1, p2):
    x0, y0 = point
    x1, y1 = p1
    x2, y2 = p2
    return abs((y2 - y1) * x0 -
               (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1)**2 +
                                                               (x2 - x1)**2)


#循迹算法 方法为LOS视线引导法
def get_los_point(ship, path, step=5):
    #当前终点

    end = get_relative_position(path[0]['longitude'], path[0]['latitude'],
                                path[1]['longitude'], path[1]['latitude'])
    #当前船位置

    #终点关于原点的向量
    mid_distance = point_to_line_distance(ship, [0, 0], end)
    #船位置关于终点向量的垂直向量
    #判断船在航迹的左侧还是右侧
    ship_vector = [
        end[1] / math.sqrt(end[0]**2 + end[1]**2),
        -end[0] / math.sqrt(end[0]**2 + end[1]**2)
    ]
    if ship[0] * end[1] - ship[1] * end[0] > 0:

        ship_vector = [-ship_vector[0], -ship_vector[1]]
    #船位置关于终点向量的投影

    ship_project = [
        ship[0] + mid_distance * ship_vector[0],
        ship[1] + mid_distance * ship_vector[1]
    ]
    #步长向量
    delta = [
        step * end[0] / math.sqrt(end[0]**2 + end[1]**2),
        step * end[1] / math.sqrt(end[0]**2 + end[1]**2)
    ]
    #视线引导终点
    if step >= get_distance(ship_project, end):
        los_end = end

    else:
        los_end = [ship_project[0] + delta[0], ship_project[1] + delta[1]]
    #循迹
    plt.scatter(los_end[0], los_end[1], c='r')
    plt.scatter(ship[0], ship[1], c='b')
    plt.plot([0, end[0]], [0, end[1]], c='g')
    plt.plot([ship[0], ship_project[0]], [ship[1], ship_project[1]], c='y')
    plt.plot([ship_project[0], los_end[0]], [ship_project[1], los_end[1]],
             c='r')
    #plt.show()

    return los_end


# 控制算法
def Model():
    global distance
    is_path = False
    path_length = 0
    finall = False
    reached = True
    los_end = []
    end = []
    while True:

        if longitude == None or latitude == None or pitch == None or roll == None or yaw == None:

            time.sleep(0.1)
            continue

        if is_path == False:
            print("初始化")
            # path = get_path([longitude, latitude],
            #                 [121.438074312978, 31.0286316336127])
            # print(longitude, latitude)
            # path = get_path([longitude, latitude],
            #                 [121.438954507803, 31.0280814275016])
            path = [{
                "longitude": 121.437910949701,
                "latitude": 31.0287220671825
            }, {
                "longitude": 121.438954507803,
                "latitude": 31.0280814275016
            }]
            path_length = len(path)
            Path_send(path)
            is_path = True
            if reached:
                ship = get_relative_position(path[0]['longitude'],
                                             path[0]['latitude'], longitude,
                                             latitude)
                los_end = get_los_point(ship, path, step=10)
                end = get_relative_position(path[0]['longitude'],
                                            path[0]['latitude'],
                                            path[1]['longitude'],
                                            path[1]['latitude'])
                reached = False
        # los_end = get_relative_position(path[0]['longitude'],
        #                                 path[0]['latitude'],
        #                                 path[1]['longitude'],
        #                                 path[1]['latitude'])
        # path = get_path([longitude, latitude],
        #                 [121.438954507803, 31.0280814275016])
        # Path_send(path)
        #print(is_path)
        #回到起点

        ship = get_relative_position(path[0]['longitude'], path[0]['latitude'],
                                     longitude, latitude)
        reached = follow_point(ship, los_end)
        if get_distance(ship, end) > 3 and reached:
            los_end = get_los_point(ship, path, step=10)
            reached = False
            print("到达中间点")
            continue
        elif reached:
            path.pop(0)
            if len(path) == 1:
                print('到达目的地!')
                Control_send(0, 0)
                finall = True
                break

            los_end = get_los_point(ship, path, step=10)
            reached = False
        if finall:
            break
        distance = get_distance(ship, los_end)
        time.sleep(0.1)  #控制量更新频率


def midData():
    while True:
        print("下一次目标点距离为：", distance)
        time.sleep(1)


if __name__ == "__main__":

    #多线程接收
    thread_GPS = threading.Thread(target=GPS_receive, daemon=True)
    thread_IMU = threading.Thread(target=IMU_receive, daemon=True)
    thread_Buoy = threading.Thread(target=Buoy_receive, daemon=True)
    thread_Mid = threading.Thread(target=midData, daemon=True)
    #thread_Path = threading.Thread(target=get_path, daemon=True)
    thread_GPS.start()
    thread_IMU.start()
    thread_Buoy.start()
    thread_Mid.start()
    #thread_Path.start()

    #控制程序
    Model()
