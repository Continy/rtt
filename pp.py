import matplotlib.pyplot as plt
import numpy as np
import math


#点到直线距离 其中line为直线两点坐标
def point_to_line_distance(point, p1, p2):
    x0, y0 = point
    x1, y1 = p1
    x2, y2 = p2
    return abs((y2 - y1) * x0 -
               (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1)**2 +
                                                               (x2 - x1)**2)


def get_distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def get_relative_position(longitude, latitude, target_longitude,
                          target_latitude):
    #x为东西方向，y为南北方向，单位为米
    x = math.cos(
        latitude * math.pi / 180) * (target_longitude - longitude) * 111000
    y = (target_latitude - latitude) * 111000
    return [x, y]


def get_los_point(ship, path, step=5):
    #当前终点

    end = get_relative_position(path[0]['longitude'], path[0]['latitude'],
                                path[1]['longitude'], path[1]['latitude'])
    #当前船位置

    mid_distance = point_to_line_distance(ship, [0, 0], end)

    #船位置关于终点向量的垂直向量
    ship_vector = [
        end[1] / math.sqrt(end[0]**2 + end[1]**2),
        -end[0] / math.sqrt(end[0]**2 + end[1]**2)
    ]

    #船位置关于终点向量的垂直投影
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
    print(ship, los_end)

    plt.scatter(los_end[0], los_end[1], c='r')
    plt.scatter(ship[0], ship[1], c='b')
    plt.scatter(end[0], end[1], c='g')
    plt.scatter(ship_project[0], ship_project[1], c='y')
    plt.plot([ship[0], ship_project[0]], [ship[1], ship_project[1]])
    plt.plot([0, ship_vector[0]], [0, ship_vector[1]])
    ax = plt.gca()
    ax.set_aspect(1)
    plt.show()
    return los_end


if __name__ == '__main__':

    path = [{
        "longitude": 121.438967128929,
        "latitude": 31.0284019596673
    }, {
        "longitude": 121.438074312978,
        "latitude": 31.0286316336127
    }]
    ship = [-21.3, -9.9]
    p1 = get_relative_position(path[0]['longitude'], path[0]['latitude'],
                               path[0]['longitude'], path[0]['latitude'])
    p2 = get_relative_position(path[0]['longitude'], path[0]['latitude'],
                               path[1]['longitude'], path[1]['latitude'])
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]])

    get_los_point(ship, path)