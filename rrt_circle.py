import matplotlib.pyplot as plt
import numpy as np
import random
import math
from scipy import interpolate


class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:

    def __init__(self, start, goal, obstacles, step_size, max_iter, boundary):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.edges = []
        self.origin_path = []
        self.assist_control_points = []
        self.control_points = []
        self.path = []
        self.boundary = boundary

    def cal_distance(self, node1, node2):  # 参数形式 point：[x,y]
        if node2.__class__ == Node:
            distance = math.sqrt((node1.x - node2.x)**2 +
                                 (node1.y - node2.y)**2)
            return distance
        else:
            distance = math.sqrt((node1.x - node2[0])**2 +
                                 (node1.y - node2[1])**2)
            return distance

    def random_sample(self):
        x = random.uniform(self.boundary[0], self.boundary[1])
        y = random.uniform(self.boundary[2], self.boundary[3])
        return Node(x, y)

    def find_nearest_node(self, node):
        min_dist = float('inf')
        nearest_node = None
        for n in self.nodes:
            dist = np.sqrt((node.x - n.x)**2 + (node.y - n.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_node = n
        return nearest_node

    def check_collision(
            self, node1, node2
    ):  # 参数形式 point：[x,y]; obstacle: [x,y,r] 设障碍物为圆形，x,y为圆心坐标，r为圆的半径
        #求两向量叉积
        for obstacle in self.obstacles:
            p1 = [node1.x - obstacle[0], node1.y - obstacle[1]]
            p2 = [node2.x - obstacle[0], node2.y - obstacle[1]]
            #三角形面积的两倍
            area = abs(p1[0] * p2[1] - p1[1] * p2[0])
            #三角形底边长度
            base = self.cal_distance(node1, node2)
            #求高
            height = area / base
            #三角形较长边长
            long_side = max(self.cal_distance(node1, obstacle),
                            self.cal_distance(node2, obstacle))
            #三角形较短边长
            short_side = min(self.cal_distance(node1, obstacle),
                             self.cal_distance(node2, obstacle))
            #求垂足到较远点的距离
            distance = math.sqrt(long_side**2 - height**2)

            if distance > base and short_side > obstacle[2]:
                continue
            elif height > obstacle[2]:
                continue
            else:
                return False
        return True

    def build_rrt(self):
        for i in range(self.max_iter):
            sample = self.random_sample()
            nearest_node = self.find_nearest_node(sample)
            theta = np.arctan2(sample.y - nearest_node.y,
                               sample.x - nearest_node.x)
            new_node = Node(nearest_node.x + self.step_size * np.cos(theta),
                            nearest_node.y + self.step_size * np.sin(theta))
            #判断是否碰撞
            if self.check_collision(nearest_node, new_node):  #如果不碰撞
                self.nodes.append(new_node)
                new_node.parent = nearest_node
                self.edges.append((nearest_node, new_node))
                if self.check_collision(new_node, self.goal):  #如果到达终点
                    self.nodes.append(self.goal)
                    self.goal.parent = new_node
                    self.edges.append((new_node, self.goal))
                    break
        node = self.goal
        while True:
            if node.parent is not None:
                self.origin_path.append((node.parent, node))
                node = node.parent
            else:
                self.origin_path.append((self.start, node))
                break

    #获取某一节点所有的上级节点
    def get_parents(self, node):
        parents = []
        generation = 1
        while node.parent is not None:
            parents.append([node.parent, generation])
            node = node.parent
            generation += 1
        parents.reverse()
        return parents

    def prune(self, node, mode=float('inf')):

        if node.parent is None:
            return
        else:
            for point in self.get_parents(node):
                if point[1] > mode:
                    continue
                if self.check_collision(point[0], node):
                    node.parent = point[0]
                    break
            self.prune(node.parent, mode)

    def get_path(self):
        self.path = []
        node = self.goal
        while True:
            if node.parent is not None:
                self.path.append((node.parent, node))
                node = node.parent
            else:
                self.path.append((self.start, node))
                break

    #点到直线的距离
    def point_to_line(self, node1, node2, node3):
        a = node2.y - node1.y
        b = node1.x - node2.x
        c = node2.x * node1.y - node1.x * node2.y
        return abs(a * node3.x + b * node3.y + c) / math.sqrt(a**2 + b**2)

    #构建贝塞尔曲线所需的控制点
    def get_control_points(self, mode='default'):
        if mode == 'default':
            for i in range(0, len(self.origin_path) - 1):
                if i == 0:
                    self.control_points.append(self.origin_path[i][1])
                    self.control_points.append(self.origin_path[i][0])
                # elif i % 3 == 0:
                #     k = (self.origin_path[i][1].y - self.origin_path[i][0].y) / (
                #         self.origin_path[i][1].x - self.origin_path[i][0].x)
                #     theta = math.atan(k)
                #     #辅助点x坐标
                #     x = self.origin_path[i][1].x + self.step_size * math.cos(
                #         theta) / 2
                #     #辅助点y坐标
                #     y = self.origin_path[i][1].y + self.step_size * math.sin(
                #         theta) / 2
                #     self.assist_control_points.append(Node(x, y))
                #     self.control_points.append(Node(x, y))
                #     self.control_points.append(self.origin_path[i][1])
                else:
                    self.control_points.append(self.origin_path[i][0])
        elif mode == 'max':
            for i in range(0, len(self.path) - 1):
                if i == 0:
                    self.control_points.append(self.path[i][1])
                    self.control_points.append(self.path[i][0])
                else:
                    self.control_points.append(self.path[i][0])

    #绘制贝塞尔曲线
    def bezier(self):
        total_curve = []
        for i in range(0, len(self.control_points) - 3, 3):
            curve = []
            for t in np.arange(0, 1, 0.01):
                x = (1 - t)**3 * self.control_points[i].x + 3 * t * (
                    1 - t)**2 * self.control_points[i + 1].x + 3 * t**2 * (
                        1 - t) * self.control_points[
                            i + 2].x + t**3 * self.control_points[i + 3].x
                y = (1 - t)**3 * self.control_points[i].y + 3 * t * (
                    1 - t)**2 * self.control_points[i + 1].y + 3 * t**2 * (
                        1 - t) * self.control_points[
                            i + 2].y + t**3 * self.control_points[i + 3].y
                curve.append(Node(x, y))
            total_curve.append(curve)
        return total_curve

    def plot(self):
        # plt.figure()
        # plt.plot(self.start.x, self.start.y, 'go')
        # plt.plot(self.goal.x, self.goal.y, 'ro')
        node = self.goal
        for edge in self.edges:
            x = [edge[0].x, edge[1].x]
            y = [edge[0].y, edge[1].y]
            plt.plot(x, y, 'b-')
        for edge in self.origin_path:
            x = [edge[0].x, edge[1].x]
            y = [edge[0].y, edge[1].y]
            plt.plot(x, y, 'y-')
        for line in self.path:
            x = [line[0].x, line[1].x]
            y = [line[0].y, line[1].y]
            plt.plot(x, y, 'g-')
        # for point in self.control_points:
        #     plt.scatter(point.x, point.y, c='k')
        # for point in self.assist_control_points:
        #     plt.scatter(point.x, point.y, c='r')
        # #绘制样条曲线
        # total_curve = self.bezier()
        # for curve in total_curve:
        #     x = [point.x for point in curve]
        #     y = [point.y for point in curve]
        #     plt.plot(x, y, 'r-')

        for obs in self.obstacles:
            x, y, r = obs
            circle = plt.Circle((x, y), r, color='k')
            plt.gcf().gca().add_artist(circle)
        plt.axis('equal')
        plt.show()


if __name__ == '__main__':
    boundary = [0, 100, 0, 100]
    start = [10, 10]
    goal = [90, 90]
    obstacles = [[55, 60, 10], [100, 60, 15], [40, 30, 18], [90, 20, 15],
                 [20, 40, 10], [60, 100, 15], [65, 45, 10], [85, 45, 10],
                 [10, 40, 10], [76, 77, 13]]
    # obstacles = [[55, 60, 10], [100, 60, 15], [40, 30, 18], [90, 20, 15],
    #              [20, 40, 10], [60, 100, 15], [65, 45, 10]]
    step_size = 4
    max_iter = 10000

    rrt = RRT(start, goal, obstacles, step_size, max_iter, boundary)
    rrt.build_rrt()
    rrt.prune(rrt.goal, 3)
    rrt.get_path()
    rrt.get_control_points()
    rrt.plot()
