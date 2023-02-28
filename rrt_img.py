import matplotlib.pyplot as plt
import numpy as np
import random
import progressbar
import math


def get_distance_point2line(point, line):
    """
    Args:
        point: [x0, y0]
        line: [x1, y1, x2, y2]
    """
    line_point1, line_point2 = np.array(line[0:2]), np.array(line[2:])
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    distance = np.abs(np.cross(
        vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
    return distance


class Node:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class RRT:

    def __init__(self, start, goal, obstacles, step_size, max_iter):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.nodes = [self.start]
        self.edges = []

    def random_sample(self):
        x = random.uniform(0, 10)
        y = random.uniform(0, 10)
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

    def is_collision_free(self, node1, node2):

        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y

        for obs in self.obstacles:
            x_obs, y_obs, r = obs

            d = get_distance_point2line([x_obs, y_obs], [x1, y1, x2, y2])
            if d < 0.5:
                return False
        return True

    """ Here is the explanation for the code above:
1. We create a random sample and find the nearest node to the sample. 
2. We then calculate the angle between the random sample and the nearest node.
3. We create a new node at a distance of step_size from the nearest node in the direction of the random sample.
4. We check if the new node is collision free.
5. If it is, we add it to the tree and then check if we can connect it to the goal.
6. If we can, we add the goal to the tree and break out of the loop. """

    def build_rrt(self):
        p = progressbar.ProgressBar()
        for i in p(range(self.max_iter)):
            sample = self.random_sample()
            nearest_node = self.find_nearest_node(sample)
            theta = np.arctan2(sample.y - nearest_node.y,
                               sample.x - nearest_node.x)
            new_node = Node(nearest_node.x + self.step_size * np.cos(theta),
                            nearest_node.y + self.step_size * np.sin(theta))
            if self.is_collision_free(nearest_node, new_node):
                self.nodes.append(new_node)
                new_node.parent = nearest_node
                self.edges.append((nearest_node, new_node))
                if self.is_collision_free(new_node, self.goal):
                    self.nodes.append(self.goal)
                    self.goal.parent = new_node
                    self.edges.append((new_node, self.goal))
                    break

    def plot(self):
        plt.figure()
        plt.plot(self.start.x, self.start.y, 'go')
        plt.plot(self.goal.x, self.goal.y, 'ro')
        for edge in self.edges:
            x = [edge[0].x, edge[1].x]
            y = [edge[0].y, edge[1].y]
            plt.plot(x, y, 'b-')
        # while True:
        #     if node.parent is not None:
        #         plt.plot([node.x, node.parent.x], [node.y, node.parent.y],
        #                  'b-')
        #         node = node.parent
        #     else:
        #         break

        plt.scatter(obstacles[:, 1], obstacles[:, 0], s=1)
        plt.axis('equal')
        plt.show()


if __name__ == '__main__':
    start = [100, 110]
    goal = [100, 0]
    namenum = 4
    img = np.load("mid/" + str(namenum) + ".npy")
    obstacles = img
    step_size = 2
    max_iter = 100

    rrt = RRT(start, goal, obstacles, step_size, max_iter)
    rrt.build_rrt()
    rrt.plot()
