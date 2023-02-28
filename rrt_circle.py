import matplotlib.pyplot as plt
import numpy as np
import random


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
        for obs in self.obstacles:
            x_obs, y_obs, r_obs = obs
            x1, y1 = node1.x, node1.y
            x2, y2 = node2.x, node2.y
            m = (y2 - y1) / (x2 - x1)
            c = y1 - m * x1
            x = np.linspace(x1, x2, num=50)
            y = m * x + c
            d = (x - x_obs)**2 + (y - y_obs)**2
            if any(d < 1):
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
        for i in range(self.max_iter):
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
        node = self.goal
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
        for obs in self.obstacles:
            x, y, r = obs
            circle = plt.Circle((x, y), r, color='k')
            plt.gcf().gca().add_artist(circle)
        plt.axis('equal')
        plt.show()


start = [1, 1]
goal = [9, 9]
obstacles = [[5, 5, 1], [1, 5, 1], [4, 2, 1], [1, 9, 1], [1, 3, 1]]
step_size = 0.3
max_iter = 10000

rrt = RRT(start, goal, obstacles, step_size, max_iter)
rrt.build_rrt()
rrt.plot()
