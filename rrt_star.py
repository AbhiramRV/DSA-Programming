import numpy as np
import matplotlib.pyplot as plt

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRTStar:
    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=3.0, goal_sample_rate=5, max_iter=500):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.min_x, self.max_x = rand_area[0][0], rand_area[0][1]
        self.min_y, self.max_y = rand_area[1][0], rand_area[1][1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.node_list = [self.start]

    def planning(self):
        for i in range(self.max_iter):
            rnd_node = self.generate_random_node()
            nearest_node = self.find_nearest_node(rnd_node)
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            if self.check_collision(new_node, self.obstacle_list):
                near_nodes = self.find_near_nodes(new_node)
                min_cost_node = nearest_node
                min_cost = self.calculate_cost(nearest_node) + self.calculate_distance(nearest_node, new_node)
                for near_node in near_nodes:
                    if self.check_collision(near_node, self.obstacle_list):
                        cost = self.calculate_cost(near_node) + self.calculate_distance(near_node, new_node)
                        if cost < min_cost:
                            min_cost_node = near_node
                            min_cost = cost
                new_node = self.rewire(new_node, near_nodes, min_cost_node)
                self.node_list.append(new_node)
                self.try_connect_to_goal(new_node)
            if i % 5 == 0:
                self.draw_graph()
            print(i)

    def generate_random_node(self):
        if np.random.randint(0, 100) > self.goal_sample_rate:
            rnd = Node(np.random.uniform(self.min_x, self.max_x),
                       np.random.uniform(self.min_y, self.max_y))
        else:
            rnd = Node(self.goal.x, self.goal.y)
        return rnd

    def find_nearest_node(self, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in self.node_list]
        minind = dlist.index(min(dlist))
        return self.node_list[minind]

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d = self.calculate_distance(from_node, to_node)
        if d > extend_length:
            new_node.x = from_node.x + extend_length * (to_node.x - from_node.x) / d
            new_node.y = from_node.y + extend_length * (to_node.y - from_node.y) / d
        else:
            new_node.x = to_node.x
            new_node.y = to_node.y
        new_node.parent = from_node
        return new_node

    def check_collision(self, node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision
        return True  # safe

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.expand_dis * np.sqrt((np.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [ind for ind, distance in enumerate(dlist) if distance <= r ** 2]
        near_nodes = [self.node_list[ind] for ind in near_inds]
        return near_nodes

    def rewire(self, new_node, near_nodes, min_cost_node):
        for near_node in near_nodes:
            if near_node == min_cost_node:
                continue
            if self.check_collision(near_node, self.obstacle_list):
                cost = self.calculate_cost(new_node) + self.calculate_distance(new_node, near_node)
                if cost < self.calculate_cost(near_node):
                    near_node.parent = new_node
        return new_node

    def calculate_cost(self, node):
        cost = 0.0
        while node.parent:
            cost += self.calculate_distance(node, node.parent)
            node = node.parent
        return cost

    def calculate_distance(self, from_node, to_node):
        return np.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)

    def try_connect_to_goal(self, new_node):
        goal_node = Node(self.goal.x, self.goal.y)
        d = self.calculate_distance(new_node, goal_node)
        if d <= self.expand_dis:
            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(goal_node)
                goal_node.parent = new_node
                return True
        return False

    def draw_graph(self):
        plt.clf()
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "g-")

        for (ox, oy, size) in self.obstacle_list:
            circle = plt.Circle((ox, oy), size, color='r')
            plt.gca().add_patch(circle)

        plt.plot(self.start.x, self.start.y, "bs")
        plt.plot(self.goal.x, self.goal.y, "rs")
        plt.axis("equal")
        plt.axis([self.min_x, self.max_x, self.min_y, self.max_y])
        plt.grid(True)
        plt.pause(0.01)

if __name__ == '__main__':
    # Define the start and goal positions
    start = (10, 10)
    goal = (90, 90)

    # Define the obstacle list as a list of tuples (x, y, radius)
    obstacle_list = [(30, 30, 10), (60, 30, 10), (30, 60, 10), (60, 60, 10)]

    # Define the random sampling area as a tuple of tuples ((x_min, x_max), (y_min, y_max))
    rand_area = ((0, 100), (0, 100))

    rrt_star = RRTStar(start, goal, obstacle_list, rand_area)
    rrt_star.planning()

    plt.show()
