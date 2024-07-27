import numpy as np
import matplotlib.pyplot as plt
import math
import random

class RRTStar:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.cost = 0.0
            self.parent = None

    def __init__(self, start, goal, obstacle_list, rand_area,
                 expand_dis=30.0, path_resolution=1.0, goal_sample_rate=20,
                 max_iter=6000, connect_circle_dist=5.0, robot_radius=4):
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = [self.start]
        self.connect_circle_dist = connect_circle_dist
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)

            if self.check_collision(new_node):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd)

            if new_node and self.calc_distance_to_goal(new_node.x, new_node.y) <= self.expand_dis:
                final_node = self.steer(new_node, self.goal)
                if self.check_collision(final_node):
                    return self.generate_final_course(len(self.node_list) - 1)
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.x += min(extend_length, d) * math.cos(theta)
        new_node.y += min(extend_length, d) * math.sin(theta)
        new_node.cost = from_node.cost + self.calc_distance_and_angle(from_node, new_node)[0]
        new_node.parent = from_node
        return new_node

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.goal.x, self.goal.y)
        return rnd

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def check_collision(self, node):
        for (ox, oy) in self.obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = math.hypot(dx, dy)
            if d <= self.robot_radius:
                return False
        return True

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node):
                costs.append(near_node.cost + self.calc_distance_and_angle(near_node, new_node)[0])
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"):
            return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost
        return new_node

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = new_node.cost + self.calc_distance_and_angle(new_node, near_node)[0]

            if self.check_collision(edge_node) and near_node.cost > edge_node.cost:
                near_node = edge_node
                self.propagate_cost_to_leaves(near_node)

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = parent_node.cost + self.calc_distance_and_angle(parent_node, node)[0]
                self.propagate_cost_to_leaves(node)

    def calc_distance_to_goal(self, x, y):
        return math.hypot(x - self.goal.x, y - self.goal.y)

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def draw_graph(self, rnd=None):
        plt.clf()
        if rnd:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")
        for (ox, oy) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms=30 * self.robot_radius)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)

def main():
    # Load map data
    data = np.load('Map/data.npy')

    start = (700, 400)
    goal = (700, 300)
    # waypoints = [(700, 711), (765, 694), (730, 400), (800, 195), (713, 95), (550, 80), (418, 65), (66, 103), (35, 218),
    #              (137, 366), (98, 524), (118, 710)]
    waypoints = [(698, 512), (710, 646), (689, 750), (570, 783), (531, 774), (536, 729), (496, 719), (449, 747), (375, 730), (302, 726), (225, 800), (133, 785), (76, 626), (57, 469), (60, 424), (57, 377), (86, 193), (102, 71), (216, 36), (281, 119), (330, 122), (386, 127), (436, 133), (481, 88), (524, 97), (551, 56), (679, 97), (708, 133)]

    # Flatten obstacles for easier access
    obstacle_list = [(x, y) for x in range(data.shape[0]) for y in range(data.shape[1]) if data[x, y] == 1]

    # Initialize path
    full_path = []
    current_start = start

    # Iterate over each waypoint including the final goal
    for waypoint in waypoints + [goal]:
        rrt_star = RRTStar(current_start, waypoint, obstacle_list, [0, max(data.shape[0], data.shape[1])])
        path = rrt_star.planning(animation=False)
        if path:
            full_path.extend(path[::-1] if full_path and path[0] == full_path[-1] else path)
            current_start = waypoint
        else:
            print("No path found to waypoint:", waypoint)
            return

    # Display the result
    if full_path:
        print("Found path:", full_path)
        # Visualize the path
        plt.imshow(data, cmap='Greys', origin='upper')
        for (i, j) in full_path:
            plt.plot(j, i, 'bo')  # plot path points
        plt.plot(start[1], start[0], 'go')  # plot start point
        plt.plot(goal[1], goal[0], 'ro')  # plot goal point
        for waypoint in waypoints:
            plt.plot(waypoint[1], waypoint[0], 'yo')  # plot waypoints
        plt.show()
    else:
        print("No path found")

if __name__ == '__main__':
    main()
