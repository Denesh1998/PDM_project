# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import math
import random
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=4.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt path planning
        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y,
                                      self.node_list[-1].z) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d, dc1, dc2,dc3 = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]
        

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * dc1
            new_node.y += self.path_resolution * dc2
            new_node.z += self.path_resolution * dc3
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
            new_node.path_z.append(new_node.z)

            

        d, _, _,_ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.path_z.append(to_node.z)
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.z = to_node.z

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y,node.z])
            node = node.parent
        path.append([node.x, node.y,node.z])

        return path

    def calc_dist_to_goal(self, x, y, z):
        dx = x - self.end.x
        dy = y - self.end.y
        dz = z - self.end.z
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.z)
        return rnd

    def draw_graph(self, rnd=None):
        fig = plt.figure(1)

        plt.clf()
        ax = fig.gca(projection='3d')
        
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, rnd.z, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, node.path_z, "-g")

        for (ox, oy, oz, size) in self.obstacle_list:
            self.plot_circle(ox, oy, oz, size)

        plt.plot(self.start.x, self.start.y, self.start.z, "xr")
        plt.plot(self.end.x, self.end.y,self.end.z, "xr")

        plt.axis("auto")
        #plt.axis([-2, 15, -2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, z, size, color="-b"):  # pragma: no cover
        theta = list(range(0, 360, 5))
        phi = list(range(0, 360, 5))
        theta.append(0)
        phi.append(0)
        xl = x + .5*size * np.outer(np.cos(theta), np.sin(phi))
        yl = y + .5*size * np.outer(np.sin(theta), np.sin(phi))
        z1 = z + .5*size * np.outer(np.ones(np.size(theta)), np.cos(phi))
        #xl = [x + size * math.sin(np.deg2rad(d1))*math.cos(np.deg2rad(d2)) for (d1,d2) in zip(theta,phi)]
        #yl = [y + size * math.sin(np.deg2rad(d1))*math.sin(np.deg2rad(d2)) for (d1,d2) in zip(theta,phi)]
        #z1 = [z + size * math.cos(np.deg2rad(d)) for d in theta]
        #plt.plot(xl, yl, z1, color)
        fig = plt.figure(1)
        ax = fig.gca(projection='3d')
        ax.plot_surface(xl, yl, z1,  rstride=4, cstride=4, color='b')

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 + (node.z - rnd_node.z)**2 
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacleList):

        if node is None:
            return False

        for (ox, oy, oz, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            dz_list = [oz - z for z in node.path_z]
            d_list = [dx * dx + dy * dy + dz*dz for (dx, dy, dz) in zip(dx_list, dy_list, dz_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = np.sqrt(dx**2 + dy**2 + dz**2)
        dc1 = dx/d
        dc2 = dy/d
        dc3 = dy/d
        return d, dc1, dc2, dc3


def main(gx=4.0, gy=5.0, gz=1.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    #obstacleList = [(5, 5, 5, 1), (3, 6, 6, 2), (3, 8, 8, 2), (3, 10, 4, 2), (7, 5, 5, 2),
    #                (9, 5, 5, 2), (8, 10, 2, 1)]  # [x, y, z, radius]
    #obstacleList = [(1, 1, 3, 1), (4, 4, 4, 2), (6, 6, 6, 2), (8, 8, 8, 2), (1, 4, 4, 2),
    #                (2, 6, 8, 2), (8, 2, 3, 1)]  # [x, y, z, radius]
    obstacleList = [(1, 1, 3, 1)]  # [x, y, z, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0, 0],
        goal=[gx, gy, gz],
        rand_area=[-2, 10],
        obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            fig = plt.figure()
            ax = fig.gca(projection='3d')

            rrt.draw_graph()
            plt.plot([x for (x, y, z) in path], [y for (x, y, z) in path], [z for (x, y, z) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()