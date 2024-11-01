import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid , Odometry, Path
from geometry_msgs.msg import PoseStamped , Twist
import math
import scipy.interpolate as si
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

from scipy.spatial import KDTree

import matplotlib.pyplot as plt

from std_msgs.msg import Bool

import time

expansion_size = 2 #for the wall

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def bspline_planning(x, y, sn):
    N = 2
    t = range(len(x))
    x_tup = si.splrep(t, x, k=N)
    y_tup = si.splrep(t, y, k=N)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, sn)
    rx = si.splev(ipl_t, x_list)
    ry = si.splev(ipl_t, y_list)

    return rx, ry

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path


def remove_close_obstacles(obstacle_x_list, obstacle_y_list, min_distance=0.2):
    obstacle_points = np.column_stack((obstacle_x_list, obstacle_y_list))
    tree = KDTree(obstacle_points)
    unique_indices = list(tree.query_ball_tree(tree, r=min_distance))
    unique_indices = set([lst[0] for lst in unique_indices])
    new_obstacle_x = [obstacle_x_list[i] for i in unique_indices]
    new_obstacle_y = [obstacle_y_list[i] for i in unique_indices]
    return new_obstacle_x, new_obstacle_y

class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10))
        # self.laser_subscription = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.laser_callback,
        #     1)
        
        self.obstacle_publisher = self.create_publisher(Bool, 'obstacle_collision_with_path', 1) 
        self.path_publisher = self.create_publisher(Path, '/robot_path', 1)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        print("Start...")
        self.flag = 0
        
        self.goal = None
        self.x = 0
        self.y = 0
        self.flag = 0

    def goal_pose_callback(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        print("Goal: ", self.goal[0], self.goal[1])
        self.flag = 1

        
    def map_callback(self, msg):
        resolution = msg.info.resolution
        originX = msg.info.origin.position.x
        originY = msg.info.origin.position.y
        column = int((self.x - originX) / resolution)
        row = int((self.y - originY) / resolution)
        if self.flag == 1 and self.goal is not None:
            columnH = int((self.goal[0] - originX) / resolution)
            rowH = int((self.goal[1] - originY) / resolution)
            data = costmap(msg.data, msg.info.width, msg.info.height, resolution)
            data[row][column] = 0
            data[data < 0] = 1
            data[data > 5] = 1
            path = astar(data, (row, column), (rowH, columnH))
            path = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
            self.path = bspline_planning(path, len(path) * 5)
            print("Robot Konumu: ", self.x, self.y)
            print("Hedefe ilerleniyor...")
            self.flag = 2
            

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)


def main(args=None):
    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
