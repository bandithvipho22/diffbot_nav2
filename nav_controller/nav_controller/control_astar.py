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

# lookahead_distance = 0.15
# speed = 0.1
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

# def pure_pursuit(current_x, current_y, current_heading, path,index):
#     global lookahead_distance
#     closest_point = None
#     v = speed
#     for i in range(index,len(path)):
#         x = path[i][0]
#         y = path[i][1]
#         distance = math.hypot(current_x - x, current_y - y)
#         if lookahead_distance < distance:
#             closest_point = (x, y)
#             index = i
#             break
#     if closest_point is not None:
#         target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
#         desired_steering_angle = target_heading - current_heading
#     else:
#         target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
#         desired_steering_angle = target_heading - current_heading
#         index = len(path)-1
#     if desired_steering_angle > math.pi:
#         desired_steering_angle -= 2 * math.pi
#     elif desired_steering_angle < -math.pi:
#         desired_steering_angle += 2 * math.pi
#     if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
#         sign = 1 if desired_steering_angle > 0 else -1
#         desired_steering_angle = sign * math.pi/4
#         v = 0.0
#     return v,desired_steering_angle,index

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
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            1)
        
        self.obstacle_publisher = self.create_publisher(Bool, 'obstacle_collision_with_path', 1) 
        self.path_publisher = self.create_publisher(Path, '/robot_path', 1)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # timer_period = 0.01
        # self.timer = self.create_timer(timer_period, self.timer_callback)
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

    # def goal_pose_callback(self,msg):
    #     self.goal = (msg.pose.position.x,msg.pose.position.y)
    #     print("Goal: ",self.goal[0],self.goal[1])
    #     self.flag = 1

    # def listener_callback(self,msg):
    #     if self.flag == 1:
    #         resolution = msg.info.resolution
    #         originX = msg.info.origin.position.x
    #         originY = msg.info.origin.position.y
    #         column = int((self.x- originX)/resolution) #x,y koordinatlarından costmap indislerine çevirme
    #         row = int((self.y- originY)/resolution) #x,y koordinatlarından costmap indislerine çevirme
    #         columnH = int((self.goal[0]- originX)/resolution)#x,y koordinatlarından costmap indislerine çevirme
    #         rowH = int((self.goal[1]- originY)/resolution)#x,y koordinatlarından costmap indislerine çevirme
    #         data = costmap(msg.data,msg.info.width,msg.info.height,resolution) #costmap düzenleme
    #         data[row][column] = 0 #robot konumu
    #         data[data < 0] = 1 
    #         data[data > 5] = 1 
    #         path = astar(data,(row,column),(rowH,columnH)) #astar algoritması ile yol bulma
    #         path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path] #x,y koordinatlarına çevirme
    #         self.path = bspline_planning(path,len(path)*5) #bspline ile düzeltme
    #         print("Robot Konumu: ",self.x,self.y)
    #         print("Hedefe ilerleniyor...")
    #         self.i = 0
    #         self.flag = 2
            
    # def laser_callback(self, msg):
    #     print("Laser received!")

    #     if self.flag == 1:
    #         resolution = msg.info.resolution
    #         originX = msg.info.origin.position.x
    #         originY = msg.info.origin.position.y
    #         column = int((self.x- originX)/resolution) #x,y koordinatlarından costmap indislerine çevirme
    #         row = int((self.y- originY)/resolution) #x,y koordinatlarından costmap indislerine çevirme
    #         columnH = int((self.goal[0]- originX)/resolution)#x,y koordinatlarından costmap indislerine çevirme
    #         rowH = int((self.goal[1]- originY)/resolution)#x,y koordinatlarından costmap indislerine çevirme
    #         data = costmap(msg.data,msg.info.width,msg.info.height,resolution) #costmap düzenleme
    #         data[row][column] = 0 #robot konumu
    #         data[data < 0] = 1 
    #         data[data > 5] = 1 
    #         path = astar(data,(row,column),(rowH,columnH)) #astar algoritması ile yol bulma
    #         path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path] #x,y koordinatlarına çevirme
    #         self.path = bspline_planning(path,len(path)*5) #bspline ile düzeltme
    #         print("Robot Konumu: ",self.x,self.y)
    #         print("Hedefe ilerleniyor...")
    #         self.i = 0
    #         self.flag = 2
       
        # max_range = 1.0

        # valid_ranges = (0.0 < np.array(msg.ranges)) & (np.array(msg.ranges) < max_range)
        # angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        # x = msg.ranges * np.cos(angles)
        # y = msg.ranges * np.sin(angles)
            
        # # Rotate the points based on the robot's current yaw
        # rotated_x = x * np.cos(self.yaw) - y * np.sin(self.yaw)
        # rotated_y = x * np.sin(self.yaw) + y * np.cos(self.yaw)
        
        # self.laser_ox, self.laser_oy = remove_close_obstacles(rotated_x[valid_ranges] + self.sx, rotated_y[valid_ranges] + self.sy, 0.1)
        
        # self.ox = np.concatenate((self.map_ox, self.laser_ox)) # combine multiple array
        # self.oy = np.concatenate((self.map_oy, self.laser_oy))
        # self.ox, self.oy = remove_close_obstacles(self.ox, self.oy, 0.1)
        # self.path.calc_obstacle_map(self.ox, self.oy)

        # # reshape these arrays to have a new axis -> self.rx[:, np.newaxis]
        # # detected obstacles & points in the environment obtained from laser -> self.laser_ox[1.1]

        # distances = np.sqrt((self.rx[:, np.newaxis] - self.laser_ox)**2 + (self.ry[:, np.newaxis] - self.laser_oy)**2)

        # obstacle_msg = Bool()
        # plt.cla() #clear current axes -> clear a plot before re-drawing or updating it with new data
        # show_animation = 1
        # if show_animation:  # pragma: no cover
        #     plt.plot(self.ox, self.oy, ".k")
        #     plt.plot(self.sx, self.sy, "og")
        #     plt.plot(self.gx, self.gy, "xb")
        #     plt.grid(True)
        #     plt.axis("equal")
        #     plt.plot(self.rx, self.ry, "-r")
        #     plt.pause(0.001)
        #     #plt.show()
        #     plt.draw()

        # if np.any(distances <= 0.2):
        #     obstacle_msg.data = True
        #     if self.last_time_obstacle_detected is None:
        #         self.last_time_obstacle_detected = time.perf_counter()
        # else:
        #     obstacle_msg.data = False
        #     self.last_time_obstacle_detected = None

        # if self.last_time_obstacle_detected and time.perf_counter() - self.last_time_obstacle_detected >= 5 or len(self.rx) < 2:
        #     self.get_path()
        #     self.last_time_obstacle_detected = None 
        
        # self.obstacle_publisher.publish(obstacle_msg)

    # def timer_callback(self):
    #     if self.flag == 2:
    #         twist = Twist()
    #         twist.linear.x , twist.angular.z,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,self.i)
    #         if(abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1])< 0.05):
    #             twist.linear.x = 0.0
    #             twist.angular.z = 0.0
    #             self.flag = 0
    #             print("Hedefe Ulasildi.\n")
    #             print("Yeni Hedef Bekleniyor..")
    #         self.publisher.publish(twist)
            #--------------------------
            # plt.cla()
            # plt.plot(self.cx, self.cy, ".r", label="course")
            # plt.plot(self.x, self.y, "-b", label="trajectory")
            # plt.
            
    #-----------------------------------------
    # def timer_callback(self):
    #     twist = Twist()
    #     err = 1
    #     if(self.flag==3):
    #         ai = 

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
