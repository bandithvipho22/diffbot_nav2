import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import PoseStamped , Twist
import math
import scipy.interpolate as si
from rclpy.qos import QoSProfile

import matplotlib.pyplot as plt

lookahead_distance = 0.15
speed = 0.5
expansion_size = 2 #for the wall

#Parameters
k = 0.1 # Look forward gain
Lfc = 2.0 # [m] look-ahead distance
kp = 2.0 #speed proportional gain
dt = 0.1 #[s] time tick
WB = 2.9 #[m] wheel base of vehicle


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

def calc_target_index(state, cx, cy):
    old_nearest_point_index = None
    
    if old_nearest_point_index is None:
            #search nearest point index
            dx = [state.rear_x - icx for icx in cx]
            dy = [state.rear_y - icy for icy in cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            old_nearest_point_index = ind
    else:
        ind = old_nearest_point_index
        distance_this_index = state.calc_distance(cx[ind],
                                                      cy[ind])

        while True: 
            distance_next_index = state.calc_distance(cx[ind + 1],
                                                          cy[ind + 1])
            if distance_this_index < distance_next_index:
                break
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            distance_this_index = distance_next_index
        
        old_nearest_point_index = ind
            
    Lf = k * state.v + Lfc #Update lood ahead distance
        
        #search look ahead target point index
    while Lf > state.calc_distance(cx[ind], cy[ind]):
        if (ind + 1) >= len(cx):
            break #not exceed goal
            
        ind += 1
    return ind, Lf



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

def plot_arrow(x, y, yaw, length = 1.0, width= 0.5, fc="r", ec="k"):
    """ 
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def PControl(target, current):
    a = kp * (target - current)
    
    return a

# Modify the pure_pursuit_control method
def pure_pursuit_control(state, cx, cy, pind):
    ind, Lf = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:  # toward goal
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

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

def createM(height, width, arr):
    costmap_mat = np.ones([height, width])
    for i in range(0, height):
        for j in range(0, width):
            if(arr[(i*width)+j]==100):
                costmap_mat[i][j] = 1
                t = 2
                for k in range(2*t+1):
                    try:
                        costmap_mat[i+k-t][j+1-t] = 1
                    except:
                        pass
            else:
                costmap_mat[i][j] = 0
        
    return costmap_mat


class navigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')
        self.state = State()
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.listener_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.info_callback,10)
        self.subscription = self.create_subscription(PoseStamped,'goal_pose',self.goal_pose_callback,QoSProfile(depth=10))
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.flag = 0
        self.ratio = 20
        print("wait for the target...")

    def goal_pose_callback(self,msg):
        self.goal = (msg.pose.position.x,msg.pose.position.y)
        self.flag = 1
        
        print("Target or goal: ",self.goal[0],self.goal[1])

    def listener_callback(self,data):
        self.width = data.info.width
        self.height = data.info.height
        arr = data.data
        if(self.flag==1):
            self.start = (int((self.height/2)-(self.xA)*self.ratio), int((self.width/2)+(self.yA)*self.ratio))
            self.goal = (int((self.height/2) - self.goal[0]*self.ratio), int((self.width/2) + self.goal[1]*self.ratio))
            grid = createM(self.height, self.width, arr)
            self.route = astar(grid, self.start, self.goal)
            self.route = self.route + [self.start]
            self.route = self.route[::-1]
            self.cx = []
            self.cy = []
            for i in range(len(self.route)):
                self.cx.append(self.route[i][0])
                self.cy.append(self.route[i][1])
            self.flag = 2

    def info_callback(self, msg):
        self.xA = msg.pose.pose.position.y
        self.yA = msg.pose.pose.position.x
        global yaw_z
        yaw_z = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) + math.pi/2
        if self.flag == 2:
            global xC, yC
            xC = (self.height / 2) - (self.xA) * self.ratio
            yC = (self.width / 2) + (self.yA) * self.ratio
            # self.state = plot_arrow(x=xC, y=yC, yaw=yaw_z, v=0.0)
            self.target_speed = 0.08
            self.target_ind = calc_target_index(self.state, self.cx, self.cy)
            self.x = [self.state.x]
            self.y = [self.state.y]
            self.yaw = [self.state.yaw]
            self.v = [self.state.v]
            self.flag = 3
        if self.flag == 3:
            xC = (self.height / 2) - (self.xA) * self.ratio
            yC = (self.width / 2) + (self.yA) * self.ratio
        
        
    # Correct the method call in the timer_callback method
    def timer_callback(self):
        twist = Twist()
        err = 1
        if self.flag == 3:
            ai = PControl(self.target_speed, self.state.v)
            # Modify the pure_pursuit_control method call parameters
            di, self.target_ind = pure_pursuit_control(self.state, self.cx, self.cy, self.target_ind)

            # Update the state using the State class's update method
            self.state.update(ai, di)  # Adjust the update logic if necessary

            twist.linear.x = self.state.v
            twist.angular.z = di / self.state.v  # Check if this calculation is appropriate
            self.publisher.publish(twist)
            self.x.append(self.state.x)
            self.y.append(self.state.y)
            self.yaw.append(self.state.yaw)
            self.v.append(self.state.v)

            # Visualization code - ensure proper plotting and updating
            plt.cla()
            plt.plot(self.cx, self.cy, ".r", label="course")
            plt.plot(self.x, self.y, "-b", label="trajectory")
            plt.plot(self.cx[self.target_ind], self.cy[self.target_ind], "go", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

        # Modify the condition for reaching the goal/target
        if self.flag == 3 and xC - err < self.goal[0] and xC + err > self.goal[0] and yC - err < self.goal[1] and yC + err > self.goal[1]:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            self.flag = 0
            print("Target reached.\n")


def main(args=None):

    rclpy.init(args=args)
    navigation_control = navigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
