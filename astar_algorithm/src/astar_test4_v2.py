import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

import matplotlib.pyplot as plt

from nav_msgs.msg import Path, OccupancyGrid, Odometry

import numpy as np
from PIL import Image
from geometry_msgs.msg import PoseStamped, Twist
from tf2_msgs.msg import TFMessage

show_animation = True

def convert_to_obstacle_coordinates(input_path):
    try:
        # Open the image
        img = Image.open(input_path)

        # Convert to grayscale if not already
        img = img.convert("L")
        
        # Get the image size
        width, height = img.size

        # List to store obstacle coordinates
        obstacle_coordinates = []

        # Iterate through pixels and find obstacle coordinates
        for y in range(height):
            for x in range(width):
                pixel_value = img.getpixel((x, y))
                if pixel_value < 200:  # Check for non-white pixel
                    obstacle_coordinates.append((x, y))
        
        return obstacle_coordinates
    
    except Exception as e:
        print("An error occurred:", str(e))
        return None

# Provide the path to the input grayscale image
input_image_path = "/home/viphu//dev_ws/src/robot_differential/map/report_map_1.jpg"

# Get the obstacle coordinates
obstacle_coordinates = convert_to_obstacle_coordinates(input_image_path)

if obstacle_coordinates is not None:
    print("Obstacle coordinates generated successfully:")
    #for coord in obstacle_coordinates:
    #    print(coord)

    # Extract x and y coordinates for plotting
    x_coords, y_coords = zip(*obstacle_coordinates)

    # Create a scatter plot using Matplotlib
    #plt.scatter(x_coords, y_coords, s=10, c='red', marker='s')
    #plt.title('Obstacle Coordinates')
    #plt.xlabel('X')
    #plt.ylabel('Y')
    #plt.gca().invert_yaxis()  # Invert y-axis to match image coordinates
    #plt.show()

print(np.array(obstacle_coordinates).shape)

obstacle = np.array(obstacle_coordinates)


class AStarPlannerNode(Node):

    def __init__(self):
        super().__init__('astar_planner_node')
        super().__init__('path_publisher')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.goal_subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callbacK, 1)
        #self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            1)
        self.path_publisher = self.create_publisher(Path, '/path', 1)
        self.p_controller = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.path = Path()

        self.sx = -2.0  # [m]
        self.sy = -0.5  # [m]
        self.gx = 0.7 # [m]
        self.gy = 0.7  # [m]
        self.grid_size = 0.5 # [m]
        self.robot_radius = 0.7 # [m]

        self.ox = ((obstacle[:,0] * 0.05)) - 9.32
        self.oy = -(((obstacle[:,1] * 0.05)) - 6.68)

        self.yaw = 0.0

    
        # self.a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
        # self.rx, self.ry = self.a_star.planning(self.sx, self.sy, self.gx, self.gy)

        self.ready = False

    def map_callback(self, msg):
        # Process the received map data
        self.map_data = msg.data

        # Retrieve parameter values
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # Create a publisher for the path
        self.path_publisher = self.create_publisher(Path, 'path', 10)

        # Run the A* planner
        self.run_planner()

    class AStarPlanner:

        def __init__(self, map_data, start_x, start_y, goal_x, goal_y, resolution, rr):
            # Initialize A* planner with map data and other parameters
            self.map_data = map_data
            self.start_x = start_x
            self.start_y = start_y
            self.goal_x = goal_x
            self.goal_y = goal_y
            self.resolution = resolution
            self.rr = rr
            self.min_x, self.min_y = 0, 0
            self.max_x, self.max_y = 0, 0
            self.x_width, self.y_width = 0, 0
            self.motion = self.get_motion_model()
            self.calc_obstacle_map()

        # --------------------------------------------------------------
        class Node:
            def __init__(self, x, y, cost, parent_index):
                self.x = x  # index of grid
                self.y = y  # index of grid
                self.cost = cost
                self.parent_index = parent_index
            # ... (The rest of your AStarPlanner methods here)
            def __str__(self):
                    return str(self.x) + "," + str(self.y) + "," + str(
                    self.cost) + "," + str(self.parent_index)
            
        def planning(self, sx, sy, gx, gy):
            """
            A star path search

            input:
                s_x: start x position [m]
                s_y: start y position [m]
                gx: goal x position [m]
                gy: goal y position [m]

            output:
                rx: x position list of the final path
                ry: y position list of the final path
            """

            start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                                self.calc_xy_index(sy, self.min_y), 0.0, -1)
            goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                                self.calc_xy_index(gy, self.min_y), 0.0, -1)

            open_set, closed_set = dict(), dict()
            open_set[self.calc_grid_index(start_node)] = start_node

            while True:
                if len(open_set) == 0:
                    print("Open set is empty..")
                    break

                c_id = min(
                    open_set,
                    key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                        open_set[
                                                                            o]))
                current = open_set[c_id]
                print("c_id", c_id)
                #----------------------------------------------------------------------------
                # show graph
                if show_animation:  # pragma: no cover
                    plt.plot(self.calc_grid_position(current.x, self.min_x),
                            self.calc_grid_position(current.y, self.min_y), "xc")
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                lambda event: [exit(
                                                    0) if event.key == 'escape' else None])
                    if len(closed_set.keys()) % 10 == 0:
                        plt.pause(0.001)

                if current.x == goal_node.x and current.y == goal_node.y:
                    print("Find goal")
                    goal_node.parent_index = current.parent_index
                    goal_node.cost = current.cost
                    break

                # Remove the item from the open set
                del open_set[c_id]

                # Add it to the closed set
                closed_set[c_id] = current

                # expand_grid search grid based on motion model
                for i, _ in enumerate(self.motion):
                    node = self.Node(current.x + self.motion[i][0],
                                    current.y + self.motion[i][1],
                                    current.cost + self.motion[i][2], c_id)
                    n_id = self.calc_grid_index(node)

                    # If the node is not safe, do nothing
                    if not self.verify_node(node):
                        continue

                    if n_id in closed_set:
                        continue

                    if n_id not in open_set:
                        open_set[n_id] = node  # discovered a new node
                    else:
                        if open_set[n_id].cost > node.cost:
                            # This path is the best until now. record it
                            open_set[n_id] = node

            rx, ry = self.calc_final_path(goal_node, closed_set)

            return rx, ry

        def calc_final_path(self, goal_node, closed_set):
            # generate final course
            rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
                self.calc_grid_position(goal_node.y, self.min_y)]
            parent_index = goal_node.parent_index
            while parent_index != -1:
                n = closed_set[parent_index]
                rx.append(self.calc_grid_position(n.x, self.min_x))
                ry.append(self.calc_grid_position(n.y, self.min_y))
                parent_index = n.parent_index

            return rx, ry

        @staticmethod
        def calc_heuristic(n1, n2):
            w = 1.0  # weight of heuristic
            d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
            return d

        def calc_grid_position(self, index, min_position):
            """
            calc grid position

            :param index:
            :param min_position:
            :return:
            """
            pos = index * self.resolution + min_position
            return pos

        def calc_xy_index(self, position, min_pos):
            return round((position - min_pos) / self.resolution)

        def calc_grid_index(self, node):
            return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

        def verify_node(self, node):
            px = self.calc_grid_position(node.x, self.min_x)
            py = self.calc_grid_position(node.y, self.min_y)

            if px < self.min_x:
                return False
            elif py < self.min_y:
                return False
            elif px >= self.max_x:
                return False
            elif py >= self.max_y:
                return False

            # collision check
            if self.obstacle_map[node.x][node.y]:
                return False

            return True
        
        # Calculate obstacle map
        def calc_obstacle_map(self, ox, oy):
    
            self.min_x = round(min(ox))
            self.min_y = round(min(oy))
            self.max_x = round(max(ox))
            self.max_y = round(max(oy))
            print("min_x:", self.min_x)
            print("min_y:", self.min_y)
            print("max_x:", self.max_x)
            print("max_y:", self.max_y)

            self.x_width = round((self.max_x - self.min_x) / self.resolution)
            self.y_width = round((self.max_y - self.min_y) / self.resolution)
            print("x_width:", self.x_width)
            print("y_width:", self.y_width)

            # obstacle map generation
            self.obstacle_map = [[False for _ in range(self.y_width)] 
                                for _ in range(self.x_width)]
            #---------------------------------------------------------------
            for ix in range(self.x_width):
                x = self.calc_grid_position(ix, self.min_x)
                for iy in range(self.y_width):
                    y = self.calc_grid_position(iy, self.min_y)
                    for iox, ioy in zip(ox, oy):
                        d = math.hypot(iox - x, ioy - y)
                        if d <= self.rr:
                            self.obstacle_map[ix][iy] = True
                            break
        
        @staticmethod
        def get_motion_model():
            # dx, dy, cost
            motion = [[1, 0, 1],
                    [0, 1, 1],
                    [-1, 0, 1],
                    [0, -1, 1],
                    [-1, -1, math.sqrt(2)],
                    [-1, 1, math.sqrt(2)],
                    [1, -1, math.sqrt(2)],
                    [1, 1, math.sqrt(2)]]

            return motion
    #------------------------------------------------------
    def run_planner(self):
        if self.map_data is None:
            self.get_logger().warn('No map data received.')
            return

        # A* grid planning code using the map data
        a_star = self.AStarPlanner(
            map_data=self.map_data,
            start_x=self.start_x,
            start_y=self.start_y,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            resolution=self.grid_resolution,
            rr=self.robot_radius
        )

        # Run the A* planner
        rx, ry = a_star.planning()

        # Create and publish the path as a Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        #----------------------------------------------
        for x, y in zip(rx, ry):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
      
        self.path_publisher.publish(path_msg)
        self.get_logger().info('A* path published.')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

    # map_subscriber = AStarPlannerNode()
    # rclpy.spin(map_subscriber)
    # map_subscriber.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
