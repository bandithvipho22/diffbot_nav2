import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import math

import tf2_ros
from tf2_geometry_msgs import do_transform_point



# Define your constants
start_x = 0.0
start_y = 2.0
start_theta = 0.0

end_x = 0.0 
end_y = 0.0  
end_theta = 0.0

sampling_time = 0.08
sim_time = 50

dt = 0.0

w = np.pi / 2
theta = np.pi / 2

r = 0.05
d = 0.20
lx = 0.185

radius = 2 # for circle

class mecanum_wheel:

    def __init__ (self,r,lx,d):
        self.r = r
        self.lx = lx
        self.d = d
        
    def inverse_kinematic(self,vx,vy,w):
    
        v1 = (vx-vy-(self.lx+self.d)*w)/self.r
       
        v2 = (vx+vy+(self.lx+self.d)*w)/self.r
     
        v3 = (vx+vy-(self.lx+self.d)*w)/self.r
   
        v4 = (vx-vy+(self.lx+self.d)*w)/self.r
        
        return v1,v2,v3,v4
    
    def forward_kinematic(self,v1,v2,v3,v4):
     
        Vx=(v1 + v2 + v3 + v4)*self.r/4
  
        Vy=(- v1 + v2 + v3 - v4)*self.r/4

        Wz=(-v1 + v2 - v3 + v4)*self.r/(4*(self.lx+self.d))
        
        return Vx,Vy,Wz
    
    def discrete_state(self,x,y,theta,v1,v2,v3,v4,dt):
        
        dx,dy,dtheta=self.forward_kinematic(v1,v2,v3,v4)
       
        x_next = x + dx * dt
        y_next = y + dy * dt
        theta_next = theta + dtheta * dt

        return x_next, y_next, theta_next

class pid_controller:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0 

    def calculate_PID(self, error):
        proportional = self.kp * error[-1]

        integral = self.integral + self.ki * (error[-1]) * self.dt
        self.integral = integral

        derivative = self.kd * (error[-1] - error[-2]) / self.dt

        output = proportional + integral + derivative
        return output

class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        # Initialize ROS2 publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # Initialize tf2 Buffer and Transform Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize your other variables, PID controllers, and classes
        self.dt = 0.08
        self.sim_time = 50
        self.current_x = start_x
        self.current_y = start_y
        self.current_theta = start_theta

        self.pid_parameters = {
            'x': {'kp': 14.5, 'ki': 1.6, 'kd': 0.02},
            'y': {'kp': 9.5, 'ki': 0.5, 'kd': 0.1},
            'theta': {'kp': 0.5, 'ki': 0.2, 'kd': 0.0},
        }

        self.pid_x = pid_controller(**self.pid_parameters['x'], dt=self.dt)
        self.pid_y = pid_controller(**self.pid_parameters['y'], dt=self.dt)
        self.pid_theta = pid_controller(**self.pid_parameters['theta'], dt=self.dt)
        

    def calc_circular(self, radius, t, w, theta):
        # Your existing circular trajectory calculation
        x = radius * np.sin(w * t)
        y = radius * np.cos(w * t)
        theta = theta
        return x, y, theta

    
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # No PID control for simplicity
        v1, v2, v3, v4 = mecanum_wheel.inverse_kinematic(linear_x, 0.0, angular_z)

        # Publish computed velocities to cmd_vel topic
        twist = Twist()
        twist.linear.x = v1
        twist.linear.y = v2
        twist.linear.z = v3
        twist.angular.x = v4

        self.cmd_vel_publisher.publish(twist)



    def odom_callback(self, msg):

        try:
            # Transform from base_link to odom
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(seconds=msg.header.stamp.sec),
                rclpy.duration.Duration(seconds=msg.header.stamp.nanosec))

            # Transform the pose from base_link to odom frame
            transformed_pose = do_transform_point(msg.pose.pose.position, transform)

            # Extract position and orientation from transformed pose
            x = transformed_pose.point.x
            y = transformed_pose.point.y

            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )

            euler = self.quaternion_to_euler(quaternion)
            theta = euler[2]  # Assuming the z-axis rotation represents the orientation

            # Update robot's state variables
            self.current_x = x
            self.current_y = y
            self.current_theta = theta

        except Exception as e:
            self.get_logger().error(f"Failed to process odometry data: {e}")

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw
    
    def plot_arrow(self, x, y, theta, length=0.025, width=0.3, fc="b", ec="k"):
        if not isinstance(x, float):
            for (ix, iy, itheta) in zip(x, y, theta):
                self.plot_arrow(ix, iy, itheta)
        else:
            plt.arrow(
                x, y, length * np.cos(theta), length * np.sin(theta),
                fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)



def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
