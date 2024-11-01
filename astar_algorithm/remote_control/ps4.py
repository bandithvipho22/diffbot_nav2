import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTeleop(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist,'cmd_vel', 5)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.velocity = 0
        self.omega = 0

        self.subscription = self.create_subscription(Joy, 'joy',self.callback,5)
        self.subscription

    def callback(self,msg):
        v1 = msg.axes[1]
        w = msg.axes[3]
        self.velocity = float(v1)
        self.omega = float(w)
        self.get_logger().info("%f" %self.velocity + "%f" %self.omega)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.velocity)
        msg.angular.z = float(self.omega)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    joy_teleop = JoyTeleop()
    
    rclpy.spin(joy_teleop)

    joy_teleop.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()