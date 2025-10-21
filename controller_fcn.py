import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations

class controller(Node):
    
    def __init__(self):
        super().__init__("controller")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.goal=[6.5,3,0.5]
        self.parameters={"max_vel_ang":[0,0,1.5], "max_vel_lin":[0.22, 0, 0], "freq":5,
                         "max_angle":[-1.57,1.57], "threshold":0.5}
        self.timer_period = float(1/(self.parameters["freq"]));

        self.distances={}
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10 )
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan',self.scan_callback,10)
        self.boolean_flag = False
    
    def timer_callback(self):
        msg = Twist()
        if

    def odom_callback(self, msg):
        x,y,z = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        qx,qy,qz,qw = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])

    
    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        distances = {}
        angle_min_par=self.parameters["max_angle"][0]
        angle_max_par=self.parameters["max_angle"][1]

        for i in range(len(self.ranges)):
            angle = angle_min + i * angle_increment
            distance = self.ranges[i]
            if angle_min_par <= angle <= angle_max_par:
                distances[angle] = distance
        self.distances = distances


        self.get_logger().info('I heard scan with %d ranges' % len(ranges))

     
        

def main(args=None):
    rclpy.init(args=args)

    contr = controller()

    rclpy.spin(contr)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    contr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()