import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations
from math import sqrt
import numpy as np
import math
class controller(Node):
    
    def __init__(self):
        super().__init__("controller")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.goal=[6.5,3,0.5]
        self.parameters={"max_vel_ang":[0,0,0.5], "max_vel_lin":[0.02, 0, 0], "freq":5.0,
                         "max_angle":[1.57,4.71], "threshold":0.25}
        self.timer_period = float(1/(self.parameters["freq"]))
        self.distance_towards=0.0
        self.distances={}
        self.position=[0,0,0]
        self.reached=False
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.subscription = self.create_subscription(Odometry,'/odom',
            self.odom_callback,10 )
        
        self.avgs=[0.0,0.0]
        self.subscription = self.create_subscription(
            LaserScan, '/scan',self.scan_callback,10)
        self.flag_adjust=[False,False]
        

    
    def timer_callback(self):
        msg = Twist()
        if self.reached==False:
            if self.distance_towards > self.parameters["threshold"]:
                msg.angular.z = 0.0
                msg.linear.x=self.parameters["max_vel_lin"][0]

                if self.flag_adjust[0]==True:
                    msg.angular.z = -0.2
                    self.flag_adjust[0]=False
                    msg.linear.x=self.parameters["max_vel_lin"][0]*2/3
                if self.flag_adjust[1]==True:
                    msg.angular.z = 0.2
                    self.flag_adjust[1]=False
                    msg.linear.x=self.parameters["max_vel_lin"][0]*2/3

            else:
                if self.avgs[0]>self.avgs[1]:
                    msg.angular.z = self.parameters["max_vel_ang"][2]
                    msg.linear.x=0.0
                else:
                    msg.angular.z = -self.parameters["max_vel_ang"][2]
                    msg.linear.x=0.0
        else:
            msg.angular.z = 0.0
            msg.linear.x=0.0
            self.get_logger().info("Goal Reached!")

        self.publisher_.publish(msg)
            #self.get_logger().info(msg)
        self.get_logger().info(f"Linear vel : {msg.linear.x} {msg.linear.y} {msg.linear.z}")
        self.get_logger().info(f"Angular vel : {msg.angular.x} {msg.angular.y} {msg.angular.z}")


    def odom_callback(self, msg):
        x,y,z = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        qx,qy,qz,qw = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
        self.position=[x,y,z]
        
        d_x = x - self.goal[0]
        d_y = y - self.goal[1]


        if sqrt(d_x**2 + d_y**2 )< 0.5:
            self.reached=True
        self.get_logger().info(f"Robot position : {x} {y} {z}")

    
    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        max_range = msg.range_max
        distances = {}
        n_left=0
        n_right=0
        sum_left=0.0
        sum_right=0.0

        for i in range(0,len(ranges)):
            angle = angle_min + i * angle_increment
            if math.isnan(ranges[i]==True): continue

            if np.isinf(ranges[i]) or ranges[i]>max_range:
                distance = max_range
            else:
                distance = ranges[i]
            
            if 0<=angle<=1.57 or 4.71<=angle<=6.28:
                distances[angle] = distance
                if i==0 : 
                    self.distance_towards = distance
                if 0<angle<=1.57: 
                    n_left +=1
                    sum_left+=distance
                    #mettere un flag per far virare lievemente a sinistra o destra se il lato del robot si avvicina troppo a un ostacolo
                    if ranges[i]<self.parameters["threshold"] and 3.14/6 <angle<=3.14/2:
                        self.flag_adjust[0]=True
                if 4.71<=angle<=6.28:
                    n_right +=1
                    sum_right+=distance
                    if ranges[i]<self.parameters["threshold"] and angle_max - 3.14/2 <=angle<= angle_max- 3.14/6:
                        self.flag_adjust[1]=True
        if n_left==0: 
            self.avgs[0]=0
        else :
            self.avgs[0]=sum_left/n_left
        if n_right==0: self.avgs[1]=0
        else :
            self.avgs[1]=sum_right/n_right
        self.distances = distances

        #self.get_logger().info(f"Distance from obstacle : {self.distance_towards}")
        #self.get_logger().info(f"Distance towards : {ranges[0]}")
        #self.get_logger().info(f"Distances : {distances.items()}")
        


        #self.get_logger().info('I heard scan with %d ranges' % len(ranges))

     
        

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
