'''Create a node called controller, which publishes velocity commands on a topic 
called /cmd_vel of type geometry_msgs/msg/Twist at a frequency of 1 Hz. 
The robot always moves at 1 m/s, and its movement follows this rule:
1. N seconds along the X-axis
2. N seconds along the Y-axis
3. N seconds opposite the X-axis
4. N seconds opposite the Y-axis
N starts from 1 and increases by 1 after each set of movements.'''

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class controller(Node):
    
    def __init__(self):
        super().__init__("controller")
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period=1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.index=0
        self.count = 1
        self.local=0
    
    def timer_callback(self):
        msg = Twist()
        
            
        if self.index==0:  # for X forward
                if self.local < self.count: 
                    msg.linear.y = 0.0
                    msg.linear.x = 1.0
                    self.publisher_.publish(msg)
                    self.get_logger().info('Publishing: "%s"' % msg)
                    self.local+=1
                else :
                     
                    self.index +=1
                    self.index = self.index % 4
                    self.local=0
                

        elif self.index==1 :# for Y forward
            if self.local < self.count: 
                msg.linear.y = 1.0
                msg.linear.x= 0.0
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg)
                self.local+=1
            else :
                     
                self.index +=1
                self.index = self.index % 4
                self.local=0
        
        elif self.index==2:  # for X backward
            if self.local < self.count: 
                msg.linear.y = 0.0
                msg.linear.x = -1.0
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg)
                self.local+=1
            else :
                     
                self.index +=1
                self.index = self.index % 4
                self.local=0
        
        elif self.index==3:  # for Y backward
            if self.local < self.count: 
                msg.linear.y = -1.0
                msg.linear.x = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg)
                self.local+=1
            else :
                    
                self.index +=1
                self.index = self.index % 4
                self.local=0
                self.count+=1 
        
        

        



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


