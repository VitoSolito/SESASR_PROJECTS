import rclpy
from rclpy.node import Node
from math import sqrt

from geometry_msgs.msg import Pose 
from std_msgs.msg import Bool 

class reset(Node):
    def __init__(self):
        super().__init__("reset")

        self.subscription = self.create_subscription(
            Pose,
            '/pose',
            self.listener_callback,
            10 )
        
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)
        self.msg1 = Bool()
        self.msg1.data = True

    def listener_callback(self,msg):
        x_actual = msg.position.x
        y_actual = msg.position.y
        d = sqrt(x_actual**2 + y_actual**2)

        if d>6.0 : 
            self.publisher_.publish(self.msg1)
            self.get_logger().info('I publish: "%s"' % self.msg1.data)


def main(args=None):
    
    rclpy.init(args=args)

    res = reset()

    rclpy.spin(res)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    res.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
        