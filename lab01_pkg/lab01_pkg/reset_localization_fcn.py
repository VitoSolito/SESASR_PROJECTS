import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 
from std_msgs.msg import Bool

class reset_localization(Node):

    def __init__(self):
        super().__init__("reset_localization")
        self.x_start=0
        self.y_start=0
        self.time = 1


        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener1_callback,
            10 )
        
        self.subscription = self.create_subscription(
            Bool,
            '/reset',
            self.listener2_callback,
            10 )
        
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
    

    def listener2_callback(self,msg2):
        self.get_logger().info('I heard: "%s"' % msg2.data)
        self.x_start = 0
        self.y_start = 0
        self.get_logger().info('Resetted position: "x: %f ,  y: % f"' % (self.x_start,self.y_start))

    def listener1_callback(self, msg1):
        msg3 = Pose()
        self.get_logger().info('I heard: "%s"' % msg1)
        self.x_start+= msg1.linear.x
        self.y_start+= msg1.linear.y

        msg3.position.x = self.x_start
        msg3.position.y = self.y_start
        self.publisher_.publish(msg3)
        self.get_logger().info('I publish: "%s"' % msg3.position)
    

def main(args=None):
    rclpy.init(args=args)

    res_loc = reset_localization()

    rclpy.spin(res_loc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    res_loc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        