import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose 

class localization(Node):

    def __init__(self):
        super().__init__("localization")
        self.x_start=0
        self.y_start=0
        self.time = 1


        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10 )
        
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        



    def listener_callback(self, msg):
        msg1 = Pose()
        self.get_logger().info('I heard: "%s"' % msg)
        self.x_start+= msg.linear.x
        self.y_start+= msg.linear.y

        msg1.position.x = self.x_start
        msg1.position.y = self.y_start
        self.publisher_.publish(msg1)
        self.get_logger().info('I publish: "%s"' % msg1.position)
    

def main(args=None):
    rclpy.init(args=args)

    loc = localization()

    rclpy.spin(loc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    loc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        