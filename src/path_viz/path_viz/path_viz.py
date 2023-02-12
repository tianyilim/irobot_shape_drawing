import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

'''
Simply draws Markers in RViz if the commanded state of "viz_on" is True.

The markers are drawn in the position reported on the "odom" topic.
'''

class PathSubscriber(Node):

    def __init__(self):
        super().__init__('path_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10  )

        self.viz_on = False

        self.check_viz = self.create_subscription(
            Bool,
            'viz_on',
            self.viz_callback,
            10  )

        self.marker_publisher = self.create_publisher(
            Marker,
            'traced_letters',
            10
        )
        self.marker_id = 0
        self.odom_cnt = 0
        self.MARKER_SPARSITY = 5   # how many 'odom' msgs apart to publish new breadcrumbs

    # Leaves breadcrumbs if required
    def listener_callback(self, msg:Odometry):
        # self.get_logger().info(f"At x: {msg.pose.pose.orientation.x:.2f} y: {msg.pose.pose.orientation.y:.2f}. Viz: {self.viz_on}")
        if self.viz_on:
            self.odom_cnt += 1
            if self.odom_cnt%self.MARKER_SPARSITY==0:
                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = msg.header.stamp
                marker.ns = "traced_letters"
                marker.id = self.marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = msg.pose.pose.position.x
                marker.pose.position.y = msg.pose.pose.position.y
                marker.pose.position.z = msg.pose.pose.position.z
                marker.pose.orientation.x = msg.pose.pose.orientation.x
                marker.pose.orientation.y = msg.pose.pose.orientation.y
                marker.pose.orientation.z = msg.pose.pose.orientation.z
                marker.pose.orientation.w = msg.pose.pose.orientation.w
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                # We get fully opaque green shapes.
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                self.marker_publisher.publish(marker)

                self.marker_id += 1

    def viz_callback(self, msg):
        self.viz_on = msg.data
        self.get_logger().info(f"Viz_on has been set to {self.viz_on}")


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = PathSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()