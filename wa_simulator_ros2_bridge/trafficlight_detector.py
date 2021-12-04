'''
This node takes in image data and lidar point cloud to output traffic light
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud
from perception_msg.msg import TrafficLight, TrafficLightList
import time

class TrafficLightRecognitionNode(Node):
    def __init__(self):
        super().__init__('traffic_light_recognition_node')


        # frequency of publishing traffic light info
        self.frequency = 30.0


        # publishers and subscribers
        self.sub_image = self.create_subscription(
                Image, 'camera/image', self.image_callback, 15)
        self.sub_pointcloud = self.create_subscription(
                PointCloud, 'lidar/pointcloud', self.pointcloud_callback, 15)
        self.pub_trafficlights =  self.create_publisher(
                TrafficLightList, 'TrafficLightRecognitionNode/trafficlight', 15)
        self.timer = self.create_timer(1/self.frequency,self.pub_callback)

        # initialize models
    
    def calc_distance(self):
        pass


    def pointcloud_callback(self, msg):
        pass

    def image_callback(self, msg):

        # run the detection model to draw bounding box (Richard)

        # detect the state of the traffic light (Nevindu)

        pass


    def pub_callback(self):

        msg = TrafficLightList()


        self.pub_trafficlights.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__":
    main()
