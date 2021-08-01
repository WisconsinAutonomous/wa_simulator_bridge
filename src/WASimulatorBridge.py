#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#from geometry_msgs.msg import Quaternion, Vector3
import geometry_msgs.msg as geo_msg
from wa_simulator_bridge.msg import WAControlMsg, WAVehicleStateMsg
from sensor_msgs.msg import Imu, NavSatFix
import multiprocessing.connection as mp
from pyrr import Vector3, Quaternion
import numpy as np


class WASimulatorBridge(Node):

    def __init__(self):
        super().__init__('w_a_simulator_bridge')

        self.address = ("localhost", 5555)
        self.connection = mp.Client(self.address)
        
        # Storage for publisher handles
        self.publisher_handles = {}
        
        # Storage for subscriber handles
        self.subscriber_handles = {}    
        
        # Periodically check for messages from the simulator
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    
    def timer_callback(self):
        """
        For every message received from the simulator, publish to the corresponding ros topic
        """

        while self.connection.poll():
        
            msg = self.connection.recv()

            if(msg["type"] == "controller_ids"):
                # msg = {
                #     "type": "controller_ids",
                #     "data": {
                #         "ids": [controller_id for controller_id in self._external_controllers.keys()]
                #     }
                # }
                for controller_id in msg["data"]["ids"]:
                    topic = self._constructControllerTopic(controller_id)
                    
                    # Define a callback for this topic
                    def topic_callback_factory(controller_id):
                        return lambda msg: self.controller_callback(msg, controller_id)
                    
                    self.subscriber_handles[topic] = self.create_subscription(WAControlMsg, topic, topic_callback_factory(controller_id), 10)
            elif(msg["type"] == "vehicle_state"):
                # msg = {
                #     "type": "vehicle_state",
                #     "data": {
                #         "id": comp.external_id,
                #         "position": Vector3(comp.get_pos()),
                #         "rotation": Quaternion(comp.get_rot()),
                #         "linear_velocity": Vector3(comp.get_pos_dt()),
                #         "angular_velocity": Vector3(comp.get_rot_dt()),
                #         "linear_acceleration": Vector3(comp.get_pos_dtdt()),
                #         "angular_acceleration": Vector3(comp.get_rot_dtdt())
                #     }
                # }
                vehicle_id = msg["data"]["id"]
                position = msg["data"]["position"]
                rotation = msg["data"]["rotation"]
                linear_velocity = msg["data"]["linear_velocity"]
                angular_velocity = msg["data"]["angular_velocity"]
                linear_acceleration = msg["data"]["linear_acceleration"]
                angular_acceleration = msg["data"]["angular_acceleration"]

                topic = self._constructVehicleStateTopic(vehicle_id)

                # Get the publisher, or construct it
                if not topic in self.publisher_handles.keys():
                    self.publisher_handles[topic] = self.create_publisher(WAVehicleStateMsg, topic, 10)
                
                # Construct and publish the ros message
                publisher = self.publisher_handles[topic]
                ros_msg = WAVehicleStateMsg()
                ros_msg.position.x = position.x
                ros_msg.position.y = position.y
                ros_msg.position.z = position.z
                ros_msg.rotation.x = rotation.x
                ros_msg.rotation.y = rotation.y
                ros_msg.rotation.z = rotation.z
                ros_msg.rotation.w = rotation.w

                ros_msg.linear_velocity.x = linear_velocity.x
                ros_msg.linear_velocity.y = linear_velocity.y
                ros_msg.linear_velocity.z = linear_velocity.z
                ros_msg.angular_velocity.x = angular_velocity.x
                ros_msg.angular_velocity.y = angular_velocity.y
                ros_msg.angular_velocity.z = angular_velocity.z

                ros_msg.linear_acceleration.x = linear_acceleration.x
                ros_msg.linear_acceleration.y = linear_acceleration.y
                ros_msg.linear_acceleration.z = linear_acceleration.z
                ros_msg.angular_acceleration.x = angular_acceleration.x
                ros_msg.angular_acceleration.y = angular_acceleration.y
                ros_msg.angular_acceleration.z = angular_acceleration.z
                
                publisher.publish(ros_msg)

            elif(msg["type"] == "imu_sensor"):
                # msg = {
                #     "type": "imu_sensor",
                #     "data": {
                #         "id": sensor.external_id,
                #         "linear_acceleration": Vector3(linear_acceleration),
                #         "angular_velocity": Vector3(angular_velocity),
                #         "orientation": Quaternion(orientation)
                #     }
                # }
                sensor_id = msg["data"]["id"]
                linear_acceleration = msg["data"]["linear_acceleration"]
                angular_velocity = msg["data"]["angular_velocity"]
                orientation = msg["data"]["orientation"]

                topic = self._constructSensorTopic(sensor_id)

                # Get the publisher, or construct it
                if not topic in self.publisher_handles.keys():
                    self.publisher_handles[topic] = self.create_publisher(Imu, topic, 10)
                
                # Construct and publish the ROS message
                publisher = self.publisher_handles[topic]
                ros_msg = Imu()
                ros_msg.orientation.x = orientation.x
                ros_msg.orientation.y = orientation.y
                ros_msg.orientation.z = orientation.z
                ros_msg.linear_acceleration.x = linear_acceleration.x
                ros_msg.linear_acceleration.y = linear_acceleration.y
                ros_msg.linear_acceleration.z = linear_acceleration.z
                ros_msg.angular_velocity.x = angular_velocity.x
                ros_msg.angular_velocity.y = angular_velocity.y
                ros_msg.angular_velocity.z = angular_velocity.z

                publisher.publish(ros_msg)

            elif(msg["type"] == "gps_sensor"):
                # msg = {
                #     "type": "gps_sensor",
                #     "data": {
                #         "id": sensor.external_id,
                #         "coordinates": Vector3(sensor.get_data())
                #     }
                # }
                sensor_id = msg["data"]["id"]
                coordinates = msg["data"]["coordinates"]

                topic = self._constructSensorTopic(sensor_id)

                # Get the publisher, or construct it
                if not topic in self.publisher_handles.keys():
                    self.publisher_handles[topic] = self.create_publisher(NavSatFix, topic, 10)
                
                publisher = self.publisher_handles[topic]
                ros_msg = NavSatFix()
                ros_msg.latitude = coordinates.y
                ros_msg.longitude = coordinates.x
                ros_msg.altitude = coordinates.z

                publisher.publish(ros_msg)


    def _constructVehicleStateTopic(self, id):
        return f"vehicle_state/{id}"
    
    def _constructControllerTopic(self, id):
        return f"controller/{id}"
    
    def _constructSensorTopic(self, id):
        return f"sensor/{id}"
        
    def controller_callback(self, msg, controller_id):
        """
        General callback for all of the subscriptions. Sends the control messages on to the simulator.
        """
        sim_msg = {
            "type": "vehicle_control",
            "data": {
                "id": controller_id,
                "steering": msg.steering,
                "throttle": msg.throttle,
                "braking": msg.braking
            }
        }
        self.connection.send(sim_msg)
        

        

def main(args=None):
    rclpy.init(args=args)

    w_a_simulator_bridge = WASimulatorBridge()

    rclpy.spin(w_a_simulator_bridge)

    rclpy.shutdown()


if __name__ == '__main__':
    main() 
