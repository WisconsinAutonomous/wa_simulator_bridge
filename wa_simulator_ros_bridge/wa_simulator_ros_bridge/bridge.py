#
# MIT License
#
# Copyright (c) 2018-2022 Wisconsin Autonomous
#
# See https://wa.wisc.edu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

# Import ROS specific modules
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Imu, NavSatFix

# Import WA Simulator
import wa_simulator as wa

# External imports
from typing import Dict, Callable, Any, List, NoReturn
import getpass

from wagrandprix_control_msgs.msg import SteeringCommand, BrakingCommand, ThrottleCommand

class WASimulatorROS2Bridge(Node):

    def __init__(self):
        super().__init__('wa_simulator_ros_bridge')

        self.logger = rclpy.logging.get_logger(self.get_name())

        # Handles for publishers and subscribers
        # Publishers/subscribers will be added dynamically, based on the requirements of the simulation
        self.publisher_handles = {}
        self.subscriber_handles = {}

        steering_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that SteeringCommand will be shipped on.")
        self.declare_parameter("steering_topic", "/control/steering", steering_descriptor)
        self.steering_topic = self.get_parameter("steering_topic").value
        
        braking_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that BrakingCommand will be shipped on.")
        self.declare_parameter("braking_topic", "/control/braking", braking_descriptor)
        self.braking_topic = self.get_parameter("braking_topic").value
        
        throttle_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description="The topic that ThrottleCommand will be shipped on.")
        self.declare_parameter("throttle_topic", "/control/throttle", throttle_descriptor)
        self.throttle_topic = self.get_parameter("throttle_topic").value

        # Handles for callbacks
        # Used whenever a message is received
        self.message_callbacks = {}

        # ---------------
        # Parse rosparams
        # ---------------
        # Port
        port_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER,
            description="The port the bridge will listen for information from the simulator on.")
        self.declare_parameter("port", value=5555, descriptor=port_descriptor)
        self.port = self.get_parameter("port").value

        # Host
        host_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="The host that the bridge will listen for information from.")
        self.declare_parameter("host", value=f"{getpass.getuser()}-wasim",
                               descriptor=host_descriptor)
        self.host = self.get_parameter("host").value

        # -----------------------
        # Simulation bridge setup
        # -----------------------
        # Create a system
        self.system = wa.WASystem()

        self.vehicle_inputs = wa.WAVehicleInputs(throttle=1)
        
        self.steering = 0
        self.throttle = 0
        self.braking = 0

        # Create the bridge
        # The bridge is responsible for communicating the simulation
        self.bridge = wa.WABridge(self.system, hostname=self.host, port=self.port, server=False)  # noqa

        # create subscribers
        self.subscriber_handles[self.steering_topic] = self.create_subscription(SteeringCommand, self.steering_topic, self._save_steering, 1)
        self.subscriber_handles[self.throttle_topic] = self.create_subscription(ThrottleCommand, self.throttle_topic, self._save_throttle, 1)
        self.subscriber_handles[self.braking_topic] = self.create_subscription(BrakingCommand, self.braking_topic, self._save_braking, 1)

        self.bridge.add_sender("vehicle_inputs", self.vehicle_inputs)


        self.bridge.add_receiver(message_parser=self.message_callback)

        # Create a simulation wrapper
        # Will be responsible for actually running the simulation
        self.sim_manager = wa.WASimulationManager(self.system, self.bridge)

        # Periodic publishing
        timer_period = 1e-9  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _save_steering(self, msg):
        self.steering = msg.value
    
    def _save_throttle(self, msg):
        self.throttle = msg.value

    def _save_braking(self, msg):
        self.braking = msg.value

    def timer_callback(self):
        """
        This callback will be called at 100hz. When a message is received from the simulation,
        the message will be converted to the ros equivalent and published.
        The first time a message is received with a specific name, it will generate a topic and a publisher.
        This will only happen if the conversion between the dict and the ros message is known.
        """

        # If the simulation isn't okay, shutdown
        if not self.sim_manager.is_ok():
            self.logger.warn("Sim manager is not okay. Something with the simulation has occured.")
            rclpy.shutdown()

        # Update vehicle inputs
        self.vehicle_inputs.steering = self.steering
        self.vehicle_inputs.throttle = self.throttle
        self.vehicle_inputs.braking = self.braking

        # Update the simulation to send and receive data
        self.sim_manager.synchronize(self.system.time)
        self.sim_manager.advance(self.system.step_size)

    def message_callback(self, name: str, message: dict):
        """
        Receives all messages from the simulation

        When a new message is received, i.e. one with a unique name, a publisher will be created
        """
        self.logger.info(f"Received message from {name}. Creating ROS publisher.")

        msg_type = message["type"]
        data = message["data"]
        if name not in self.message_callbacks:
            if msg_type in self._publisher_creators and msg_type in self._message_callbacks:
                # Create the publishers
                publisher_creator = self._publisher_creators[msg_type]
                publisher_creator(self)

                # Add the receiver
                message_callback = self._message_callbacks[msg_type]
                self.bridge.add_receiver(
                    name, self, message_parser=message_callback)

                # Call the callback since this receiver was added after the message was actually received
                message_callback(self, message)
            else:
                raise RuntimeError(
                    f"Could not infer publisher creator for message type {msg_type}.")
        else:
            # Sanity check
            raise RuntimeError(
                f"Global receiver callback was called. This shouldn't be happening...")

    # -----------------------------
    # Inferrable publisher creators
    # -----------------------------
    _publisher_creators: Dict[str, Callable[[
        'WASimulatorROS2Bridge'], NoReturn]] = {}

    def _create_publisher_WASystem(self):
        global Time, Int32
        from builtin_interfaces.msg import Time
        from std_msgs.msg import Int32

        self.publisher_handles["system/time"] = self.create_publisher(Time, "system/time", 1)
        self.publisher_handles["system/step_number"] = self.create_publisher(Int32, "system/step_number", 1)
    _publisher_creators["WASystem"] = _create_publisher_WASystem

    def _create_publisher_WAVehicle(self):
        global WAVehicleMsg, Accel, Twist, Pose, Vector3, Quaternion, Point
        from wa_simulator_ros_msgs.msg import WAVehicle as WAVehicleMsg
        from geometry_msgs.msg import Accel, Twist, Pose, Vector3, Quaternion, Point

        self.publisher_handles["vehicle/state"] = self.create_publisher(WAVehicleMsg, "vehicle/state", 1)
    _publisher_creators["WAVehicle"] = _create_publisher_WAVehicle

    def _create_publisher_WAGPSSensor(self):
        global NavSatFix
        from sensor_msgs.msg import NavSatFix

        self.publisher_handles["sensor/gps"] = self.create_publisher(NavSatFix, "sensor/gps", 1)
    _publisher_creators["WAGPSSensor"] = _create_publisher_WAGPSSensor

    def _create_publisher_WAIMUSensor(self):
        global Imu
        global Vector3, Quaternion
        from sensor_msgs.msg import Imu
        from geometry_msgs.msg import Vector3, Quaternion

        self.publisher_handles["sensor/imu"] = self.create_publisher(Imu, "sensor/imu", 1)
    _publisher_creators["WAIMUSensor"] = _create_publisher_WAIMUSensor

    def _create_publisher_WAWheelEncoderSensor(self):
        global Float64
        from std_msgs.msg import Float64

        self.publisher_handles["sensor/wheel_encoder"] = self.create_publisher(Float64, "sensor/wheel_encoder", 1)
    _publisher_creators["WAWheelEncoderSensor"] = _create_publisher_WAWheelEncoderSensor

    def _create_publisher_WATrack(self):
        global WATrackMsg, Point, Float64
        from wa_simulator_ros_msgs.msg import WATrack as WATrackMsg
        from geometry_msgs.msg import Point
        from std_msgs.msg import Float64

        self.publisher_handles["track/visible"] = self.create_publisher(WATrackMsg, "track/visible", 1)
        self.publisher_handles["track/mapped"] = self.create_publisher(WATrackMsg, "track/mapped", 1)
    _publisher_creators["WATrack"] = _create_publisher_WATrack

    def _create_publisher_WAMatplotlibVisualization(self):
        global Image
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        self.cvbridge = CvBridge()
        self.publisher_handles["visualization/matplotlib"] = self.create_publisher(Image, "visualization/matplotlib", 1)
    _publisher_creators["WAMatplotlibVisualization"] = _create_publisher_WAMatplotlibVisualization

    # ----------------------------
    # Inferrable message callbacks
    # ----------------------------
    _message_callbacks: Dict[str, Callable[['WASimulatorROS2Bridge', dict], None]] = {}

    def _message_callback_WASystem(self, message: dict):
        data = message["data"]
        self.publisher_handles["system/time"].publish(self._time_to_Time(data["time"]))
        self.publisher_handles["system/step_number"].publish(Int32(data=data["step_number"]))
    _message_callbacks['WASystem'] = _message_callback_WASystem

    def _message_callback_WAVehicle(self, message: dict):
        data = message["data"]

        vehicle_msg = WAVehicleMsg()
        vehicle_msg.accel = Accel()
        vehicle_msg.accel.linear = self._WAVector_to_Vector3(data["linear_acceleration"])
        vehicle_msg.accel.angular = self._WAVector_to_Vector3(wa.WAVector(list(data["angular_acceleration"].to_euler())))
        vehicle_msg.twist = Twist()
        vehicle_msg.twist.linear = self._WAVector_to_Vector3(data["linear_velocity"])
        vehicle_msg.twist.angular = self._WAVector_to_Vector3(wa.WAVector(list(data["angular_velocity"].to_euler())))
        vehicle_msg.pose = Pose()
        vehicle_msg.pose.position = self._WAVector_to_Point(data["position"])
        vehicle_msg.pose.orientation = self._WAQuaternion_to_Quaternion(data["rotation"])

        self.publisher_handles["vehicle/state"].publish(vehicle_msg)
    _message_callbacks['WAVehicle'] = _message_callback_WAVehicle

    def _message_callback_WAGPSSensor(self, message: dict):
        data = message["data"]
        self.publisher_handles["sensor/gps"].publish(
            self._WAGPSSensor_to_NavSatFix(**data))
    _message_callbacks['WAGPSSensor'] = _message_callback_WAGPSSensor

    def _message_callback_WAIMUSensor(self, message: dict):
        data = message["data"]
        self.publisher_handles["sensor/imu"].publish(self._WAIMUSensor_to_Imu(**data))
    _message_callbacks['WAIMUSensor'] = _message_callback_WAIMUSensor

    def _message_callback_WAWheelEncoderSensor(self, message: dict):
        data = message["data"]
        self.publisher_handles["sensor/wheel_encoder"].publish(Float64(data=float(data["angular_speed"])))
    _message_callbacks['WAWheelEncoderSensor'] = _message_callback_WAWheelEncoderSensor

    def _message_callback_WATrack(self, message: dict):
        data = message["data"]

        visible_track = WATrackMsg()

        visible = data["visible"]
        visible_track.left_visible_points = self._points_to_Point_list(visible["left"])
        visible_track.right_visible_points = self._points_to_Point_list(visible["right"])
        
        self.publisher_handles["track/visible"].publish(visible_track)

        mapped_track = WATrackMsg()

        mapped = data["mapped"]
        mapped_track.mapped_coords = self._points_to_Point_list(mapped["coords"])
        mapped_track.mapped_points = self._points_to_Point_list(mapped["points"])
        mapped_track.mapped_widths = self._points_to_Point_list(mapped["widths"])

        self.publisher_handles["track/mapped"].publish(mapped_track)

    _message_callbacks['WATrack'] = _message_callback_WATrack

    def _message_callback_WAMatplotlibVisualization(self, message: dict):
        data = message["data"]

        if len(data["image"]):
            image = self.cvbridge.cv2_to_imgmsg(data["image"])

            self.publisher_handles["visualization/matplotlib"].publish(image)
    _message_callbacks['WAMatplotlibVisualization'] = _message_callback_WAMatplotlibVisualization

    # ---------------------------------------------
    # WA Simulator data types to ROS msg converters
    # ---------------------------------------------

    def _time_to_Time(self, time: float):
        sec = int(time)
        return Time(sec=sec, nanosec=int((time-sec)*1e9))

    def _WAVector_to_Vector3(self, vec: wa.WAVector):
        return Vector3(x=vec.x, y=vec.y, z=vec.z)

    def _WAVector_to_Point(self, vec: wa.WAVector):
        return Point(x=vec.x, y=vec.y, z=vec.z)

    def _WAQuaternion_to_Quaternion(self, q: wa.WAQuaternion):
        return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

    def _WAGPSSensor_to_NavSatFix(self, latitude: float, longitude: float, altitude: float):
        return NavSatFix(latitude=latitude, longitude=longitude, altitude=altitude)

    def _WAIMUSensor_to_Imu(self, linear_acceleration: wa.WAVector, angular_velocity: wa.WAQuaternion, orientation: wa.WAQuaternion):
        angular_velocity = wa.WAVector(angular_velocity.to_euler())
        return Imu(orientation=self._WAQuaternion_to_Quaternion(orientation), angular_velocity=self._WAVector_to_Vector3(angular_velocity), linear_acceleration=self._WAVector_to_Vector3(linear_acceleration))

    def _points_to_PoseArray(self, points):
        return PoseArray(poses=[Pose(position=Point(x=x, y=y, z=z)) for x, y, z in points])

    def _points_to_Point_list(self, points):
        return [Point(x=x,y=y,z=z) for x, y, z in points]


def main(args=None):
    rclpy.init(args=args)

    bridge = WASimulatorROS2Bridge()

    rclpy.spin(bridge)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
