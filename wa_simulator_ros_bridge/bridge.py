# Import ROS specific modules
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from sensor_msgs.msg import Imu, NavSatFix

# Import WA Simulator
import wa_simulator as wa

# External imports
from typing import Dict, Callable, Any, List, NoReturn

class WASimulatorROS2Bridge(Node):

    def __init__(self):
        super().__init__('wa_simulator_ros_bridge')

        # Handles for publishers and subscribers
        # Publishers/subscribers will be added dynamically, based on the requirements of the simulation
        self.publisher_handles = {}
        self.subscriber_handles = {}    

        # Handles for callbacks
        # Used whenever a message is received
        self.message_callbacks = {}

        # -----------------------
        # Simulation bridge setup
        # -----------------------
        # Create a system
        self.system = wa.WASystem()

        # Create the bridge
        # The bridge is responsible for communicating the simulation
        self.bridge = wa.WABridge(self.system, hostname="0.0.0.0", port=5555, server=False)
        self.bridge.add_receiver(message_parser=self.message_callback)

        # Create a simulation wrapper
        # Will be responsible for actually running the simulation
        self.sim_manager = wa.WASimulationManager(self.system, self.bridge)

        # Periodic publishing
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        This callback will be called at 100hz. When a message is received from the simulation,
        the message will be converted to the ros equivalent and published.

        The first time a message is received with a specific name, it will generate a topic and a publisher.
        This will only happen if the conversion between the dict and the ros message is known.
        """
        
        # If the simulation isn't okay, shutdown
        if not self.sim_manager.is_ok():
            rclpy.shutdown()

        # Update the simulation to send and receive data
        self.sim_manager.synchronize(self.system.time)
        self.sim_manager.advance(self.system.step_size)

    def message_callback(self, name: str, message: dict):
        """
        Receives all messages from the simulation

        When a new message is received, i.e. one with a unique name, a subscriber will be created
        """
        msg_type = message["type"]
        data = message["data"]
        if name not in self.message_callbacks:
            if msg_type in self._publisher_creators and msg_type in self._message_callbacks:
                # Create the publishers
                publisher_creator = self._publisher_creators[msg_type]
                publisher_creator(self)

                # Add the receiver
                message_callback = self._message_callbacks[msg_type]
                self.bridge.add_receiver(name, self, message_parser=message_callback) 

                # Call the callback since this receiver was added after the message was actually received
                message_callback(self, message)
            else:
                raise RuntimeError(f"Could not infer publisher creator for message type {msg_type}.")
        else:
            # Sanity check
            raise RuntimeError(f"Global receiver callback was called. This shouldn't be happening...")
        
    # -----------------------------
    # Inferrable publisher creators
    # -----------------------------
    _publisher_creators: Dict[str, Callable[['WASimulatorROS2Bridge'], NoReturn]] = {}
            
    def _create_publisher_WASystem(self):
        global Time, Int32
        from builtin_interfaces.msg import Time
        from std_msgs.msg import Int32

        self.publisher_handles["time"] = self.create_publisher(Time, "time", 1)
        self.publisher_handles["step_number"] = self.create_publisher(Int32, "step_number", 1)
    _publisher_creators["WASystem"] = _create_publisher_WASystem

    def _create_publisher_WAVehicle(self):
        global Vector3, Quaternion
        from geometry_msgs.msg import Vector3, Quaternion

        self.publisher_handles["position"] = self.create_publisher(Vector3, "position", 1)
        self.publisher_handles["rotation"] = self.create_publisher(Quaternion, "rotation", 1)
        self.publisher_handles["linear_velocity"] = self.create_publisher(Vector3, "linear_velocity", 1)
        self.publisher_handles["angular_velocity"] = self.create_publisher(Quaternion, "angular_velocity", 1)
        self.publisher_handles["linear_acceleration"] = self.create_publisher(Vector3, "linear_acceleration", 1)
        self.publisher_handles["angular_acceleration"] = self.create_publisher(Quaternion, "angular_acceleration", 1)
    _publisher_creators["WAVehicle"] = _create_publisher_WAVehicle

    def _create_publisher_WAGPSSensor(self):
        global NavSatFix
        from sensor_msgs.msg import NavSatFix

        self.publisher_handles["gps"] = self.create_publisher(NavSatFix, "gps", 1)
    _publisher_creators["WAGPSSensor"] = _create_publisher_WAGPSSensor

    def _create_publisher_WAIMUSensor(self):
        global Imu
        from sensor_msgs.msg import Imu

        self.publisher_handles["imu"] = self.create_publisher(Imu, "imu", 1)
    _publisher_creators["WAIMUSensor"] = _create_publisher_WAIMUSensor
       
    # ----------------------------
    # Inferrable message callbacks
    # ----------------------------
    _message_callbacks: Dict[str, Callable[['WASimulatorROS2Bridge', dict], None]] = {}

    def _message_callback_WASystem(self, message: dict):
        data = message["data"]
        self.publisher_handles["time"].publish(self._time_to_Time(data["time"]))
        self.publisher_handles["step_number"].publish(Int32(data=data["step_number"]))
    _message_callbacks['WASystem'] = _message_callback_WASystem

    def _message_callback_WAVehicle(self, message: dict):
        data = message["data"]
        self.publisher_handles["position"].publish(self._WAVector_to_Vector3(data["position"]))
        self.publisher_handles["rotation"].publish(self._WAQuaternion_to_Quaternion(data["rotation"]))
        self.publisher_handles["linear_velocity"].publish(self._WAVector_to_Vector3(data["linear_velocity"]))
        self.publisher_handles["angular_velocity"].publish(self._WAQuaternion_to_Quaternion(data["angular_velocity"]))
        self.publisher_handles["linear_acceleration"].publish(self._WAVector_to_Vector3(data["linear_acceleration"]))
        self.publisher_handles["angular_acceleration"].publish(self._WAQuaternion_to_Quaternion(data["angular_acceleration"]))
    _message_callbacks['WAVehicle'] = _message_callback_WAVehicle

    def _message_callback_WAGPSSensor(self, message: dict):
        data = message["data"]
        self.publisher_handles["gps"].publish(self._WAGPSSensor_to_NavSatFix(**data))
    _message_callbacks['WAGPSSensor'] = _message_callback_WAGPSSensor

    def _message_callback_WAIMUSensor(self, message: dict):
        data = message["data"]
        self.publisher_handles["imu"].publish(self._WAIMUSensor_to_Imu(**data))
    _message_callbacks['WAIMUSensor'] = _message_callback_WAIMUSensor

    # ---------------------------------------------
    # WA Simulator data types to ROS msg converters
    # ---------------------------------------------

    def _time_to_Time(self, time: float):
        sec = int(time)
        return Time(sec=sec, nanosec=int((time-sec)*1e9)) 

    def _WAVector_to_Vector3(self, vec: wa.WAVector):
        return Vector3(x=vec.x, y=vec.y, z=vec.z)

    def _WAQuaternion_to_Quaternion(self, q: wa.WAQuaternion):
        return Quaternion(x=q.x, y=q.y, z=q.z, w=q.w)

    def _WAGPSSensor_to_NavSatFix(self, latitude: float, longitude: float, altitude: float):
        return NavSatFix(latitude=latitude, longitude=longitude, altitude=altitude)

    def _WAIMUSensor_to_Imu(self, linear_acceleration: wa.WAVector, angular_velocity: wa.WAQuaternion, orientation: wa.WAQuaternion):
        angular_velocity = wa.WAVector(angular_velocity.to_euler())
        return Imu(orientation=self._WAQuaternion_to_Quaternion(orientation), angular_velocity=self._WAVector_to_Vector3(angular_velocity), linear_acceleration=self._WAVector_to_Vector3(linear_acceleration))


def main(args=None):
    rclpy.init(args=args)

    bridge = WASimulatorROS2Bridge()

    rclpy.spin(bridge)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
