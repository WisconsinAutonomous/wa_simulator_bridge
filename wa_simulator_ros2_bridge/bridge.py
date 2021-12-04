# Import ROS specific modules
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType

# Import WA Simulator
import wa_simulator as wa

class WASimulatorROS2Bridge(Node):

    def __init__(self):
        super().__init__('wa_simulator_ros2_bridge')

        # Handles for publishers and subscribers
        # Publishers/subscribers will be added dynamically, based on the requirements of the simulation
        self.publisher_handles = {}
        self.subscriber_handles = {}    

        # -----------------------
        # Simulation bridge setup
        # -----------------------
        # Create a system
        self.system = wa.WASystem()

        # Create the bridge
        # The bridge is responsible for communicating the simulation
        self.bridge = wa.WABridge(system, hostname="0.0.0.0", port=5555, server=False)
        self.bridge.add_receiver(message_parser=self.message_callback)

        # Create a simulation wrapper
        # Will be responsible for actually running the simulation
        self.sim_manager = wa.WASimulationManager(system, bridge)

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
        sim_manager.synchronize(self.system.time)
        sim_manager.advance(self.system.step_size)

    def message_callback(self, message: dict):
        """
        Receives all messages from the simulation

        When a new message is received, i.e. one with a unique name, a subscriber will be created
        """
        print(message)



def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
