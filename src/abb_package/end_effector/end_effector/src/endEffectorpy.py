import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.timer import Timer
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future # For checking future completion

# Import your custom service message type
# Make sure 'abb_robot_msgs' is in your workspace and built
from abb_robot_msgs.srv import SetIOSignal

import time # For time.sleep if needed, though wait_for_service is non-blocking

class EndEffectorClient(Node):
    """
    A ROS 2 client node that periodically sends SetIOSignal service requests
    to toggle a gripper signal.
    """
    def __init__(self):
        """
        Constructor for EndEffectorClient.
        Initializes the node, creates a service client, and sets up a timer.
        """
        super().__init__('end_effector_client') # Initialize the ROS 2 node with the name 'end_effector_client'
        self.toggle_ = False # Initialize the toggle state for the signal value

        # Create a service client for the SetIOSignal service
        # The service name is '/rws_client/set_gripper_state'
        # The service type is SetIOSignal from abb_robot_msgs.srv
        self.client_: Client = self.create_client(SetIOSignal, '/rws_client/set_gripper_state')

        # Create a wall timer that calls the send_request method every 5 seconds
        self.timer_: Timer = self.create_timer(5.0, self.send_request) # 5.0 seconds

        self.get_logger().info("EndEffectorClient node initialized.")

    def send_request(self):
        """
        Method to send a SetIOSignal service request.
        It checks if the service is available, creates a request,
        sends it asynchronously, and processes the response.
        """
        # Wait for the service to be available. Timeout is 1 second.
        # This is a non-blocking wait because it's called within a timer callback
        # and the executor is spinning in a separate thread.
        if not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for service '/rws_client/set_gripper_state'...")
            return

        # Create a new request object for the SetIOSignal service
        request = SetIOSignal.Request()
        request.signal = "do_gripper"  # The name of the IO signal to control

        # Set the value of the signal based on the current toggle state
        # '1' for True, '0' for False
        request.value = "1" if self.toggle_ else "0"

        self.get_logger().info(f"Sending signal: {request.signal} = {request.value}")

        # Send the asynchronous service request
        # The result is a Future object
        self.result_future: Future = self.client_.call_async(request)

        # Add a done callback to process the result when it's ready
        # This prevents blocking the timer thread and allows the executor to manage the future
        self.result_future.add_done_callback(self.response_callback)

    def response_callback(self, future: Future):
        """
        Callback function executed when the service response is received.
        Processes the response and logs the outcome.
        """
        try:
            # Get the response from the future. This will raise an exception if the service call failed.
            response = future.result()

            # Check the result_code from the service response
            if response.result_code == 0:
                self.get_logger().info("Signal set successfully.")
            else:
                self.get_logger().error(f"Failed to set signal. Message: {response.message}")

        except Exception as e:
            # Handle exceptions during the service call (e.g., service not available, network issues)
            self.get_logger().error(f"Service call failed: {e}")

        # Toggle the state for the next request
        self.toggle_ = not self.toggle_

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args) # Initialize the ROS 2 client library

    # Create an instance of our EndEffectorClient node
    node = EndEffectorClient()

    # Create a MultiThreadedExecutor to spin the node
    # This allows the timer callback and the service response callback
    # to run concurrently if needed.
    executor = MultiThreadedExecutor()
    executor.add_node(node) # Add the node to the executor

    try:
        # Spin the executor indefinitely until the node is shut down
        executor.spin()
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.get_logger().info("Node stopped cleanly.")
    finally:
        # Shutdown the executor and then the ROS 2 client library
        executor.shutdown()
        node.destroy_node() # Destroy the node explicitly
        rclpy.shutdown()

if __name__ == '__main__':
    main()