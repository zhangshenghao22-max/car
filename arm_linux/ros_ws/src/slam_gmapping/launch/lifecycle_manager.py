#!/usr/bin/env python3
"""
Simple lifecycle manager to activate lslidar_driver_node
"""
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lslidar_lifecycle_manager')
        self.client = self.create_client(ChangeState, 'lslidar_driver_node/change_state')
        self.activated = False
        
        # Wait for service to be available
        self.get_logger().info('Waiting for lslidar_driver_node lifecycle service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        # Configure the node
        self.get_logger().info('Configuring lslidar_driver_node...')
        if self.send_transition(Transition.TRANSITION_CONFIGURE):
            # Wait a bit, then activate
            self.timer = self.create_timer(2.0, self.activate_callback)
        else:
            self.get_logger().error('Failed to configure, exiting...')
            rclpy.shutdown()
    
    def activate_callback(self):
        if not self.activated:
            self.get_logger().info('Activating lslidar_driver_node...')
            if self.send_transition(Transition.TRANSITION_ACTIVATE):
                self.activated = True
                self.get_logger().info('LiDAR node activated successfully!')
                # Exit after successful activation
                self.timer.cancel()
                rclpy.shutdown()
            else:
                self.get_logger().error('Failed to activate')
    
    def send_transition(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f'Transition {transition_id} successful')
                return True
            else:
                self.get_logger().error(f'Transition {transition_id} failed: {result}')
                return False
        else:
            self.get_logger().error(f'Transition {transition_id} timed out')
            return False

def main():
    rclpy.init()
    node = LifecycleManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

