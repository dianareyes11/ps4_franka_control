#!/usr/bin/env python3
"""
Ultra-simple PS4 Gripper Controller
Only controls gripper with TRIANGLE button
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import threading

class SimpleGripperControl(Node):
    def __init__(self):
        super().__init__('simple_gripper_control')
        
        # PS4 controller subscription
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        
        # Gripper action client - try different topics
        self.gripper_client = ActionClient(
            self, GripperCommand, '/franka_gripper/gripper_action'
        )
        
        # Gripper state
        self.gripper_open = 0.04    # Open position (radians)
        self.gripper_close = 0.00   # Close position (radians)
        self.gripper_is_open = True # Start open
        self.gripper_busy = False
        self.last_button = 0
        
        self.get_logger().info("🎮 Simple Gripper Controller Started!")
        self.get_logger().info("Press TRIANGLE to toggle gripper")
        
        # Try to connect
        if self.gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info("✅ Gripper connected!")
        else:
            self.get_logger().warning("⚠️  Gripper not found, will try when button pressed")
    
    def joy_callback(self, msg):
        # TRIANGLE button is usually buttons[3]
        if len(msg.buttons) > 3:
            current = msg.buttons[3]
            # Press detected (0 -> 1)
            if current == 1 and self.last_button == 0:
                self.toggle_gripper()
            self.last_button = current
    
    def toggle_gripper(self):
        if self.gripper_busy:
            self.get_logger().info("Wait, gripper is moving...")
            return
            
        self.gripper_busy = True
        
        # Choose target position
        if self.gripper_is_open:
            target = self.gripper_close
            action = "CLOSE"
            self.gripper_is_open = False
        else:
            target = self.gripper_open
            action = "OPEN"
            self.gripper_is_open = True
        
        self.get_logger().info(f"🤏 Gripper: {action}")
        
        # Send command in background thread
        thread = threading.Thread(target=self._send_gripper, args=(target,))
        thread.daemon = True
        thread.start()
    
    def _send_gripper(self, target_position):
        try:
            # Connect if needed
            if not self.gripper_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("❌ Cannot connect to gripper")
                return
            
            # Create and send goal
            goal = GripperCommand.Goal()
            goal.command.position = target_position
            goal.command.max_effort = 40.0
            
            future = self.gripper_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                self.get_logger().info("✅ Gripper command sent")
            else:
                self.get_logger().warning("⚠️  Gripper command timeout")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
        finally:
            self.gripper_busy = False

def main():
    rclpy.init()
    node = SimpleGripperControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()