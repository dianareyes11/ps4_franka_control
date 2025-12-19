#!/usr/bin/env python3
"""
PS4 Franka Control Node

This node provides dual-mode control of a Franka Panda robot arm using a PS4 controller
via MoveIt Servo. It supports both translation and rotation control modes.

Author: Diana
License: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


class PS4FrankaControl(Node):
    """
    Main controller class for PS4-based Franka Panda arm control.
    
    This class handles:
    - PS4 controller input processing
    - MoveIt Servo configuration
    - Dual-mode control (translation/rotation)
    - Twist command generation for robot movement
    """
    
    def __init__(self):
        """
        Initialize the PS4 Franka controller node.
        
        Sets up subscribers, publishers, service clients, and control parameters.
        """
        super().__init__('ps4_franka_control')
        
        # ===========================================================================
        # ROS2 COMMUNICATION SETUP
        # ===========================================================================
        
        # Subscribe to PS4 controller Joy messages
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )
        
        # Publisher for servo twist commands
        self.servo_pub = self.create_publisher(
            TwistStamped, 
            '/servo_node/delta_twist_cmds', 
            10
        )
        
        # Service client for configuring MoveIt Servo command type
        self.servo_client = self.create_client(
            ServoCommandType, 
            '/servo_node/switch_command_type'
        )
        
        # ===========================================================================
        # CONTROL PARAMETERS
        # ===========================================================================
        
        self.linear_scale = 0.1  # Linear velocity scale (m/s)
        self.angular_scale = 0.6  # Angular velocity scale (rad/s)
        self.dead_zone = 0.15     # Dead zone threshold
        
        # ===========================================================================
        # STATE VARIABLES
        # ===========================================================================
        
        self.servo_configured = False  # Whether Servo is ready to accept commands
        self.config_attempted = False  # Whether we've tried to configure Servo
        self.control_mode = 0          # 0 = Translation, 1 = Rotation
        self.mode_changed = False      # Prevents rapid mode switching
        
        # ===========================================================================
        # INITIALIZATION COMPLETE
        # ===========================================================================
        
        self.get_logger().info("🎮 PS4 Franka Controller Started!")
        self.get_logger().info("Controls:")
        self.get_logger().info("  • CIRCLE: Activate Servo mode")
        self.get_logger().info("  • L1: Switch between Translation/Rotation modes")
        self.get_logger().info("  • Current mode: TRANSLATION")
        

    def configure_servo(self):
        """
        Configure MoveIt Servo to accept twist commands.
        """
        if self.config_attempted:
            return
            
        self.config_attempted = True
        self.get_logger().info("Configuring Servo for twist commands...")
        
        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Servo service not available!")
            return
            
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        
        future = self.servo_client.call_async(request)
        future.add_done_callback(self.servo_config_callback)
        

    def servo_config_callback(self, future):
        """
        Handle the response from Servo configuration service call.
        """
        try:
            response = future.result()
            if response.success:
                self.servo_configured = True
                self.get_logger().info("Servo configured for twist commands!")
            else:
                self.get_logger().error("Servo configuration failed!")
        except Exception as e:
            self.get_logger().error(f"Servo configuration error: {e}")
    

    def apply_dead_zone(self, value):
        """
        Apply dead zone to controller input to prevent drift.
        """
        if abs(value) < self.dead_zone:
            return 0.0
        else:
            return (value - self.dead_zone * (1.0 if value > 0 else -1.0)) / (1.0 - self.dead_zone)
        

    def joy_callback(self, msg):
        """
        Process incoming PS4 controller messages and generate robot commands.
        """
        # Button handling
        if len(msg.buttons) > 1 and msg.buttons[1] == 1 and not self.config_attempted:
            self.configure_servo()
            return
            
        if len(msg.buttons) > 4 and msg.buttons[4] == 1 and not self.mode_changed:
            self.mode_changed = True
            self.control_mode = 1 - self.control_mode
            mode_name = "ROTATION" if self.control_mode == 1 else "TRANSLATION"
            self.get_logger().info(f"Control mode changed to: {mode_name}")
        elif len(msg.buttons) > 4 and msg.buttons[4] == 0:
            self.mode_changed = False
            
        if not self.servo_configured:
            if not self.config_attempted:
                self.get_logger().info("Press CIRCLE to activate Servo mode", once=True)
            return
        
        # Extract raw axis values
        left_stick_x = msg.axes[0]
        left_stick_y = msg.axes[1]
        l2_trigger = msg.axes[2]
        right_stick_x = msg.axes[3]
        r2_trigger = msg.axes[5]
        
        # Apply dead zones
        left_stick_x = self.apply_dead_zone(left_stick_x)
        left_stick_y = self.apply_dead_zone(left_stick_y)
        right_stick_x = self.apply_dead_zone(right_stick_x)
        
        # Convert trigger values
        l2_trigger = (1.0 - l2_trigger) / 2.0
        r2_trigger = (1.0 - r2_trigger) / 2.0
        
        l2_trigger = 0.0 if l2_trigger < self.dead_zone else l2_trigger
        r2_trigger = 0.0 if r2_trigger < self.dead_zone else r2_trigger
        
        # Create twist message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "panda_link0"
        
        if self.control_mode == 0:
            # TRANSLATION MODE
            twist_msg.twist.linear.x = left_stick_y * self.linear_scale
            twist_msg.twist.linear.y = left_stick_x * self.linear_scale
            z_control = r2_trigger - l2_trigger
            twist_msg.twist.linear.z = z_control * self.linear_scale
            twist_msg.twist.angular.z = right_stick_x * self.angular_scale
        else:
            # ROTATION MODE
            twist_msg.twist.angular.y = left_stick_y * self.angular_scale
            twist_msg.twist.angular.x = -left_stick_x * self.angular_scale
            z_control = r2_trigger - l2_trigger
            twist_msg.twist.linear.z = z_control * self.linear_scale
            twist_msg.twist.angular.z = right_stick_x * self.angular_scale
        
        # Publish command
        self.servo_pub.publish(twist_msg)

        #i am trying to make a change 
        
        # Log significant movements
        if (abs(twist_msg.twist.linear.x) > 0.001 or 
            abs(twist_msg.twist.linear.y) > 0.001 or 
            abs(twist_msg.twist.linear.z) > 0.001 or 
            abs(twist_msg.twist.angular.x) > 0.001 or
            abs(twist_msg.twist.angular.y) > 0.001 or
            abs(twist_msg.twist.angular.z) > 0.001):
            
            mode_name = "ROTATION" if self.control_mode == 1 else "TRANSLATION"
            self.get_logger().info(
                f"[{mode_name}] Linear=({twist_msg.twist.linear.x:.3f}, "
                f"{twist_msg.twist.linear.y:.3f}, {twist_msg.twist.linear.z:.3f}) "
                f"Angular=({twist_msg.twist.angular.x:.3f}, "
                f"{twist_msg.twist.angular.y:.3f}, {twist_msg.twist.angular.z:.3f})"
            )


def main():
    """
    Main function to initialize and run the PS4 Franka control node.
    """
    rclpy.init()
    node = PS4FrankaControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("PS4 Franka controller shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
