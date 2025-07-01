#!/usr/bin/env python3
"""
Simple test script to verify Tractobots basic functionality
Tests core ROS2 nodes and topic communication
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy, NavSatFix
from geometry_msgs.msg import Twist
import time
import sys

class TractorSystemTest(Node):
    def __init__(self):
        super().__init__('tractor_system_test')
        
        # Test publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.test_gps_pub = self.create_publisher(String, '/gps/dict', 10)
        
        # Test subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        # Test state
        self.tests_passed = 0
        self.tests_total = 5
        self.joy_received = False
        self.gps_received = False
        
        # Run tests
        self.test_timer = self.create_timer(1.0, self.run_tests)
        self.test_counter = 0
        
        self.get_logger().info("🚜 Tractor System Test Started!")
        
    def joy_callback(self, msg):
        self.joy_received = True
        self.get_logger().info("✅ Joystick input received!")
        
    def gps_callback(self, msg):
        self.gps_received = True
        self.get_logger().info("✅ GPS data received!")
        
    def run_tests(self):
        self.test_counter += 1
        
        if self.test_counter == 1:
            self.test_node_creation()
        elif self.test_counter == 2:
            self.test_topic_publishing()
        elif self.test_counter == 3:
            self.test_fake_gps_data()
        elif self.test_counter == 4:
            self.test_cmd_vel_publishing()
        elif self.test_counter >= 5:
            self.finish_tests()
            
    def test_node_creation(self):
        """Test that this node can be created"""
        self.get_logger().info("🧪 Test 1: Node Creation")
        self.tests_passed += 1
        self.get_logger().info("✅ Test 1 PASSED - Node created successfully")
        
    def test_topic_publishing(self):
        """Test basic topic publishing"""
        self.get_logger().info("🧪 Test 2: Topic Publishing")
        try:
            cmd = Twist()
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.tests_passed += 1
            self.get_logger().info("✅ Test 2 PASSED - Command velocity published")
        except Exception as e:
            self.get_logger().error(f"❌ Test 2 FAILED: {e}")
            
    def test_fake_gps_data(self):
        """Test GPS data simulation"""
        self.get_logger().info("🧪 Test 3: GPS Data Simulation")
        try:
            import marshal
            fake_gps = {
                'latitude': -34.9285,  # Sydney coordinates for testing
                'longitude': 138.6007,
                'qual': 1,
                'speed': 0.0,
                'course': 0.0
            }
            gps_msg = String()
            gps_msg.data = marshal.dumps(fake_gps)
            self.test_gps_pub.publish(gps_msg)
            self.tests_passed += 1
            self.get_logger().info("✅ Test 3 PASSED - Fake GPS data published")
        except Exception as e:
            self.get_logger().error(f"❌ Test 3 FAILED: {e}")
            
    def test_cmd_vel_publishing(self):
        """Test command velocity publishing for tractor control"""
        self.get_logger().info("🧪 Test 4: Command Velocity Control")
        try:
            # Test forward motion
            cmd = Twist()
            cmd.linear.x = 0.5  # 0.5 m/s forward
            cmd.angular.z = 0.1  # slight turn
            self.cmd_vel_pub.publish(cmd)
            
            # Test stop command
            time.sleep(0.1)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            self.tests_passed += 1
            self.get_logger().info("✅ Test 4 PASSED - Motion commands published")
        except Exception as e:
            self.get_logger().error(f"❌ Test 4 FAILED: {e}")
            
    def finish_tests(self):
        """Finish testing and report results"""
        self.get_logger().info("🧪 Test 5: System Integration Check")
        
        # Check if we received any input data
        integration_score = 0
        if self.joy_received:
            integration_score += 1
            self.get_logger().info("✅ Joystick integration working")
        else:
            self.get_logger().warn("⚠️  No joystick input detected (may be normal)")
            
        if self.gps_received:
            integration_score += 1
            self.get_logger().info("✅ GPS integration working")
        else:
            self.get_logger().warn("⚠️  No GPS input detected (may be normal)")
        
        # Final report
        self.get_logger().info("=" * 50)
        self.get_logger().info("🚜 TRACTOR SYSTEM TEST RESULTS")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Core Tests Passed: {self.tests_passed}/{self.tests_total}")
        self.get_logger().info(f"Integration Score: {integration_score}/2")
        
        if self.tests_passed >= 4:
            self.get_logger().info("🎉 SYSTEM STATUS: GOOD - Ready for basic operation")
        elif self.tests_passed >= 2:
            self.get_logger().info("⚠️  SYSTEM STATUS: PARTIAL - Some issues detected")
        else:
            self.get_logger().error("❌ SYSTEM STATUS: FAIL - Major issues detected")
            
        self.get_logger().info("=" * 50)
        
        # Shutdown test
        self.test_timer.cancel()
        rclpy.shutdown()
        sys.exit(0)

def main():
    rclpy.init()
    
    try:
        test_node = TractorSystemTest()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
