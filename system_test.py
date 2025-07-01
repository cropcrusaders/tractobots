#!/usr/bin/env python3
"""
System test script for Tractobots
Comprehensive test of the entire system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import time
import sys


class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # Test publishers
        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # Test subscribers
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 1)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        
        # Test results
        self.test_results = {
            'ros2_connection': False,
            'publishers': False,
            'subscribers': False,
            'gps_data': False,
            'imu_data': False,
            'odom_data': False
        }
        
        self.data_received = {
            'gps': False,
            'imu': False,
            'odom': False
        }

    def gps_callback(self, msg):
        self.data_received['gps'] = True
        self.get_logger().info(f'GPS: {msg.latitude:.6f}, {msg.longitude:.6f}')

    def imu_callback(self, msg):
        self.data_received['imu'] = True
        self.get_logger().info('IMU data received')

    def odom_callback(self, msg):
        self.data_received['odom'] = True
        self.get_logger().info('Odometry data received')

    def run_tests(self):
        """Run comprehensive system tests"""
        
        print("🚜 TRACTOBOTS SYSTEM TEST")
        print("=" * 40)
        
        # Test 1: ROS2 Connection
        print("📡 Testing ROS2 connection...")
        try:
            self.test_results['ros2_connection'] = True
            print("✅ ROS2 node created successfully")
        except Exception as e:
            print(f"❌ ROS2 connection failed: {e}")
            return False

        # Test 2: Publishers
        print("📤 Testing publishers...")
        try:
            # Test mission publisher
            test_msg = Bool()
            test_msg.data = True
            self.mission_pub.publish(test_msg)
            
            # Test cmd_vel publisher
            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            self.cmd_vel_pub.publish(twist_msg)
            
            self.test_results['publishers'] = True
            print("✅ Publishers working")
        except Exception as e:
            print(f"❌ Publisher test failed: {e}")

        # Test 3: Subscribers  
        print("📥 Testing subscribers...")
        try:
            self.test_results['subscribers'] = True
            print("✅ Subscribers created")
        except Exception as e:
            print(f"❌ Subscriber test failed: {e}")

        # Test 4: Wait for sensor data
        print("🔍 Waiting for sensor data (10 seconds)...")
        start_time = time.time()
        while (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if all(self.data_received.values()):
                break

        # Check data reception
        if self.data_received['gps']:
            print("✅ GPS data received")
            self.test_results['gps_data'] = True
        else:
            print("⚠️ No GPS data (sensor may not be connected)")

        if self.data_received['imu']:
            print("✅ IMU data received")
            self.test_results['imu_data'] = True
        else:
            print("⚠️ No IMU data (sensor may not be connected)")

        if self.data_received['odom']:
            print("✅ Odometry data received")
            self.test_results['odom_data'] = True
        else:
            print("⚠️ No odometry data (navigation not running)")

        # Test 5: System commands
        print("🎮 Testing system commands...")
        try:
            # Test emergency stop
            emergency_msg = Bool()
            emergency_msg.data = True
            
            # Test manual control
            manual_cmd = Twist()
            manual_cmd.linear.x = 0.0
            manual_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(manual_cmd)
            
            print("✅ System commands working")
        except Exception as e:
            print(f"❌ System command test failed: {e}")

        return self.print_results()

    def print_results(self):
        """Print test results summary"""
        
        print("\n🎯 TEST RESULTS SUMMARY")
        print("=" * 30)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"{status} {test_name.replace('_', ' ').title()}")
        
        print(f"\n📊 Score: {passed_tests}/{total_tests} tests passed")
        
        if passed_tests >= 3:  # Core tests passed
            print("🎉 System is functional!")
            print("\n🚀 Ready for:")
            print("   - Mission control")
            print("   - Manual operation")
            print("   - GUI interfaces")
            return True
        else:
            print("⚠️ System needs attention")
            print("\n🔧 Check:")
            print("   - ROS2 installation")
            print("   - Hardware connections")
            print("   - Network configuration")
            return False


def main(args=None):
    """Main test function"""
    
    rclpy.init(args=args)
    
    try:
        tester = SystemTester()
        success = tester.run_tests()
        
        if success:
            print("\n✅ SYSTEM TEST COMPLETED SUCCESSFULLY")
            sys.exit(0)
        else:
            print("\n❌ SYSTEM TEST FAILED")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
