import os
import sys
import types
import importlib.util


def load_driver():
    # Stub required external modules that are unavailable in test env
    stub_modules = {
        'PID': types.ModuleType('PID'),
        'nvector': types.ModuleType('nvector'),
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'rclpy.duration': types.ModuleType('rclpy.duration'),
        'rclpy.qos': types.ModuleType('rclpy.qos'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.ModuleType('std_msgs.msg'),
    }
    class Node: pass
    stub_modules['rclpy.node'].Node = Node
    class Duration:
        def __init__(self, seconds=0):
            self.seconds = seconds
    stub_modules['rclpy.duration'].Duration = Duration
    class QoSProfile:
        def __init__(self, depth=10):
            self.depth = depth
    stub_modules['rclpy.qos'].QoSProfile = QoSProfile
    stub_modules['sensor_msgs.msg'].Joy = type('Joy', (), {})
    stub_modules['std_msgs.msg'].UInt8 = type('UInt8', (), {})
    stub_modules['std_msgs.msg'].Bool = type('Bool', (), {})
    stub_modules['std_msgs.msg'].String = type('String', (), {})
    stub_modules['std_msgs.msg'].Float64 = type('Float64', (), {})

    sys.modules.update(stub_modules)

    driver_path = os.path.join(
        os.path.dirname(__file__), '..', 'tractobots_navigation', 'driver.py'
    )
    spec = importlib.util.spec_from_file_location('driver', driver_path)
    driver = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(driver)
    return driver


def test_minmax():
    driver = load_driver()
    assert driver.minmax(0, 10, -5) == 0
    assert driver.minmax(0, 10, 15) == 10
    assert driver.minmax(0, 10, 7) == 7


def test_reverse_course():
    driver = load_driver()
    assert driver.reverse_course(0) == 180
    assert driver.reverse_course(90) == 270
    assert driver.reverse_course(270) == 90


def test_scaler():
    driver = load_driver()
    sc = driver.Scaler(0, 100, 0, 1)
    assert sc.__call__(0) == 0
    assert sc.__call__(50) == 0.5
    assert sc.__call__(100) == 1
