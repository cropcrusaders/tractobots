#!/usr/bin/env python3
"""Mission control UI node.

This lightweight node exposes a very small HTTP interface that allows an
operator to start or stop a mission from a web browser. It publishes a
boolean on ``/mission/start`` where ``True`` means start and ``False``
means stop.  The HTTP server runs in a background thread so the ROS 2
executor can continue spinning.
"""

import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MissionUINode(Node):
    def __init__(self):
        super().__init__('mission_ui_node')

        # Publisher for mission start/stop commands
        self.mission_pub = self.create_publisher(Bool, '/mission/start', 1)
        # Publisher for emergency stop toggle
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 1)

        # Start HTTP server in background
        server_address = ('0.0.0.0', 8088)
        handler = self._make_handler()
        self.httpd = HTTPServer(server_address, handler)
        self.server_thread = threading.Thread(target=self.httpd.serve_forever,
                                              daemon=True)
        self.server_thread.start()
        self.get_logger().info(
            f"Mission UI available at http://{server_address[0]}:{server_address[1]}" )

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send_html(self, html: str):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(html.encode('utf-8'))

            def do_GET(self):
                if self.path != '/':
                    self.send_error(404)
                    return

                html = (
                    "<html><body><h1>Mission Control</h1>"
                    "<form method='post' action='/start'>"
                    "<button type='submit'>Start Mission</button></form>"
                    "<form method='post' action='/stop'>"
                    "<button type='submit'>Stop Mission</button></form>"
                    "<form method='post' action='/estop'>"
                    "<button type='submit'>Emergency Stop</button></form>"
                    "<form method='post' action='/reset'>"
                    "<button type='submit'>Reset E-Stop</button></form>"
                    "</body></html>"
                )
                self._send_html(html)

            def do_POST(self):
                if self.path == '/start':
                    node.mission_pub.publish(Bool(data=True))
                    node.get_logger().info('Mission start requested via UI')
                elif self.path == '/stop':
                    node.mission_pub.publish(Bool(data=False))
                    node.get_logger().info('Mission stop requested via UI')
                elif self.path == '/estop':
                    node.emergency_pub.publish(Bool(data=True))
                    node.get_logger().warn('Emergency stop triggered via UI')
                elif self.path == '/reset':
                    node.emergency_pub.publish(Bool(data=False))
                    node.get_logger().info('Emergency stop cleared via UI')
                else:
                    self.send_error(404)
                    return

                self.send_response(204)
                self.end_headers()

        return Handler

    def destroy_node(self):
        self.httpd.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionUINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
