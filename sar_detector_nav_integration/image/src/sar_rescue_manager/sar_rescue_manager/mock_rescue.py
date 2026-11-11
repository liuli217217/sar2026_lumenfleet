"""DETECTOR_NAV_INTEGRATION: simulated action only; publishes no robot commands."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockRescue(Node):
    def __init__(self):
        super().__init__('sar_mock_rescue')
        self.duration = float(self.declare_parameter('duration_sec', 2.).value)
        if not 0 < self.duration <= 30:
            raise ValueError('Mock duration must be 0..30 seconds')
        self.active = None
        self.replies = {}
        self.pub = self.create_publisher(String, '/sar_rescue/action_result', 10)
        self.create_subscription(String, '/sar_rescue/action_request', self.request, 10)
        self.create_timer(.1, self.tick)

    def request(self, msg):
        try:
            request = json.loads(msg.data)
            token = request['token']
            if (request['mode'] != 'mock' or not isinstance(token, str)
                    or not isinstance(request['target_id'], int)):
                return
            if token in self.replies:
                self.pub.publish(String(data=self.replies[token]))
            elif self.active is None:
                self.active = (request, time.monotonic())
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn('Invalid simulated rescue request')

    def tick(self):
        if self.active is None:
            return
        request, start = self.active
        if time.monotonic()-start >= self.duration:
            reply = json.dumps(dict(token=request['token'], target_id=request['target_id'],
                                    mode='mock', success=True))
            self.replies[request['token']] = reply
            if len(self.replies) > 100:
                del self.replies[next(iter(self.replies))]
            self.pub.publish(String(data=reply))
            self.active = None


def main(args=None):
    rclpy.init(args=args)
    node = MockRescue()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
