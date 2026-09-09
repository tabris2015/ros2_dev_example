# Copyright YEAR Jose Laruta
"""Starter node for lesson LESSON_NN. Replace with the lesson's real node."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TopicNode(Node):
    """Logs a heartbeat once a second."""

    def __init__(self) -> None:
        super().__init__('TOPIC')
        self._count = 0
        self._timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info('TOPIC started')

    def _on_timer(self) -> None:
        """Log and count one heartbeat."""
        self._count += 1
        self.get_logger().info(f'heartbeat #{self._count}')


def main(args: list[str] | None = None) -> None:
    """Entry point: init, spin, and shut down cleanly."""
    rclpy.init(args=args)
    node = TopicNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
