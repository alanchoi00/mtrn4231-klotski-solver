from typing import TYPE_CHECKING

import tf2_ros

if TYPE_CHECKING:
    from ..sense_node import Sense


class TransformationManager:
    def __init__(self, node: "Sense"):
        self.node = node
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.cached_tf = {}
        self.tf_timer = self.node.create_timer(0.1, self.tf_timer_callback)

    def tf_timer_callback(self):
        """
        Re-broadcast last known transforms so TF frames don't disappear in RViz.
        """
        if not self.cached_tf:
            return
        now = self.node.get_clock().now().to_msg()
        for _, tf_msg in self.cached_tf.items():
            tf_msg.header.stamp = now
            self.tf_broadcaster.sendTransform(tf_msg)

    def send_marker_transform(self, marker_id: int, tf_msg: tf2_ros.TransformStamped):
        self.cached_tf[f'aruco_{marker_id}'] = tf_msg
        self.tf_broadcaster.sendTransform(tf_msg)
