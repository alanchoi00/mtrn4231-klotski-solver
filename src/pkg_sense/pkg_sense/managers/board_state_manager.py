from typing import TYPE_CHECKING

from klotski_interfaces.msg import BoardState

if TYPE_CHECKING:
    from ..sense_node import Sense


class BoardStateManager:
    """
    Manages board state publishing.
    """

    def __init__(self, node: "Sense"):
        self.node = node
        self.state_pub = self.node.create_publisher(BoardState, "/board_state", 10)

    def publish_board_state(self, state: BoardState):
        self.state_pub.publish(state)
