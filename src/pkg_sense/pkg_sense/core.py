from typing import Type

from pkg_sense.sense_node import Sense


def get_sense_node_type() -> Type[Sense]:
    """Returns the Sense node type to avoid circular imports."""
    return Sense
