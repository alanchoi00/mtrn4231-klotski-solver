# pkg_brain/handlers/__init__.py
"""
Brain handlers for the Klotski solver.

This module provides a collection of handlers that process different phases
of the solving pipeline: sensing the board, planning moves, and executing them.
"""

from .base import BaseHandler, BrainNodeLike, HandlerResult
from .execute import ExecuteHandler
from .plan import PlanHandler
from .sense import SenseHandler

__all__ = [
    # Base classes and types
    "BaseHandler",
    "BrainNodeLike",
    "HandlerResult",

    # Handler implementations
    "SenseHandler",
    "PlanHandler",
    "ExecuteHandler",
]
