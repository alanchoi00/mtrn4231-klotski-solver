from __future__ import annotations
from typing import Generic, TypeVar, Type, Dict, ClassVar, Any, cast
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

T = TypeVar("T")


class DeclareParam(Generic[T]):
    """
    Subscriptable parameter declaration helper.

    Usage:
        width = Declare[int](node, "board_width_cells", "Board width in cells")
    """

    python_type: ClassVar[Type[Any]]

    def __class_getitem__(cls, item: Type[T]):
        typed = type(f"Declare_{item.__name__}", (DeclareParam,), {})
        typed.python_type = item
        return typed

    def __new__(cls, node: Node, name: str, description: str) -> T:
        python_type = cls.python_type

        ros_type_map: Dict[Type, int] = {
            int: rclpy.Parameter.Type.INTEGER.value,
            float: rclpy.Parameter.Type.DOUBLE.value,
            bool: rclpy.Parameter.Type.BOOL.value,
            str: rclpy.Parameter.Type.STRING.value,
            list[int]: rclpy.Parameter.Type.INTEGER_ARRAY.value,
            list[float]: rclpy.Parameter.Type.DOUBLE_ARRAY.value,
            list[bool]: rclpy.Parameter.Type.BOOL_ARRAY.value,
            list[str]: rclpy.Parameter.Type.STRING_ARRAY.value,
            list[bytes]: rclpy.Parameter.Type.BYTE_ARRAY.value,
        }

        if python_type not in ros_type_map:
            raise TypeError(f"Unsupported ROS parameter type: {python_type}")

        node.declare_parameter(
            name,
            descriptor=ParameterDescriptor(
                type=ros_type_map[python_type],
                description=description,
                read_only=True,
            ),
        )

        raw = node.get_parameter(name).get_parameter_value()

        val = None

        if python_type is int:
            val = raw.integer_value
            if val is None:
                raise RuntimeError("Parameter integer value is None")
        if python_type is float:
            val = raw.double_value
            if val is None:
                raise RuntimeError("Parameter double value is None")
        if python_type is bool:
            val = raw.bool_value
            if val is None:
                raise RuntimeError("Parameter bool value is None")
        if python_type is str:
            val = raw.string_value
            if val is None:
                raise RuntimeError("Parameter string value is None")
        if python_type is list[int]:
            val = raw.integer_array_value
            if val is None:
                raise RuntimeError("Parameter integer array value is None")
        if python_type is list[float]:
            val = raw.double_array_value
            if val is None:
                raise RuntimeError("Parameter double array value is None")
        if python_type is list[bool]:
            val = raw.bool_array_value
            if val is None:
                raise RuntimeError("Parameter bool array value is None")
        if python_type is list[str]:
            val = raw.string_array_value
            if val is None:
                raise RuntimeError("Parameter string array value is None")
        if python_type is list[bytes]:
            val = raw.byte_array_value
            if val is None:
                raise RuntimeError("Parameter byte array value is None")
        if val is not None:
            return cast(T, val)

        raise RuntimeError("Unhandled parameter type")
