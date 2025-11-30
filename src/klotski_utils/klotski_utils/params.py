from __future__ import annotations
from typing import Generic, TypeVar, Type, Dict, ClassVar, Any
import rclpy
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

    def __new__(cls, node, name: str, description: str) -> T:
        python_type = cls.python_type

        ros_type_map: Dict[Type, rclpy.Parameter.Type] = {
            int: rclpy.Parameter.Type.INTEGER,
            float: rclpy.Parameter.Type.DOUBLE,
            bool: rclpy.Parameter.Type.BOOL,
            str: rclpy.Parameter.Type.STRING,
            list[int]: rclpy.Parameter.Type.INTEGER_ARRAY,
            list[float]: rclpy.Parameter.Type.DOUBLE_ARRAY,
            list[bool]: rclpy.Parameter.Type.BOOL_ARRAY,
            list[str]: rclpy.Parameter.Type.STRING_ARRAY,
            list[bytes]: rclpy.Parameter.Type.BYTE_ARRAY,
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

        if python_type is int:
            return raw.integer_value
        if python_type is float:
            return raw.double_value
        if python_type is bool:
            return raw.bool_value
        if python_type is str:
            return raw.string_value
        if python_type is list[int]:
            return raw.integer_array_value
        if python_type is list[float]:
            return raw.double_array_value
        if python_type is list[bool]:
            return raw.bool_array_value
        if python_type is list[str]:
            return raw.string_array_value
        if python_type is list[bytes]:
            return raw.byte_array_value

        raise RuntimeError("Unhandled parameter type")
