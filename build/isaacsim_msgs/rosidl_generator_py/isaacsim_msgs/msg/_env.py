# generated from rosidl_generator_py/resource/_idl.py.em
# with input from isaacsim_msgs:msg/Env.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Env(type):
    """Metaclass of message 'Env'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('isaacsim_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'isaacsim_msgs.msg.Env')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__env
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__env
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__env
            cls._TYPE_SUPPORT = module.type_support_msg__msg__env
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__env

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Env(metaclass=Metaclass_Env):
    """Message class 'Env'."""

    __slots__ = [
        '_robots',
        '_environments',
        '_robot_positions',
        '_robot_rotations',
        '_environment_positions',
        '_environment_rotations',
    ]

    _fields_and_field_types = {
        'robots': 'sequence<string>',
        'environments': 'sequence<string>',
        'robot_positions': 'sequence<string>',
        'robot_rotations': 'sequence<string>',
        'environment_positions': 'sequence<string>',
        'environment_rotations': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robots = kwargs.get('robots', [])
        self.environments = kwargs.get('environments', [])
        self.robot_positions = kwargs.get('robot_positions', [])
        self.robot_rotations = kwargs.get('robot_rotations', [])
        self.environment_positions = kwargs.get('environment_positions', [])
        self.environment_rotations = kwargs.get('environment_rotations', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.robots != other.robots:
            return False
        if self.environments != other.environments:
            return False
        if self.robot_positions != other.robot_positions:
            return False
        if self.robot_rotations != other.robot_rotations:
            return False
        if self.environment_positions != other.environment_positions:
            return False
        if self.environment_rotations != other.environment_rotations:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def robots(self):
        """Message field 'robots'."""
        return self._robots

    @robots.setter
    def robots(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'robots' field must be a set or sequence and each value of type 'str'"
        self._robots = value

    @builtins.property
    def environments(self):
        """Message field 'environments'."""
        return self._environments

    @environments.setter
    def environments(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'environments' field must be a set or sequence and each value of type 'str'"
        self._environments = value

    @builtins.property
    def robot_positions(self):
        """Message field 'robot_positions'."""
        return self._robot_positions

    @robot_positions.setter
    def robot_positions(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'robot_positions' field must be a set or sequence and each value of type 'str'"
        self._robot_positions = value

    @builtins.property
    def robot_rotations(self):
        """Message field 'robot_rotations'."""
        return self._robot_rotations

    @robot_rotations.setter
    def robot_rotations(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'robot_rotations' field must be a set or sequence and each value of type 'str'"
        self._robot_rotations = value

    @builtins.property
    def environment_positions(self):
        """Message field 'environment_positions'."""
        return self._environment_positions

    @environment_positions.setter
    def environment_positions(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'environment_positions' field must be a set or sequence and each value of type 'str'"
        self._environment_positions = value

    @builtins.property
    def environment_rotations(self):
        """Message field 'environment_rotations'."""
        return self._environment_rotations

    @environment_rotations.setter
    def environment_rotations(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'environment_rotations' field must be a set or sequence and each value of type 'str'"
        self._environment_rotations = value
