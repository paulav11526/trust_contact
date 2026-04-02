# generated from rosidl_generator_py/resource/_idl.py.em
# with input from messages:msg/ForceEvent.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ForceEvent(type):
    """Metaclass of message 'ForceEvent'."""

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
            module = import_type_support('messages')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'messages.msg.ForceEvent')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__force_event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__force_event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__force_event
            cls._TYPE_SUPPORT = module.type_support_msg__msg__force_event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__force_event

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ForceEvent(metaclass=Metaclass_ForceEvent):
    """Message class 'ForceEvent'."""

    __slots__ = [
        '_force_magnitude_1',
        '_timestamp_1_start',
        '_timestamp_1_end',
        '_force_magnitude_2',
        '_timestamp_2_start',
        '_timestamp_2_end',
    ]

    _fields_and_field_types = {
        'force_magnitude_1': 'double',
        'timestamp_1_start': 'builtin_interfaces/Time',
        'timestamp_1_end': 'builtin_interfaces/Time',
        'force_magnitude_2': 'double',
        'timestamp_2_start': 'builtin_interfaces/Time',
        'timestamp_2_end': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.force_magnitude_1 = kwargs.get('force_magnitude_1', float())
        from builtin_interfaces.msg import Time
        self.timestamp_1_start = kwargs.get('timestamp_1_start', Time())
        from builtin_interfaces.msg import Time
        self.timestamp_1_end = kwargs.get('timestamp_1_end', Time())
        self.force_magnitude_2 = kwargs.get('force_magnitude_2', float())
        from builtin_interfaces.msg import Time
        self.timestamp_2_start = kwargs.get('timestamp_2_start', Time())
        from builtin_interfaces.msg import Time
        self.timestamp_2_end = kwargs.get('timestamp_2_end', Time())

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
        if self.force_magnitude_1 != other.force_magnitude_1:
            return False
        if self.timestamp_1_start != other.timestamp_1_start:
            return False
        if self.timestamp_1_end != other.timestamp_1_end:
            return False
        if self.force_magnitude_2 != other.force_magnitude_2:
            return False
        if self.timestamp_2_start != other.timestamp_2_start:
            return False
        if self.timestamp_2_end != other.timestamp_2_end:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def force_magnitude_1(self):
        """Message field 'force_magnitude_1'."""
        return self._force_magnitude_1

    @force_magnitude_1.setter
    def force_magnitude_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'force_magnitude_1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'force_magnitude_1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._force_magnitude_1 = value

    @builtins.property
    def timestamp_1_start(self):
        """Message field 'timestamp_1_start'."""
        return self._timestamp_1_start

    @timestamp_1_start.setter
    def timestamp_1_start(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp_1_start' field must be a sub message of type 'Time'"
        self._timestamp_1_start = value

    @builtins.property
    def timestamp_1_end(self):
        """Message field 'timestamp_1_end'."""
        return self._timestamp_1_end

    @timestamp_1_end.setter
    def timestamp_1_end(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp_1_end' field must be a sub message of type 'Time'"
        self._timestamp_1_end = value

    @builtins.property
    def force_magnitude_2(self):
        """Message field 'force_magnitude_2'."""
        return self._force_magnitude_2

    @force_magnitude_2.setter
    def force_magnitude_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'force_magnitude_2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'force_magnitude_2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._force_magnitude_2 = value

    @builtins.property
    def timestamp_2_start(self):
        """Message field 'timestamp_2_start'."""
        return self._timestamp_2_start

    @timestamp_2_start.setter
    def timestamp_2_start(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp_2_start' field must be a sub message of type 'Time'"
        self._timestamp_2_start = value

    @builtins.property
    def timestamp_2_end(self):
        """Message field 'timestamp_2_end'."""
        return self._timestamp_2_end

    @timestamp_2_end.setter
    def timestamp_2_end(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp_2_end' field must be a sub message of type 'Time'"
        self._timestamp_2_end = value
