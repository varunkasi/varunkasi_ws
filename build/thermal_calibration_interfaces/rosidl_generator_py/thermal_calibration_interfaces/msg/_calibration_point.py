# generated from rosidl_generator_py/resource/_idl.py.em
# with input from thermal_calibration_interfaces:msg/CalibrationPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CalibrationPoint(type):
    """Metaclass of message 'CalibrationPoint'."""

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
            module = import_type_support('thermal_calibration_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'thermal_calibration_interfaces.msg.CalibrationPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__calibration_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__calibration_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__calibration_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__calibration_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__calibration_point

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CalibrationPoint(metaclass=Metaclass_CalibrationPoint):
    """Message class 'CalibrationPoint'."""

    __slots__ = [
        '_id',
        '_x',
        '_y',
        '_raw_value',
        '_reference_temp',
        '_timestamp',
    ]

    _fields_and_field_types = {
        'id': 'uint32',
        'x': 'uint16',
        'y': 'uint16',
        'raw_value': 'uint16',
        'reference_temp': 'float',
        'timestamp': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.x = kwargs.get('x', int())
        self.y = kwargs.get('y', int())
        self.raw_value = kwargs.get('raw_value', int())
        self.reference_temp = kwargs.get('reference_temp', float())
        self.timestamp = kwargs.get('timestamp', str())

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
        if self.id != other.id:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.raw_value != other.raw_value:
            return False
        if self.reference_temp != other.reference_temp:
            return False
        if self.timestamp != other.timestamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'id' field must be an unsigned integer in [0, 4294967295]"
        self._id = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'x' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'x' field must be an unsigned integer in [0, 65535]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'y' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'y' field must be an unsigned integer in [0, 65535]"
        self._y = value

    @builtins.property
    def raw_value(self):
        """Message field 'raw_value'."""
        return self._raw_value

    @raw_value.setter
    def raw_value(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'raw_value' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'raw_value' field must be an unsigned integer in [0, 65535]"
        self._raw_value = value

    @builtins.property
    def reference_temp(self):
        """Message field 'reference_temp'."""
        return self._reference_temp

    @reference_temp.setter
    def reference_temp(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'reference_temp' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'reference_temp' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._reference_temp = value

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'timestamp' field must be of type 'str'"
        self._timestamp = value
