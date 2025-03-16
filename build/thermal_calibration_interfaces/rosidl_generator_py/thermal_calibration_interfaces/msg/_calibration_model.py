# generated from rosidl_generator_py/resource/_idl.py.em
# with input from thermal_calibration_interfaces:msg/CalibrationModel.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'parameters'
# Member 'raw_value_range'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CalibrationModel(type):
    """Metaclass of message 'CalibrationModel'."""

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
                'thermal_calibration_interfaces.msg.CalibrationModel')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__calibration_model
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__calibration_model
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__calibration_model
            cls._TYPE_SUPPORT = module.type_support_msg__msg__calibration_model
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__calibration_model

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


class CalibrationModel(metaclass=Metaclass_CalibrationModel):
    """Message class 'CalibrationModel'."""

    __slots__ = [
        '_model_type',
        '_degree',
        '_parameters',
        '_r_squared',
        '_rmse',
        '_points_count',
        '_timestamp',
        '_raw_value_range',
    ]

    _fields_and_field_types = {
        'model_type': 'string',
        'degree': 'int8',
        'parameters': 'sequence<float>',
        'r_squared': 'float',
        'rmse': 'float',
        'points_count': 'uint32',
        'timestamp': 'builtin_interfaces/Time',
        'raw_value_range': 'sequence<uint16>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.model_type = kwargs.get('model_type', str())
        self.degree = kwargs.get('degree', int())
        self.parameters = array.array('f', kwargs.get('parameters', []))
        self.r_squared = kwargs.get('r_squared', float())
        self.rmse = kwargs.get('rmse', float())
        self.points_count = kwargs.get('points_count', int())
        from builtin_interfaces.msg import Time
        self.timestamp = kwargs.get('timestamp', Time())
        self.raw_value_range = array.array('H', kwargs.get('raw_value_range', []))

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
        if self.model_type != other.model_type:
            return False
        if self.degree != other.degree:
            return False
        if self.parameters != other.parameters:
            return False
        if self.r_squared != other.r_squared:
            return False
        if self.rmse != other.rmse:
            return False
        if self.points_count != other.points_count:
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.raw_value_range != other.raw_value_range:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def model_type(self):
        """Message field 'model_type'."""
        return self._model_type

    @model_type.setter
    def model_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'model_type' field must be of type 'str'"
        self._model_type = value

    @builtins.property
    def degree(self):
        """Message field 'degree'."""
        return self._degree

    @degree.setter
    def degree(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'degree' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'degree' field must be an integer in [-128, 127]"
        self._degree = value

    @builtins.property
    def parameters(self):
        """Message field 'parameters'."""
        return self._parameters

    @parameters.setter
    def parameters(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'parameters' array.array() must have the type code of 'f'"
            self._parameters = value
            return
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'parameters' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._parameters = array.array('f', value)

    @builtins.property
    def r_squared(self):
        """Message field 'r_squared'."""
        return self._r_squared

    @r_squared.setter
    def r_squared(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r_squared' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'r_squared' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._r_squared = value

    @builtins.property
    def rmse(self):
        """Message field 'rmse'."""
        return self._rmse

    @rmse.setter
    def rmse(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rmse' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'rmse' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._rmse = value

    @builtins.property
    def points_count(self):
        """Message field 'points_count'."""
        return self._points_count

    @points_count.setter
    def points_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'points_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'points_count' field must be an unsigned integer in [0, 4294967295]"
        self._points_count = value

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp' field must be a sub message of type 'Time'"
        self._timestamp = value

    @builtins.property
    def raw_value_range(self):
        """Message field 'raw_value_range'."""
        return self._raw_value_range

    @raw_value_range.setter
    def raw_value_range(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'raw_value_range' array.array() must have the type code of 'H'"
            self._raw_value_range = value
            return
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'raw_value_range' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._raw_value_range = array.array('H', value)
