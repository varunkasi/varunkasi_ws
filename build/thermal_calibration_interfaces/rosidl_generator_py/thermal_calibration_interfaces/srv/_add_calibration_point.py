# generated from rosidl_generator_py/resource/_idl.py.em
# with input from thermal_calibration_interfaces:srv/AddCalibrationPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AddCalibrationPoint_Request(type):
    """Metaclass of message 'AddCalibrationPoint_Request'."""

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
                'thermal_calibration_interfaces.srv.AddCalibrationPoint_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add_calibration_point__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add_calibration_point__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add_calibration_point__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add_calibration_point__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add_calibration_point__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AddCalibrationPoint_Request(metaclass=Metaclass_AddCalibrationPoint_Request):
    """Message class 'AddCalibrationPoint_Request'."""

    __slots__ = [
        '_x',
        '_y',
        '_raw_value',
        '_reference_temp',
    ]

    _fields_and_field_types = {
        'x': 'uint16',
        'y': 'uint16',
        'raw_value': 'uint16',
        'reference_temp': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x = kwargs.get('x', int())
        self.y = kwargs.get('y', int())
        self.raw_value = kwargs.get('raw_value', int())
        self.reference_temp = kwargs.get('reference_temp', float())

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
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.raw_value != other.raw_value:
            return False
        if self.reference_temp != other.reference_temp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_AddCalibrationPoint_Response(type):
    """Metaclass of message 'AddCalibrationPoint_Response'."""

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
                'thermal_calibration_interfaces.srv.AddCalibrationPoint_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__add_calibration_point__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__add_calibration_point__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__add_calibration_point__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__add_calibration_point__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__add_calibration_point__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AddCalibrationPoint_Response(metaclass=Metaclass_AddCalibrationPoint_Response):
    """Message class 'AddCalibrationPoint_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_point_id',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'point_id': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.point_id = kwargs.get('point_id', int())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        if self.point_id != other.point_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def point_id(self):
        """Message field 'point_id'."""
        return self._point_id

    @point_id.setter
    def point_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'point_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'point_id' field must be an unsigned integer in [0, 4294967295]"
        self._point_id = value


class Metaclass_AddCalibrationPoint(type):
    """Metaclass of service 'AddCalibrationPoint'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('thermal_calibration_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'thermal_calibration_interfaces.srv.AddCalibrationPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__add_calibration_point

            from thermal_calibration_interfaces.srv import _add_calibration_point
            if _add_calibration_point.Metaclass_AddCalibrationPoint_Request._TYPE_SUPPORT is None:
                _add_calibration_point.Metaclass_AddCalibrationPoint_Request.__import_type_support__()
            if _add_calibration_point.Metaclass_AddCalibrationPoint_Response._TYPE_SUPPORT is None:
                _add_calibration_point.Metaclass_AddCalibrationPoint_Response.__import_type_support__()


class AddCalibrationPoint(metaclass=Metaclass_AddCalibrationPoint):
    from thermal_calibration_interfaces.srv._add_calibration_point import AddCalibrationPoint_Request as Request
    from thermal_calibration_interfaces.srv._add_calibration_point import AddCalibrationPoint_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
