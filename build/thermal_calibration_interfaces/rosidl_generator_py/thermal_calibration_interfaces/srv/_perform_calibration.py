# generated from rosidl_generator_py/resource/_idl.py.em
# with input from thermal_calibration_interfaces:srv/PerformCalibration.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PerformCalibration_Request(type):
    """Metaclass of message 'PerformCalibration_Request'."""

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
                'thermal_calibration_interfaces.srv.PerformCalibration_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__perform_calibration__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__perform_calibration__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__perform_calibration__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__perform_calibration__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__perform_calibration__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PerformCalibration_Request(metaclass=Metaclass_PerformCalibration_Request):
    """Message class 'PerformCalibration_Request'."""

    __slots__ = [
        '_model_type',
        '_degree',
    ]

    _fields_and_field_types = {
        'model_type': 'string',
        'degree': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.model_type = kwargs.get('model_type', str())
        self.degree = kwargs.get('degree', int())

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


# Import statements for member types

# Member 'model_parameters'
import array  # noqa: E402, I100

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_PerformCalibration_Response(type):
    """Metaclass of message 'PerformCalibration_Response'."""

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
                'thermal_calibration_interfaces.srv.PerformCalibration_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__perform_calibration__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__perform_calibration__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__perform_calibration__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__perform_calibration__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__perform_calibration__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PerformCalibration_Response(metaclass=Metaclass_PerformCalibration_Response):
    """Message class 'PerformCalibration_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_model_parameters',
        '_r_squared',
        '_rmse',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'model_parameters': 'sequence<float>',
        'r_squared': 'float',
        'rmse': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.model_parameters = array.array('f', kwargs.get('model_parameters', []))
        self.r_squared = kwargs.get('r_squared', float())
        self.rmse = kwargs.get('rmse', float())

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
        if self.model_parameters != other.model_parameters:
            return False
        if self.r_squared != other.r_squared:
            return False
        if self.rmse != other.rmse:
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
    def model_parameters(self):
        """Message field 'model_parameters'."""
        return self._model_parameters

    @model_parameters.setter
    def model_parameters(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'model_parameters' array.array() must have the type code of 'f'"
            self._model_parameters = value
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
                "The 'model_parameters' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._model_parameters = array.array('f', value)

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


class Metaclass_PerformCalibration(type):
    """Metaclass of service 'PerformCalibration'."""

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
                'thermal_calibration_interfaces.srv.PerformCalibration')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__perform_calibration

            from thermal_calibration_interfaces.srv import _perform_calibration
            if _perform_calibration.Metaclass_PerformCalibration_Request._TYPE_SUPPORT is None:
                _perform_calibration.Metaclass_PerformCalibration_Request.__import_type_support__()
            if _perform_calibration.Metaclass_PerformCalibration_Response._TYPE_SUPPORT is None:
                _perform_calibration.Metaclass_PerformCalibration_Response.__import_type_support__()


class PerformCalibration(metaclass=Metaclass_PerformCalibration):
    from thermal_calibration_interfaces.srv._perform_calibration import PerformCalibration_Request as Request
    from thermal_calibration_interfaces.srv._perform_calibration import PerformCalibration_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
