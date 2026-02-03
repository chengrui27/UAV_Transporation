# generated from rosidl_generator_py/resource/_idl.py.em
# with input from plan_env:msg/ESDFMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'map_origin'
# Member 'map_voxel_num'
# Member 'local_bound_min'
# Member 'local_bound_max'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ESDFMap(type):
    """Metaclass of message 'ESDFMap'."""

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
            module = import_type_support('plan_env')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'plan_env.msg.ESDFMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__esdf_map
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__esdf_map
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__esdf_map
            cls._TYPE_SUPPORT = module.type_support_msg__msg__esdf_map
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__esdf_map

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ESDFMap(metaclass=Metaclass_ESDFMap):
    """Message class 'ESDFMap'."""

    __slots__ = [
        '_header',
        '_map_origin',
        '_resolution',
        '_map_voxel_num',
        '_local_bound_min',
        '_local_bound_max',
        '_shm_name',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'map_origin': 'double[3]',
        'resolution': 'double',
        'map_voxel_num': 'int32[3]',
        'local_bound_min': 'int32[3]',
        'local_bound_max': 'int32[3]',
        'shm_name': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 3),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int32'), 3),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        if 'map_origin' not in kwargs:
            self.map_origin = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.map_origin = numpy.array(kwargs.get('map_origin'), dtype=numpy.float64)
            assert self.map_origin.shape == (3, )
        self.resolution = kwargs.get('resolution', float())
        if 'map_voxel_num' not in kwargs:
            self.map_voxel_num = numpy.zeros(3, dtype=numpy.int32)
        else:
            self.map_voxel_num = numpy.array(kwargs.get('map_voxel_num'), dtype=numpy.int32)
            assert self.map_voxel_num.shape == (3, )
        if 'local_bound_min' not in kwargs:
            self.local_bound_min = numpy.zeros(3, dtype=numpy.int32)
        else:
            self.local_bound_min = numpy.array(kwargs.get('local_bound_min'), dtype=numpy.int32)
            assert self.local_bound_min.shape == (3, )
        if 'local_bound_max' not in kwargs:
            self.local_bound_max = numpy.zeros(3, dtype=numpy.int32)
        else:
            self.local_bound_max = numpy.array(kwargs.get('local_bound_max'), dtype=numpy.int32)
            assert self.local_bound_max.shape == (3, )
        self.shm_name = kwargs.get('shm_name', str())

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
        if self.header != other.header:
            return False
        if any(self.map_origin != other.map_origin):
            return False
        if self.resolution != other.resolution:
            return False
        if any(self.map_voxel_num != other.map_voxel_num):
            return False
        if any(self.local_bound_min != other.local_bound_min):
            return False
        if any(self.local_bound_max != other.local_bound_max):
            return False
        if self.shm_name != other.shm_name:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def map_origin(self):
        """Message field 'map_origin'."""
        return self._map_origin

    @map_origin.setter
    def map_origin(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'map_origin' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'map_origin' numpy.ndarray() must have a size of 3"
            self._map_origin = value
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
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'map_origin' field must be a set or sequence with length 3 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._map_origin = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def resolution(self):
        """Message field 'resolution'."""
        return self._resolution

    @resolution.setter
    def resolution(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'resolution' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'resolution' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._resolution = value

    @builtins.property
    def map_voxel_num(self):
        """Message field 'map_voxel_num'."""
        return self._map_voxel_num

    @map_voxel_num.setter
    def map_voxel_num(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'map_voxel_num' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 3, \
                "The 'map_voxel_num' numpy.ndarray() must have a size of 3"
            self._map_voxel_num = value
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
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'map_voxel_num' field must be a set or sequence with length 3 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._map_voxel_num = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def local_bound_min(self):
        """Message field 'local_bound_min'."""
        return self._local_bound_min

    @local_bound_min.setter
    def local_bound_min(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'local_bound_min' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 3, \
                "The 'local_bound_min' numpy.ndarray() must have a size of 3"
            self._local_bound_min = value
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
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'local_bound_min' field must be a set or sequence with length 3 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._local_bound_min = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def local_bound_max(self):
        """Message field 'local_bound_max'."""
        return self._local_bound_max

    @local_bound_max.setter
    def local_bound_max(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int32, \
                "The 'local_bound_max' numpy.ndarray() must have the dtype of 'numpy.int32'"
            assert value.size == 3, \
                "The 'local_bound_max' numpy.ndarray() must have a size of 3"
            self._local_bound_max = value
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
                 len(value) == 3 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'local_bound_max' field must be a set or sequence with length 3 and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._local_bound_max = numpy.array(value, dtype=numpy.int32)

    @builtins.property
    def shm_name(self):
        """Message field 'shm_name'."""
        return self._shm_name

    @shm_name.setter
    def shm_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'shm_name' field must be of type 'str'"
        self._shm_name = value
