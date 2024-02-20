# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tutorial_interfaces:msg/Board.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Board(type):
    """Metaclass of message 'Board'."""

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
            module = import_type_support('tutorial_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tutorial_interfaces.msg.Board')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__board
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__board
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__board
            cls._TYPE_SUPPORT = module.type_support_msg__msg__board
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__board

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Board(metaclass=Metaclass_Board):
    """Message class 'Board'."""

    __slots__ = [
        '_key_up',
        '_key_down',
        '_key_left',
        '_key_right',
        '_key_left_shift',
        '_key_left_ctrl',
        '_w',
        '_a',
        '_s',
        '_d',
        '_q',
        '_e',
        '_c',
        '_x',
        '_z',
        '_j',
    ]

    _fields_and_field_types = {
        'key_up': 'int32',
        'key_down': 'int32',
        'key_left': 'int32',
        'key_right': 'int32',
        'key_left_shift': 'int32',
        'key_left_ctrl': 'int32',
        'w': 'int32',
        'a': 'int32',
        's': 'int32',
        'd': 'int32',
        'q': 'int32',
        'e': 'int32',
        'c': 'int32',
        'x': 'int32',
        'z': 'int32',
        'j': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.key_up = kwargs.get('key_up', int())
        self.key_down = kwargs.get('key_down', int())
        self.key_left = kwargs.get('key_left', int())
        self.key_right = kwargs.get('key_right', int())
        self.key_left_shift = kwargs.get('key_left_shift', int())
        self.key_left_ctrl = kwargs.get('key_left_ctrl', int())
        self.w = kwargs.get('w', int())
        self.a = kwargs.get('a', int())
        self.s = kwargs.get('s', int())
        self.d = kwargs.get('d', int())
        self.q = kwargs.get('q', int())
        self.e = kwargs.get('e', int())
        self.c = kwargs.get('c', int())
        self.x = kwargs.get('x', int())
        self.z = kwargs.get('z', int())
        self.j = kwargs.get('j', int())

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
        if self.key_up != other.key_up:
            return False
        if self.key_down != other.key_down:
            return False
        if self.key_left != other.key_left:
            return False
        if self.key_right != other.key_right:
            return False
        if self.key_left_shift != other.key_left_shift:
            return False
        if self.key_left_ctrl != other.key_left_ctrl:
            return False
        if self.w != other.w:
            return False
        if self.a != other.a:
            return False
        if self.s != other.s:
            return False
        if self.d != other.d:
            return False
        if self.q != other.q:
            return False
        if self.e != other.e:
            return False
        if self.c != other.c:
            return False
        if self.x != other.x:
            return False
        if self.z != other.z:
            return False
        if self.j != other.j:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def key_up(self):
        """Message field 'key_up'."""
        return self._key_up

    @key_up.setter
    def key_up(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_up' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_up' field must be an integer in [-2147483648, 2147483647]"
        self._key_up = value

    @builtins.property
    def key_down(self):
        """Message field 'key_down'."""
        return self._key_down

    @key_down.setter
    def key_down(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_down' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_down' field must be an integer in [-2147483648, 2147483647]"
        self._key_down = value

    @builtins.property
    def key_left(self):
        """Message field 'key_left'."""
        return self._key_left

    @key_left.setter
    def key_left(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_left' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_left' field must be an integer in [-2147483648, 2147483647]"
        self._key_left = value

    @builtins.property
    def key_right(self):
        """Message field 'key_right'."""
        return self._key_right

    @key_right.setter
    def key_right(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_right' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_right' field must be an integer in [-2147483648, 2147483647]"
        self._key_right = value

    @builtins.property
    def key_left_shift(self):
        """Message field 'key_left_shift'."""
        return self._key_left_shift

    @key_left_shift.setter
    def key_left_shift(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_left_shift' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_left_shift' field must be an integer in [-2147483648, 2147483647]"
        self._key_left_shift = value

    @builtins.property
    def key_left_ctrl(self):
        """Message field 'key_left_ctrl'."""
        return self._key_left_ctrl

    @key_left_ctrl.setter
    def key_left_ctrl(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'key_left_ctrl' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'key_left_ctrl' field must be an integer in [-2147483648, 2147483647]"
        self._key_left_ctrl = value

    @builtins.property
    def w(self):
        """Message field 'w'."""
        return self._w

    @w.setter
    def w(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'w' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'w' field must be an integer in [-2147483648, 2147483647]"
        self._w = value

    @builtins.property
    def a(self):
        """Message field 'a'."""
        return self._a

    @a.setter
    def a(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'a' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'a' field must be an integer in [-2147483648, 2147483647]"
        self._a = value

    @builtins.property
    def s(self):
        """Message field 's'."""
        return self._s

    @s.setter
    def s(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 's' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 's' field must be an integer in [-2147483648, 2147483647]"
        self._s = value

    @builtins.property
    def d(self):
        """Message field 'd'."""
        return self._d

    @d.setter
    def d(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'd' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'd' field must be an integer in [-2147483648, 2147483647]"
        self._d = value

    @builtins.property
    def q(self):
        """Message field 'q'."""
        return self._q

    @q.setter
    def q(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'q' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'q' field must be an integer in [-2147483648, 2147483647]"
        self._q = value

    @builtins.property
    def e(self):
        """Message field 'e'."""
        return self._e

    @e.setter
    def e(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'e' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'e' field must be an integer in [-2147483648, 2147483647]"
        self._e = value

    @builtins.property
    def c(self):
        """Message field 'c'."""
        return self._c

    @c.setter
    def c(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'c' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'c' field must be an integer in [-2147483648, 2147483647]"
        self._c = value

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
            assert value >= -2147483648 and value < 2147483648, \
                "The 'x' field must be an integer in [-2147483648, 2147483647]"
        self._x = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'z' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'z' field must be an integer in [-2147483648, 2147483647]"
        self._z = value

    @builtins.property
    def j(self):
        """Message field 'j'."""
        return self._j

    @j.setter
    def j(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'j' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'j' field must be an integer in [-2147483648, 2147483647]"
        self._j = value
